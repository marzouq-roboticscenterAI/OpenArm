"""One convenient handle for the whole rig: the Ranger Air rover + the DS2-C servo.

:class:`Rover` wraps both cloned libraries behind a single object so a demo (or
the bundled web server) can talk to *both* devices without caring which CAN
adapter is which. It:

* auto-identifies the two USB-CAN adapters (see :mod:`roverservo.interfaces`),
* opens :class:`rangerair.RangerAir` on the rover interface and
  :class:`ds2c.DS2CServo` on the servo interface,
* exposes thin, JSON-friendly control methods for each device,
* runs a **teleop deadman**: rover drive and servo jog only persist while the
  caller keeps refreshing them, and are auto-stopped if the stream goes stale
  (e.g. the browser tab closed), and
* offers :meth:`estop_all` and :meth:`snapshot` for the whole rig at once.

Threading model:

* The rover (:class:`rangerair.RangerAir`) already owns its bus with internal
  RX/TX threads and a thread-safe state snapshot, so rover calls just forward.
* The servo library is **not** thread-safe (one python-can bus, SDO
  request/response matched by polling ``recv``). Concurrent access from several
  HTTP handler threads would interleave SDO responses. So all servo bus I/O is
  funnelled through a single **servo worker thread**: control calls enqueue
  commands, and the worker also refreshes a cached telemetry snapshot. Callers
  never touch the servo bus directly.

Nothing here talks to hardware until :meth:`connect` is called.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Optional

from . import _deps  # noqa: F401  (adds sibling libs to sys.path)
from . import interfaces
from .interfaces import Assignment, InterfaceError

import ds2c
import rangerair
from rangerair import MotionMode


# How long a rover-drive / servo-jog setpoint is honoured without a refresh
# before the deadman zeroes it. The browser refreshes held controls faster than
# this; when it stops (key released, tab closed), motion halts on its own.
DEADMAN_TIMEOUT = 0.5    # seconds
_WATCHDOG_PERIOD = 0.1   # seconds (rover deadman)
_SERVO_TICK = 0.1        # seconds (servo telemetry refresh + jog deadman)


class Rover:
    """Combined control handle for the rover chassis and the servo lift.

    Args:
        rover_iface: Force the rover's CAN interface (skip auto-detect for it).
        servo_iface: Force the servo's CAN interface (skip auto-detect for it).
        servo_node: CANopen node id of the DS2-C servo (default 1).
        max_velocity_pps: Software speed limit passed to the servo library.
        deadman_timeout: Seconds a held setpoint survives without a refresh.
        verbose: Print progress while identifying/connecting.
    """

    def __init__(
        self,
        rover_iface: Optional[str] = None,
        servo_iface: Optional[str] = None,
        servo_node: int = 1,
        max_velocity_pps: Optional[int] = 100_000,
        deadman_timeout: float = DEADMAN_TIMEOUT,
        verbose: bool = True,
    ):
        self.rover_iface = rover_iface
        self.servo_iface = servo_iface
        self.servo_node = servo_node
        self.max_velocity_pps = max_velocity_pps
        self.deadman_timeout = deadman_timeout
        self.verbose = verbose

        self.rover: Optional[rangerair.RangerAir] = None
        self.servo: Optional[ds2c.DS2CServo] = None

        self._lock = threading.RLock()
        self._stop_evt = threading.Event()

        # -- rover teleop bookkeeping (guarded by _lock) --
        self._rover_cmd = (0.0, 0.0, 0.0)     # (linear, angular, steer)
        self._rover_cmd_stamp = 0.0
        self._rover_enabled = False
        self._watchdog: Optional[threading.Thread] = None

        # -- servo worker plumbing --
        self._servo_q: "queue.Queue[tuple]" = queue.Queue()
        self._servo_worker: Optional[threading.Thread] = None
        self._servo_abort = threading.Event()  # preempt an in-progress move
        # servo state cache (guarded by _lock)
        self._servo_cache: dict = {}
        self._servo_status = "idle"
        self._servo_jogging = False
        self._servo_jog_v = 0
        self._servo_jog_stamp = 0.0
        self._servo_moving = False

    # -- logging ------------------------------------------------------------ #
    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[rover] {msg}")

    # -- lifecycle ---------------------------------------------------------- #
    def identify(self, configure: bool = True) -> Assignment:
        """Detect / confirm which interface is which. Returns the assignment."""
        forced = Assignment(rover=self.rover_iface, servo=self.servo_iface)
        if forced.both_found:
            self._log(f"interfaces forced: {forced}")
            if configure:
                interfaces.configure_interface(forced.rover, interfaces.ROVER_BITRATE)
                interfaces.configure_interface(forced.servo, interfaces.SERVO_BITRATE)
            return forced

        found = interfaces.identify(configure=configure, verbose=self.verbose)
        self.rover_iface = self.rover_iface or found.rover
        self.servo_iface = self.servo_iface or found.servo
        return Assignment(rover=self.rover_iface, servo=self.servo_iface)

    def connect(self, require_both: bool = True, configure: bool = True) -> None:
        """Identify the adapters and open both device drivers.

        Args:
            require_both: If ``True`` (default), raise unless *both* devices are
                found. If ``False``, connect to whichever were found.
            configure: Set the interface bitrates via ``sudo`` while probing.

        Raises:
            InterfaceError: If a required device could not be located.
        """
        assign = self.identify(configure=configure)

        if assign.rover:
            self._log(f"opening Ranger Air on {assign.rover}")
            self.rover = rangerair.RangerAir(assign.rover)   # RX/TX threads start
        if assign.servo:
            self._log(f"opening DS2-C servo on {assign.servo}")
            self.servo = ds2c.DS2CServo(
                channel=assign.servo, node=self.servo_node,
                max_velocity_pps=self.max_velocity_pps, verbose=self.verbose,
            ).open()

        if require_both and (self.rover is None or self.servo is None):
            self.close()
            missing = []
            if self.rover is None:
                missing.append("rover (Ranger Air @ 500k)")
            if self.servo is None:
                missing.append("servo (DS2-C @ 1M)")
            raise InterfaceError(
                "could not connect to: " + ", ".join(missing) +
                ". Check power, wiring, the rover remote's SWB switch (must be TOP), "
                "and that both USB-CAN adapters are plugged in. You can also force "
                "the mapping with ROVER_CAN=/SERVO_CAN= environment variables."
            )

        if self.rover is not None:
            if self.rover.wait_for_feedback(timeout=2.0):
                self._log("rover feedback confirmed")
            else:
                self._log("WARNING: rover opened but no feedback yet (powered? remote in TOP?)")

        self._stop_evt.clear()
        if self.rover is not None:
            self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
            self._watchdog.start()
        if self.servo is not None:
            self._servo_worker = threading.Thread(target=self._servo_worker_loop, daemon=True)
            self._servo_worker.start()

    def close(self) -> None:
        """Stop everything and release both buses. Idempotent.

        Tears the worker/watchdog threads down *first* (so nothing else is
        touching the buses), then issues a direct stop to each device. Doing the
        final stop after the join — rather than enqueuing it — avoids any race
        with the worker exiting, and guarantees no two threads share a bus.
        """
        self._servo_abort.set()   # break out of any in-progress blocking move
        self._stop_evt.set()
        for t in (self._watchdog, self._servo_worker):
            if t and t.is_alive():
                t.join(timeout=2.0)
        # threads are gone; safe to drive each bus directly one last time
        if self.rover is not None:
            with self._lock:
                self._rover_cmd = (0.0, 0.0, 0.0)
                self._rover_enabled = False
            try:
                self.rover.stop()
                self.rover.disable()
            except Exception:
                pass
            try:
                self.rover.close()
            except Exception:
                pass
            self.rover = None
        if self.servo is not None:
            try:
                self.servo.clear_estop_latch()
                self.servo.emergency_stop()
            except Exception:
                pass
            try:
                self.servo.close()
            except Exception:
                pass
            self.servo = None

    def __enter__(self) -> "Rover":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ==================================================================== #
    # Rover (Ranger Air) control  — forwarded to the internally-threaded lib
    # ==================================================================== #
    def _need_rover(self) -> rangerair.RangerAir:
        if self.rover is None:
            raise InterfaceError("rover not connected")
        return self.rover

    def rover_enable(self) -> None:
        """Put the chassis in CAN command mode (and clear faults / e-stop)."""
        bot = self._need_rover()
        bot.enable()
        bot.clear_errors()
        with self._lock:
            self._rover_enabled = True

    def rover_disable(self) -> None:
        bot = self._need_rover()
        with self._lock:
            self._rover_enabled = False
            self._rover_cmd = (0.0, 0.0, 0.0)
        bot.disable()

    def rover_clear_errors(self, code: int = 0x00) -> None:
        self._need_rover().clear_errors(code)

    def rover_set_mode(self, mode: str) -> None:
        """Set kinematic mode by name: ackermann / tilt / spin / park."""
        bot = self._need_rover()
        m = MotionMode[mode.strip().upper()]
        with self._lock:
            self._rover_cmd = (0.0, 0.0, 0.0)
        bot.set_mode(m)

    def rover_light(self, on: bool) -> None:
        self._need_rover().set_light(on)

    def rover_drive(self, linear: float = 0.0, angular: float = 0.0,
                    steer: float = 0.0) -> None:
        """Set the rover motion setpoint (deadman-guarded — refresh to hold)."""
        bot = self._need_rover()
        with self._lock:
            self._rover_cmd = (linear, angular, steer)
            self._rover_cmd_stamp = time.monotonic()
        bot.drive(linear=linear, angular=angular, steer=steer)

    def rover_stop(self) -> None:
        bot = self._need_rover()
        with self._lock:
            self._rover_cmd = (0.0, 0.0, 0.0)
        bot.stop()

    # ==================================================================== #
    # Servo (DS2-C) control — enqueued to the single servo worker thread
    # ==================================================================== #
    def _need_servo(self) -> ds2c.DS2CServo:
        if self.servo is None:
            raise InterfaceError("servo not connected")
        return self.servo

    def _servo_enqueue(self, name: str, **kw) -> None:
        self._need_servo()
        self._servo_q.put((name, kw))

    def servo_check(self) -> str:
        """Enqueue a pre-flight (statusword + clear latched fault). Returns cached state."""
        self._servo_enqueue("check")
        with self._lock:
            return self._servo_cache.get("state", "checking…")

    def servo_enable(self) -> None:
        """Enable the drive in position mode, ready for discrete moves."""
        self._servo_enqueue("enable")

    def servo_disable(self) -> None:
        self._servo_enqueue("disable")

    def servo_move(self, pulses: int, velocity: int = 10_000) -> None:
        """Fire a relative move of ``pulses`` (positive = physically DOWN).

        Runs on the servo worker; the HTTP handler returns immediately and the
        UI polls :meth:`snapshot` for progress. Rejected while jogging or while
        another move is running.
        """
        with self._lock:
            if self._servo_jogging:
                raise InterfaceError("servo is jogging — stop the jog before a discrete move")
            if self._servo_moving:
                raise InterfaceError("servo move already in progress")
        self._servo_enqueue("move", pulses=int(pulses), velocity=int(velocity))

    def servo_raise(self, pulses: int, velocity: int = 10_000) -> None:
        """Discrete move physically UP (negative target)."""
        self.servo_move(-abs(int(pulses)), velocity=velocity)

    def servo_lower(self, pulses: int, velocity: int = 10_000) -> None:
        """Discrete move physically DOWN (positive target)."""
        self.servo_move(abs(int(pulses)), velocity=velocity)

    def servo_jog(self, velocity: int) -> None:
        """Continuous jog at signed ``velocity`` pps (deadman-guarded).

        Positive = physically DOWN, negative = UP. Refresh faster than the
        deadman to keep moving; :meth:`servo_stop` / :meth:`servo_disable` ends
        the jog. The stamp is updated here (not just in the worker) so the
        deadman timing tracks the caller's refresh rate exactly.
        """
        with self._lock:
            if self._servo_moving:
                raise InterfaceError("servo is completing a discrete move")
            self._servo_jog_stamp = time.monotonic()
        self._servo_enqueue("jog", velocity=int(velocity))

    def servo_stop(self) -> None:
        """Stop the servo: zero the jog if jogging, else quick-stop."""
        self._servo_enqueue("stop")

    def servo_estop(self) -> None:
        """Servo emergency stop: quick-stop then disable voltage (preempts a move)."""
        self._servo_abort.set()
        self._servo_enqueue("estop")

    def servo_fault_reset(self) -> None:
        self._servo_enqueue("fault_reset")

    # ==================================================================== #
    # Whole-rig
    # ==================================================================== #
    def estop_all(self) -> None:
        """Stop BOTH devices immediately. Safe to call repeatedly."""
        if self.rover is not None:
            with self._lock:
                self._rover_cmd = (0.0, 0.0, 0.0)
                self._rover_enabled = False
            try:
                self.rover.stop()
                self.rover.disable()
            except Exception:
                pass
        if self.servo is not None:
            # preempt any in-progress move, then run the stop on the worker;
            # also drop it in front so it is the next thing executed.
            self._servo_abort.set()
            if self._servo_worker and self._servo_worker.is_alive():
                self._servo_q.put(("estop", {}))
            else:
                # worker not running (e.g. during close) — stop directly
                try:
                    self.servo.clear_estop_latch()
                    self.servo.emergency_stop()
                except Exception:
                    pass

    # ==================================================================== #
    # Rover deadman watchdog
    # ==================================================================== #
    def _watchdog_loop(self) -> None:
        while not self._stop_evt.wait(_WATCHDOG_PERIOD):
            if self.rover is None:
                continue
            now = time.monotonic()
            with self._lock:
                cmd = self._rover_cmd
                stale = (now - self._rover_cmd_stamp) > self.deadman_timeout
            if any(cmd) and stale:
                self._log("deadman: rover setpoint stale — stopping")
                try:
                    self.rover.stop()
                except Exception:
                    pass
                with self._lock:
                    self._rover_cmd = (0.0, 0.0, 0.0)

    # ==================================================================== #
    # Servo worker thread — sole owner of the servo bus
    # ==================================================================== #
    def _servo_worker_loop(self) -> None:
        servo = self.servo
        assert servo is not None
        next_tick = time.monotonic()
        while not self._stop_evt.is_set():
            try:
                name, kw = self._servo_q.get(timeout=0.05)
            except queue.Empty:
                name = None
            if name is not None:
                self._servo_exec(servo, name, kw)
            now = time.monotonic()
            if now >= next_tick:
                next_tick = now + _SERVO_TICK
                self._servo_refresh_telemetry(servo)
                self._servo_jog_deadman(servo, now)

    def _set_servo_status(self, status: str) -> None:
        with self._lock:
            self._servo_status = status

    def _servo_exec(self, servo: "ds2c.DS2CServo", name: str, kw: dict) -> None:
        try:
            if name == "check":
                st = servo.check()
                self._set_servo_status(f"checked: {st.name}")
            elif name == "enable":
                servo.enable()
                servo.configure_position_mode(velocity=10_000, accel=10_000, decel=10_000)
                with self._lock:
                    self._servo_jogging = False
                    self._servo_jog_v = 0
                    self._servo_status = "enabled (position)"
            elif name == "disable":
                with self._lock:
                    self._servo_jogging = False
                    self._servo_jog_v = 0
                servo.disable()
                self._set_servo_status("disabled")
            elif name == "move":
                self._servo_do_move(servo, kw["pulses"], kw["velocity"])
            elif name == "jog":
                self._servo_do_jog(servo, kw["velocity"])
            elif name == "stop":
                with self._lock:
                    jogging = self._servo_jogging
                if jogging:
                    servo.set_velocity(0)
                    with self._lock:
                        self._servo_jog_v = 0
                    self._set_servo_status("jog stopped")
                else:
                    servo.quick_stop()
                    self._set_servo_status("quick-stopped")
            elif name == "estop":
                servo.clear_estop_latch()
                servo.emergency_stop()
                with self._lock:
                    self._servo_jogging = False
                    self._servo_jog_v = 0
                    self._servo_status = "E-STOPPED"
            elif name == "fault_reset":
                servo.fault_reset()
                self._set_servo_status("fault reset")
        except Exception as exc:  # noqa: BLE001 — surface, keep the worker alive
            self._set_servo_status(f"{name} error: {exc}")
        finally:
            self._servo_abort.clear()

    def _servo_do_move(self, servo: "ds2c.DS2CServo", pulses: int, velocity: int) -> None:
        """Blocking relative move, run inside the worker with abort + telemetry."""
        with self._lock:
            self._servo_moving = True
            self._servo_status = f"moving {pulses:+d} pulses"
        self._servo_abort.clear()
        try:
            servo.configure_position_mode(velocity=velocity, accel=50_000, decel=50_000)
            # start the move but don't let the library block; we poll here so we
            # can also refresh telemetry and honour an abort/e-stop.
            servo.move_relative(pulses, wait=False, label="web-move")
            deadline = time.time() + 30.0
            reached = False
            while (time.time() < deadline
                   and not self._servo_abort.is_set()
                   and not self._stop_evt.is_set()):
                self._servo_refresh_telemetry(servo)
                with self._lock:
                    st = self._servo_cache.get("statusword")
                if st is not None and (st & ds2c.SW_TARGET_REACHED):
                    reached = True
                    break
                time.sleep(0.1)
            if self._servo_abort.is_set():
                servo.clear_estop_latch()
                servo.emergency_stop()
                self._set_servo_status("move aborted (E-stop)")
            elif reached:
                # drop controlword bit4 so the next move gets a fresh rising edge
                servo.sdo_download(ds2c.OD_CONTROLWORD, 0, 2, ds2c.CW_ENABLE_OPERATION)
                self._set_servo_status("idle (move done)")
            else:
                servo.emergency_stop()
                self._set_servo_status("move timed out — stopped")
        finally:
            with self._lock:
                self._servo_moving = False

    def _servo_do_jog(self, servo: "ds2c.DS2CServo", velocity: int) -> None:
        with self._lock:
            jogging = self._servo_jogging
        if not jogging:
            servo.configure_velocity_mode(0, accel=50_000, decel=50_000)
            servo.enable()          # starts at zero velocity
            with self._lock:
                self._servo_jogging = True
        servo.set_velocity(velocity)
        with self._lock:
            self._servo_jog_v = velocity
            self._servo_jog_stamp = time.monotonic()
            self._servo_status = f"jogging {velocity:+d} pps"

    def _servo_jog_deadman(self, servo: "ds2c.DS2CServo", now: float) -> None:
        with self._lock:
            jogging = self._servo_jogging
            jog_v = self._servo_jog_v
            stale = (now - self._servo_jog_stamp) > self.deadman_timeout
        if jogging and jog_v != 0 and stale:
            self._log("deadman: servo jog stale — zeroing velocity")
            try:
                servo.set_velocity(0)
            except Exception:
                pass
            with self._lock:
                self._servo_jog_v = 0
                self._servo_status = "jog stopped (deadman)"

    def _servo_refresh_telemetry(self, servo: "ds2c.DS2CServo") -> None:
        """Read live drive data into the cache. Only the worker calls this."""
        cache: dict = {}
        try:
            st = servo.read_state()
            cache.update({
                "state": st.name,
                "statusword": st.statusword,
                "fault": st.fault,
                "op_enabled": st.operation_enabled,
                "target_reached": st.target_reached,
            })
        except Exception as exc:  # noqa: BLE001
            cache["state"] = f"read error: {exc}"
        for key, fn in (("position_pulses", servo.read_position),
                        ("velocity_pps", servo.read_velocity)):
            try:
                cache[key] = int(fn())
            except Exception:
                cache[key] = None
        if cache.get("position_pulses") is not None:
            cache["position_rev"] = round(cache["position_pulses"] / ds2c.PULSES_PER_REV, 4)
        if cache.get("velocity_pps") is not None:
            cache["velocity_rpm"] = round(ds2c.pps_to_rpm(cache["velocity_pps"]), 1)
        with self._lock:
            # preserve statusword etc. keys even if a later read failed
            self._servo_cache.update(cache)

    # ==================================================================== #
    # Unified state snapshot (no bus I/O — reads caches only)
    # ==================================================================== #
    def snapshot(self) -> dict:
        """A JSON-serialisable snapshot of the whole rig, for the web UI."""
        return {"rover": self._rover_snapshot(), "servo": self._servo_snapshot()}

    def _rover_snapshot(self) -> dict:
        if self.rover is None:
            return {"present": False}
        s = self.rover.state
        sysfb = s.system
        with self._lock:
            cmd = self._rover_cmd
            enabled = self._rover_enabled
        stale = (time.monotonic() - s.last_rx_monotonic) if s.last_rx_monotonic else None
        out = {
            "present": True,
            "iface": self.rover_iface,
            "connected": s.connected,
            "link_stale": (stale is not None and stale > 1.0),
            "enabled": enabled,
            "voltage": sysfb.voltage if sysfb else None,
            "control_mode": sysfb.mode.name if sysfb else None,
            "system_ok": (sysfb.normal if sysfb else None),
            "estop": (sysfb.estop if sysfb else None),
            "motion_mode": s.motion_mode.name if s.motion_mode else None,
            "switching_mode": s.switching_mode,
            "faults": s.faults,
            "cmd": {"linear": cmd[0], "angular": cmd[1], "steer": cmd[2]},
            "motion": None,
            "wheel_speeds": list(s.wheel_speeds) if s.wheel_speeds else None,
            "steer_angles": list(s.steer_angles) if s.steer_angles else None,
        }
        if s.motion:
            out["motion"] = {
                "linear": s.motion.linear_mps,
                "angular": s.motion.angular_rps,
                "steer": s.motion.steer_rad,
            }
        return out

    def _servo_snapshot(self) -> dict:
        if self.servo is None:
            return {"present": False}
        with self._lock:
            out = dict(self._servo_cache)
            out.update({
                "present": True,
                "iface": self.servo_iface,
                "node": self.servo_node,
                "status": self._servo_status,
                "jogging": self._servo_jogging,
                "jog_pps": self._servo_jog_v,
                "busy": self._servo_moving,
            })
        return out
