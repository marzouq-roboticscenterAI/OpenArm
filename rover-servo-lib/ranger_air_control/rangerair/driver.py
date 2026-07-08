"""High-level Ranger Air driver.

Wraps the raw CAN bus with two background threads:

* a **50 Hz transmit heartbeat** that streams the current 0x111 motion command
  (the chassis halts if it does not see a command within 500 ms), and
* a **receive thread** that decodes incoming feedback frames into a
  :class:`RangerState` snapshot you can poll at any time via
  :attr:`RangerAir.state`.

The public surface is small: construct :class:`RangerAir`, call
:meth:`~RangerAir.enable`, then :meth:`~RangerAir.drive` / :meth:`~RangerAir.stop`,
and read :attr:`~RangerAir.state`. Everything is thread-safe.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass

from . import protocol as P
from .bus import CanBus

_TX_PERIOD = 0.02  # 50 Hz — well inside the chassis' 500 ms command timeout


@dataclass
class RangerState:
    """Immutable-ish snapshot of the latest decoded chassis feedback.

    A fresh copy is returned each time you read :attr:`RangerAir.state`, so you
    can hold onto it without worrying about the background thread mutating it.
    Any field may be ``None`` until the corresponding feedback frame has been
    received at least once.

    Attributes:
        connected: ``True`` once any feedback frame has ever been received.
        system: Latest 0x211 system frame (status/mode/voltage/faults), or None.
        motion: Latest 0x221 body-motion frame (v, w, steer), or None.
        motion_mode: Current kinematic mode from 0x291, or None.
        switching_mode: ``True`` while the chassis is mid mode-switch (0x291);
            motion commands are ignored during a switch.
        wheel_speeds: Per-wheel speeds (wheels 1..4) in m/s from 0x281, or None.
        steer_angles: Per-wheel steering angles (motors 5..8) in rad from 0x271,
            or None.
        last_rx_monotonic: ``time.monotonic()`` timestamp of the last frame
            received; useful for staleness/link-loss detection.
    """

    connected: bool = False
    system: P.SystemFeedback | None = None
    motion: P.MotionFeedback | None = None
    motion_mode: P.MotionMode | None = None
    switching_mode: bool = False
    wheel_speeds: tuple[float, float, float, float] | None = None
    steer_angles: tuple[float, float, float, float] | None = None
    last_rx_monotonic: float = 0.0

    @property
    def voltage(self) -> float | None:
        """Battery voltage in volts, or ``None`` if no system frame yet."""
        return self.system.voltage if self.system else None

    @property
    def faults(self) -> list[str]:
        """Human-readable list of active fault descriptions (empty if healthy).

        Returns:
            A list of strings such as ``["EMERGENCY STOP triggered"]``; empty
            when the system frame is absent or reports no faults.
        """
        if not self.system:
            return []
        return P.describe_faults(self.system.fault_bytes)


class RangerAir:
    """Control interface for one Ranger Air chassis over a SocketCAN interface.

    Construction opens a raw CAN socket and (by default) spins up the RX/TX
    background threads immediately. Use it as a context manager so the robot is
    always stopped and returned to standby on exit::

        with RangerAir("can0") as bot:
            bot.wait_for_feedback()
            bot.enable()
            bot.drive(linear=0.1)
            ...

    Args:
        interface: SocketCAN interface name to bind to (e.g. ``"can0"``). If
            ``None`` (default), taken from the ``RANGER_CAN`` environment
            variable, falling back to ``"can0"``. Set ``RANGER_CAN`` (e.g. to a
            udev-stable name like ``"can_rover"``) so this chassis never collides
            with another device's default interface. Must already be up at
            500 kbit/s — see ``setup_can.sh``.
        auto_start: If ``True`` (default), start the RX/TX threads in the
            constructor. Pass ``False`` to defer until you call :meth:`start`.

    Raises:
        OSError: If the CAN interface cannot be bound (usually because it is
            down or does not exist).

    Note:
        Constructing this object transmits nothing. No motion is possible until
        you call :meth:`enable`; before that the TX heartbeat is idle.
    """

    def __init__(self, interface: str | None = None, auto_start: bool = True):
        # Resolve the interface: explicit arg > RANGER_CAN env > legacy "can0".
        interface = interface or os.environ.get("RANGER_CAN", "can0")
        self.interface = interface
        self.bus = CanBus(interface)
        self._state = RangerState()
        self._state_lock = threading.Lock()

        # Current motion command, protected by _cmd_lock. Sent every _TX_PERIOD.
        self._cmd_lock = threading.Lock()
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._cmd_steer = 0.0
        self._mode = P.MotionMode.ACKERMANN
        self._heartbeat_on = False  # only stream 0x111 once enabled

        self._stop_evt = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        if auto_start:
            self.start()

    # ----- lifecycle ------------------------------------------------------ #

    def start(self) -> None:
        """Start the RX (feedback-decode) and TX (heartbeat) threads.

        Called automatically by ``__init__`` unless ``auto_start=False``. Safe
        to leave to the constructor; only call this yourself if you deferred.
        """
        self._rx_thread.start()
        self._tx_thread.start()

    def close(self) -> None:
        """Stop the robot, drop it to standby, join threads, and close the bus.

        Sends a zero-velocity command and a standby control-mode command, then
        tears down the background threads and the CAN socket. Idempotent and
        exception-safe: bus errors during shutdown are swallowed. Invoked
        automatically when used as a context manager.
        """
        try:
            self.stop()
            time.sleep(0.05)
            self.disable()
        except OSError:
            pass
        self._stop_evt.set()
        for t in (self._tx_thread, self._rx_thread):
            if t.is_alive():
                t.join(timeout=1.0)
        self.bus.close()

    def __enter__(self) -> "RangerAir":
        """Enter the context manager; returns ``self``."""
        return self

    def __exit__(self, *exc) -> None:
        """Exit the context manager; calls :meth:`close`."""
        self.close()

    # ----- mode / error commands ------------------------------------------ #

    def enable(self) -> None:
        """Switch the chassis into CAN command mode and arm the TX heartbeat.

        Sends the 0x421 control-mode command (``CAN_COMMAND``) and flips the
        internal heartbeat flag so the TX thread begins streaming 0x111 motion
        frames at 50 Hz. Until this is called, the robot ignores motion commands
        and the heartbeat is idle.

        Returns:
            None.

        Raises:
            OSError: If the CAN frame cannot be sent.

        Note:
            The FS remote has priority. If its SWB switch is not in the TOP
            (command-control) position, the chassis will refuse command mode
            and ``state.system.mode`` will not become ``CAN_COMMAND``.
        """
        self.bus.send(P.CanId.CMD_CONTROL_MODE,
                      P.encode_control_mode(P.ControlMode.CAN_COMMAND))
        with self._cmd_lock:
            self._heartbeat_on = True

    def disable(self) -> None:
        """Disarm the heartbeat and return the chassis to standby mode.

        Zeroes the pending motion command, stops streaming 0x111, and sends the
        0x421 control-mode command (``STANDBY``). The robot will not accept
        motion commands again until :meth:`enable` is called.

        Returns:
            None.

        Raises:
            OSError: If the CAN frame cannot be sent.
        """
        with self._cmd_lock:
            self._heartbeat_on = False
            self._cmd_linear = self._cmd_angular = self._cmd_steer = 0.0
        self.bus.send(P.CanId.CMD_CONTROL_MODE,
                      P.encode_control_mode(P.ControlMode.STANDBY))

    def clear_errors(self, code: int = 0x00) -> None:
        """Clear chassis faults via the 0x441 status-set command.

        Args:
            code: Error-clear selector byte. ``0x00`` (default) clears all
                non-critical faults, including releasing the *e-stop error*
                after the physical button has been twisted out. Other values
                target specific faults, e.g. ``0x01``–``0x08`` clear motor
                driver 1–8 comms faults, ``0x09`` clears battery under-voltage,
                ``0x0A`` clears remote-loss, ``0x0F`` over-current,
                ``0x10`` over-temperature (see manual §3.2 / ``protocol.py``).

        Returns:
            None.

        Raises:
            OSError: If the CAN frame cannot be sent.
        """
        self.bus.send(P.CanId.CMD_CLEAR_ERROR, P.encode_clear_error(code))

    def set_mode(self, mode: P.MotionMode, settle: float = 0.5) -> None:
        """Set the kinematic mode and block briefly while the wheels re-orient.

        Sends the 0x141 mode command and zeroes the pending motion command (the
        chassis ignores motion while switching). Optionally sleeps to let the
        physical mode switch complete before you issue motion.

        Args:
            mode: Target :class:`~rangerair.MotionMode`
                (``ACKERMANN`` / ``TILT`` / ``SPIN`` / ``PARK``).
            settle: Seconds to sleep after sending, giving the steering motors
                time to re-orient. Pass ``0`` to return immediately and instead
                poll :attr:`state`.``switching_mode`` yourself.

        Returns:
            None.

        Raises:
            OSError: If the CAN frame cannot be sent.
        """
        with self._cmd_lock:
            self._mode = mode
            # zero the command while the mode switches (chassis ignores motion then)
            self._cmd_linear = self._cmd_angular = self._cmd_steer = 0.0
        self.bus.send(P.CanId.CMD_MOTION_MODE, P.encode_motion_mode(mode))
        if settle:
            time.sleep(settle)

    def set_light(self, on: bool) -> None:
        """Turn the chassis light bars on or off via the 0x121 light command.

        Args:
            on: ``True`` to switch the lights steady-on, ``False`` for off.

        Returns:
            None.

        Raises:
            OSError: If the CAN frame cannot be sent.
        """
        self.bus.send(P.CanId.CMD_LIGHT, P.encode_light(enable=True, on=on))

    # ----- motion commands ------------------------------------------------ #

    def drive(self, linear: float = 0.0, angular: float = 0.0,
              steer: float = 0.0) -> None:
        """Set the target motion; the 50 Hz heartbeat holds it automatically.

        This call is non-blocking: it only updates the setpoint that the TX
        thread streams. The robot keeps moving at this setpoint until you call
        :meth:`drive` again, :meth:`stop`, :meth:`disable`, or :meth:`close`.
        Values are clamped to the protocol limits for the active mode by
        :func:`rangerair.protocol.encode_motion`.

        Args:
            linear: Forward (+) / reverse (-) body speed in **m/s**. Effective
                in Ackermann and tilt modes. Clamped to ±1.5 m/s.
            angular: Body yaw rate in **rad/s**, counter-clockwise positive.
                Effective in spin mode. Clamped to ±3.259 rad/s.
            steer: Inner steering angle in **rad**, left-turn positive. Clamped
                to ±0.698 rad in Ackermann mode, ±1.571 rad in tilt mode.

        Returns:
            None.

        Note:
            Only meaningful after :meth:`enable`; otherwise the heartbeat is
            idle and nothing is transmitted. ``enable()`` does not need to be
            re-sent between ``drive()`` calls.
        """
        with self._cmd_lock:
            self._cmd_linear = linear
            self._cmd_angular = angular
            self._cmd_steer = steer

    def stop(self) -> None:
        """Command zero velocity while keeping the heartbeat alive.

        Sets the motion setpoint to all-zero so the robot decelerates to a halt,
        but continues streaming 0x111 so the chassis stays in command mode
        (unlike :meth:`disable`, which drops to standby). Prefer this for a
        normal in-motion stop.

        Returns:
            None.
        """
        with self._cmd_lock:
            self._cmd_linear = self._cmd_angular = self._cmd_steer = 0.0

    def park(self) -> None:
        """Engage parking mode — the four wheels lock into an X.

        Convenience wrapper for ``set_mode(MotionMode.PARK)``.

        Returns:
            None.

        Raises:
            OSError: If the CAN frame cannot be sent.
        """
        self.set_mode(P.MotionMode.PARK)

    # ----- state ---------------------------------------------------------- #

    @property
    def state(self) -> RangerState:
        """A thread-safe snapshot copy of the latest decoded feedback.

        Returns:
            A new :class:`RangerState` each access. Safe to read fields from
            without additional locking; the background RX thread will not mutate
            the object you receive.
        """
        with self._state_lock:
            # shallow copy is fine — feedback objects are replaced, not mutated
            s = self._state
            return RangerState(
                connected=s.connected,
                system=s.system,
                motion=s.motion,
                motion_mode=s.motion_mode,
                switching_mode=s.switching_mode,
                wheel_speeds=s.wheel_speeds,
                steer_angles=s.steer_angles,
                last_rx_monotonic=s.last_rx_monotonic,
            )

    def wait_for_feedback(self, timeout: float = 2.0) -> bool:
        """Block until a system-status frame (0x211) has been decoded.

        Use this right after construction to confirm the link is live and to
        guarantee :attr:`state`.``system`` is populated before you read it.

        Args:
            timeout: Maximum seconds to wait for the first system frame.

        Returns:
            ``True`` if a system frame arrived within ``timeout``; ``False`` on
            timeout (chassis unpowered, bus down, or wrong bitrate).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.state.system is not None:
                return True
            time.sleep(0.02)
        return False

    # ----- background loops ----------------------------------------------- #

    def _tx_loop(self) -> None:
        """TX thread: stream the current 0x111 motion command at 50 Hz.

        Only transmits while the heartbeat is armed (after :meth:`enable`).
        Maintains a fixed cadence using an absolute next-tick schedule and
        resyncs if it ever falls behind. Runs until :meth:`close` sets the stop
        event. Send errors are swallowed so a transient bus hiccup does not kill
        the loop.
        """
        next_t = time.monotonic()
        while not self._stop_evt.is_set():
            with self._cmd_lock:
                if self._heartbeat_on:
                    payload = P.encode_motion(
                        self._cmd_linear, self._cmd_angular,
                        self._cmd_steer, self._mode,
                    )
                else:
                    payload = None
            if payload is not None:
                try:
                    self.bus.send(P.CanId.CMD_MOTION, payload)
                except OSError:
                    pass
            next_t += _TX_PERIOD
            sleep = next_t - time.monotonic()
            if sleep > 0:
                self._stop_evt.wait(sleep)
            else:
                next_t = time.monotonic()  # we fell behind; resync

    def _rx_loop(self) -> None:
        """RX thread: receive frames and dispatch them to :meth:`_handle_frame`.

        Blocks on :meth:`rangerair.bus.CanBus.recv` (which honours the socket
        timeout), ignores timeouts, and exits on a socket error or when the stop
        event is set.
        """
        while not self._stop_evt.is_set():
            try:
                frame = self.bus.recv()
            except OSError:
                break
            if frame is None:
                continue
            self._handle_frame(frame)

    def _handle_frame(self, frame) -> None:
        """Decode one feedback frame into the shared :class:`RangerState`.

        Args:
            frame: A :class:`rangerair.bus.CanFrame` received from the bus.

        Unknown IDs and short payloads are ignored. Updates ``connected`` and
        ``last_rx_monotonic`` for every frame so link-liveness tracks all
        traffic, not just the frame types we decode.
        """
        cid, data = frame.can_id, frame.data
        if len(data) < 1:
            return
        with self._state_lock:
            self._state.connected = True
            self._state.last_rx_monotonic = time.monotonic()
            if cid == P.CanId.FB_SYSTEM and len(data) >= 8:
                self._state.system = P.decode_system(data)
            elif cid == P.CanId.FB_MOTION and len(data) >= 8:
                self._state.motion = P.decode_motion(data)
            elif cid == P.CanId.FB_MOTION_MODE and len(data) >= 2:
                mode, switching = P.decode_motion_mode(data)
                self._state.motion_mode = mode
                self._state.switching_mode = switching
            elif cid == P.CanId.FB_WHEEL_SPEEDS and len(data) >= 8:
                self._state.wheel_speeds = P.decode_wheel_speeds(data)
            elif cid == P.CanId.FB_STEER_ANGLES and len(data) >= 8:
                self._state.steer_angles = P.decode_steer_angles(data)
