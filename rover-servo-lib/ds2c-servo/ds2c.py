"""
ds2c — a small Python control library for the DS2-C servo drive (CANopen / CiA402).

This is a direct Python port of ``ds2c_test_move.sh`` (by Jack's friend), built on
top of ``python-can`` (SocketCAN backend) instead of shelling out to
``cansend``/``candump``. It speaks the same protocol the script does:

  * CANopen node          : 1 (DIP SW1=ON, SW2=OFF, SW3=OFF)
  * bus rate              : 1,000,000 bps
  * gear ratio            : 10000 pulses/rev
  * CiA402 enable sequence: controlword 6 -> 7 -> 15
  * relative-move start   : controlword 0x5F (bit4 rising edge), then 0x0F
  * quick stop            : controlword 0x0B
  * fault reset           : controlword 0x80 (bit7)

On top of what the shell script did, this library also *reads* live data the
script never touched — actual position (0x6064), actual velocity (0x606C),
mode of operation display (0x6061) and a decoded CiA402 state — via SDO uploads.

------------------------------------------------------------------------------
!!! SAFETY !!!
This is a SOFTWARE stop only. If the CAN adapter is unplugged or the drive
hangs, this library cannot stop the motor. Keep a hand on real mains/DC power
(or a physical E-stop) whenever you run it. Direction is INVERTED on this rig:
positive pulses / positive velocity move the carriage physically DOWN.
Use the raise_*/lower_* helpers if you want unambiguous up/down.
------------------------------------------------------------------------------

Requires: python-can (``pip install python-can``) and a configured SocketCAN
interface (e.g. ``can0``). Bring the interface up once with:

    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up

or call ``DS2CServo.bring_up_interface()`` (which runs those via sudo for you).

Typical use:

    from ds2c import DS2CServo

    with DS2CServo(channel="can0", node=1) as servo:
        servo.check()                     # connectivity + fault check
        servo.enable()                    # CiA402 enable
        servo.move_relative(100)          # 100 pulses (physically DOWN)
        print(servo.read_position())      # live position in pulses
        # quick-stop + disable happens automatically on error / Ctrl+C
"""

from __future__ import annotations

import os
import struct
import subprocess
import time
from dataclasses import dataclass
from typing import Optional

try:
    import can
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "python-can is required: install it with 'pip install python-can'"
    ) from exc


# --- CANopen object dictionary indices (CiA402) -----------------------------
OD_CONTROLWORD = 0x6040          # u16, write
OD_STATUSWORD = 0x6041           # u16, read
OD_MODE_OF_OPERATION = 0x6060    # i8,  write
OD_MODE_DISPLAY = 0x6061         # i8,  read
OD_POSITION_ACTUAL = 0x6064      # i32, read  (pulses)
OD_VELOCITY_ACTUAL = 0x606C      # i32, read  (pulses/s)
OD_TARGET_POSITION = 0x607A      # i32, write (relative, in position mode)
OD_PROFILE_VELOCITY = 0x6081     # u32, write (pulses/s)
OD_PROFILE_ACCEL = 0x6083        # u32, write (pulses/s^2)
OD_PROFILE_DECEL = 0x6084        # u32, write (pulses/s^2)
OD_TARGET_VELOCITY = 0x60FF      # i32, write (pulses/s, profile-velocity mode)
OD_MAX_PROFILE_VELOCITY = 0x607F # u32, write — DRIVE-enforced ceiling on profile velocity
OD_MAX_MOTOR_SPEED = 0x6080      # u32, write — DRIVE-enforced absolute speed ceiling

# --- modes of operation -----------------------------------------------------
MODE_PROFILE_POSITION = 1
MODE_PROFILE_VELOCITY = 3

# --- controlword commands (CiA402) ------------------------------------------
CW_SHUTDOWN = 0x0006             # step 1 of enable
CW_SWITCH_ON = 0x0007            # step 2 of enable
CW_ENABLE_OPERATION = 0x000F     # step 3 of enable / "operational, bit4 low"
CW_START_REL_MOVE = 0x005F       # bit4 set: start a relative move
CW_QUICK_STOP = 0x000B           # quick stop ("急停")
CW_DISABLE_VOLTAGE = 0x0000      # de-energize
CW_FAULT_RESET = 0x0080          # bit7: reset fault

# --- statusword bit masks (CiA402) ------------------------------------------
SW_FAULT = 0x0008
SW_TARGET_REACHED = 0x0400
SW_OPERATION_ENABLED_MASK = 0x006F
SW_OPERATION_ENABLED_VALUE = 0x0027

# CANopen COB-ID bases
SDO_TX_BASE = 0x600              # host -> drive (client)
SDO_RX_BASE = 0x580              # drive -> host (server response)
NMT_COB_ID = 0x000
NMT_START_REMOTE = 0x01          # "enter operational"

# SDO command bytes
_SDO_DOWNLOAD_CMD = {1: 0x2F, 2: 0x2B, 4: 0x23}   # expedited write, by byte-count
_SDO_UPLOAD_REQ = 0x40                             # expedited read request
_SDO_ABORT = 0x80                                  # server abort response

# gearing / mechanical (from the vendor spec, per the shell script)
PULSES_PER_REV = 10000


class DS2CError(Exception):
    """Base class for DS2-C errors."""


class SDOTimeout(DS2CError):
    """No SDO response from the drive within the timeout."""


class SDOAbort(DS2CError):
    """The drive aborted the SDO transfer (returned an abort code)."""

    def __init__(self, index: int, sub: int, code: int):
        self.index, self.sub, self.code = index, sub, code
        super().__init__(
            f"SDO abort on 0x{index:04X}:{sub:02X} — code 0x{code:08X}"
        )


class DriveFault(DS2CError):
    """The drive is in a fault state or failed to reach a required state."""


class SpeedLimitExceeded(DS2CError):
    """A commanded velocity exceeded the configured software speed limit."""

    def __init__(self, requested: int, limit: int):
        self.requested, self.limit = requested, limit
        super().__init__(
            f"commanded velocity {requested} pps exceeds the software speed "
            f"limit of {limit} pps ({pps_to_rpm(requested):.1f} rpm > "
            f"{pps_to_rpm(limit):.1f} rpm). Raise it with set_speed_limit(pps) "
            f"or pass max_velocity_pps=... to DS2CServo(...) if you really mean it."
        )


# --- unit helpers -----------------------------------------------------------
def pps_to_rpm(pps: float) -> float:
    """Convert pulses/s to rev/min using the drive's 10000 pulses/rev gearing."""
    return pps * 60.0 / PULSES_PER_REV


def rpm_to_pps(rpm: float) -> int:
    """Convert rev/min to pulses/s (rounded) using 10000 pulses/rev."""
    return round(rpm * PULSES_PER_REV / 60.0)


# --- decoded CiA402 state ---------------------------------------------------
@dataclass
class DriveState:
    statusword: int

    @property
    def fault(self) -> bool:
        return bool(self.statusword & SW_FAULT)

    @property
    def target_reached(self) -> bool:
        return bool(self.statusword & SW_TARGET_REACHED)

    @property
    def operation_enabled(self) -> bool:
        return (self.statusword & SW_OPERATION_ENABLED_MASK) == SW_OPERATION_ENABLED_VALUE

    @property
    def name(self) -> str:
        s = self.statusword
        # Standard CiA402 state decode (mask 0x006F / 0x004F).
        if (s & 0x004F) == 0x0000:
            return "Not ready to switch on"
        if (s & 0x004F) == 0x0040:
            return "Switch on disabled"
        if (s & 0x006F) == 0x0021:
            return "Ready to switch on"
        if (s & 0x006F) == 0x0023:
            return "Switched on"
        if (s & 0x006F) == 0x0027:
            return "Operation enabled"
        if (s & 0x006F) == 0x0007:
            return "Quick stop active"
        if (s & 0x004F) == 0x000F:
            return "Fault reaction active"
        if (s & 0x004F) == 0x0008:
            return "Fault"
        return "Unknown"

    def __str__(self) -> str:
        flags = []
        if self.fault:
            flags.append("FAULT")
        if self.operation_enabled:
            flags.append("op-enabled")
        if self.target_reached:
            flags.append("target-reached")
        return f"0x{self.statusword:04X} ({self.name}){' [' + ','.join(flags) + ']' if flags else ''}"


class DS2CServo:
    """
    Control a DS2-C servo drive over SocketCAN.

    Parameters
    ----------
    channel : Optional[str]
        SocketCAN interface name. If None (default), taken from the ``DS2C_CAN``
        environment variable, falling back to "can0". Set ``DS2C_CAN`` (e.g. to a
        udev-stable name like "can_servo") so this drive never collides with
        another device's default interface.
    node : int
        CANopen node id (default 1). Use ``detect_node()`` to auto-discover.
    bitrate : int
        Nominal bus bitrate; used only by ``bring_up_interface`` (default 1_000_000).
    sdo_timeout : float
        Seconds to wait for an SDO response (default 1.0).
    max_velocity_pps : Optional[int]
        Software speed limit in pulses/s. Any commanded velocity whose magnitude
        exceeds this raises SpeedLimitExceeded instead of moving. Default 20000
        pps (= 120 rpm at 10000 pulses/rev). Set to None to disable the guard
        (NOT recommended), or lower it (e.g. 2000 = 12 rpm) to feel safe.
        This is a client-side check; for a limit the DRIVE itself enforces, also
        call apply_drive_speed_limit().
    verbose : bool
        Print a timestamped log line for each significant action (default True).
    """

    def __init__(
        self,
        channel: Optional[str] = None,
        node: int = 1,
        bitrate: int = 1_000_000,
        sdo_timeout: float = 1.0,
        max_velocity_pps: Optional[int] = 20_000,
        verbose: bool = True,
    ):
        # Resolve the interface: explicit arg > DS2C_CAN env > legacy "can0".
        self.channel = channel or os.environ.get("DS2C_CAN", "can0")
        self.node = node
        self.bitrate = bitrate
        self.sdo_timeout = sdo_timeout
        self.max_velocity_pps = max_velocity_pps
        self.verbose = verbose
        self.bus: Optional[can.BusABC] = None
        self._estopped = False

    # -- lifecycle -----------------------------------------------------------
    def open(self) -> "DS2CServo":
        """Open the SocketCAN bus. The interface must already be up."""
        if self.bus is None:
            self.bus = can.Bus(interface="socketcan", channel=self.channel)
            self._log(f"opened {self.channel}")
        return self

    def close(self) -> None:
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

    def __enter__(self) -> "DS2CServo":
        return self.open()

    def __exit__(self, exc_type, exc, tb) -> None:
        # Mirror the shell script's trap: quick-stop + disable on any error/Ctrl+C.
        if exc_type is not None:
            try:
                self.emergency_stop()
            except Exception:
                pass
        self.close()

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _require_bus(self) -> can.BusABC:
        if self.bus is None:
            raise DS2CError("bus is not open — call open() or use 'with DS2CServo(...)'")
        return self.bus

    # -- interface setup (needs sudo; optional convenience) ------------------
    def bring_up_interface(self) -> None:
        """
        Configure and bring the SocketCAN interface up at ``self.bitrate``.
        Runs 'sudo ip link ...', identical to what the shell script does.
        Skips work if the interface is already up at the right bitrate.
        """
        try:
            details = subprocess.run(
                ["ip", "-details", "link", "show", self.channel],
                capture_output=True, text=True,
            ).stdout
            if f"bitrate {self.bitrate}" in details and "state UP" in details:
                self._log(f"{self.channel} already up at {self.bitrate}bps")
                return
        except FileNotFoundError:
            raise DS2CError("'ip' command not found")

        self._log(f"bringing up {self.channel} at {self.bitrate}bps (needs sudo)")
        subprocess.run(["sudo", "ip", "link", "set", self.channel, "down"],
                       check=False)
        subprocess.run(
            ["sudo", "ip", "link", "set", self.channel, "type", "can",
             "bitrate", str(self.bitrate)], check=True)
        subprocess.run(["sudo", "ip", "link", "set", self.channel, "up"],
                       check=True)

    # -- low-level SDO -------------------------------------------------------
    def _send(self, arb_id: int, data: bytes) -> None:
        bus = self._require_bus()
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

    def sdo_download(self, index: int, sub: int, size: int, value: int) -> None:
        """Expedited SDO write (1, 2 or 4 bytes). ``value`` may be signed."""
        if size not in _SDO_DOWNLOAD_CMD:
            raise ValueError(f"bad SDO size {size}")
        payload = (value & ((1 << (8 * size)) - 1)).to_bytes(size, "little")
        payload = payload.ljust(4, b"\x00")
        data = bytes([_SDO_DOWNLOAD_CMD[size], index & 0xFF, (index >> 8) & 0xFF, sub]) + payload
        self._send(SDO_TX_BASE + self.node, data)
        # Read (and validate) the server's download response, if any.
        self._await_sdo_response(index, sub, expect_abort_only=True)

    def sdo_upload(self, index: int, sub: int) -> bytes:
        """
        Expedited SDO read. Returns the raw data bytes (1..4).
        Raises SDOTimeout or SDOAbort on failure.
        """
        req = bytes([_SDO_UPLOAD_REQ, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0])
        self._send(SDO_TX_BASE + self.node, req)
        return self._await_sdo_response(index, sub)

    def _await_sdo_response(
        self, index: int, sub: int, expect_abort_only: bool = False
    ) -> bytes:
        """
        Wait for the matching SDO server response (COB-ID 0x580+node).
        On a download we only care about spotting an abort; on an upload we
        return the payload bytes. Ignores unrelated traffic (heartbeats, etc.).
        """
        bus = self._require_bus()
        deadline = time.time() + self.sdo_timeout
        while time.time() < deadline:
            msg = bus.recv(timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if msg.arbitration_id != SDO_RX_BASE + self.node:
                continue
            d = msg.data
            if len(d) < 4:
                continue
            resp_index = d[1] | (d[2] << 8)
            if resp_index != index or d[3] != sub:
                continue  # a response for some other object; keep waiting
            cmd = d[0]
            if cmd == _SDO_ABORT:
                code = int.from_bytes(d[4:8], "little")
                raise SDOAbort(index, sub, code)
            if expect_abort_only:
                return b""
            # Expedited upload: 'n' = number of *unused* bytes is encoded in bits
            # 2..3 for scs=2; if size flag not set, fall back to 4 data bytes.
            if cmd & 0x02:  # size indicated
                n = (cmd >> 2) & 0x03
                nbytes = 4 - n
            else:
                nbytes = 4
            return bytes(d[4:4 + nbytes])
        if expect_abort_only:
            return b""  # download with no confirmation seen — treat as best-effort
        raise SDOTimeout(
            f"no SDO response for 0x{index:04X}:{sub:02X} from node {self.node} "
            f"(check wiring / node id / bitrate)"
        )

    # -- typed reads ---------------------------------------------------------
    def read_statusword(self) -> int:
        return int.from_bytes(self.sdo_upload(OD_STATUSWORD, 0)[:2], "little", signed=False)

    def read_state(self) -> DriveState:
        return DriveState(self.read_statusword())

    def read_position(self) -> int:
        """Actual position in pulses (0x6064, signed). NOT read by the shell script."""
        return int.from_bytes(self.sdo_upload(OD_POSITION_ACTUAL, 0)[:4], "little", signed=True)

    def read_velocity(self) -> int:
        """Actual velocity in pulses/s (0x606C, signed)."""
        return int.from_bytes(self.sdo_upload(OD_VELOCITY_ACTUAL, 0)[:4], "little", signed=True)

    def read_mode(self) -> int:
        """Current mode of operation display (0x6061, signed 8-bit)."""
        return int.from_bytes(self.sdo_upload(OD_MODE_DISPLAY, 0)[:1], "little", signed=True)

    def read_position_rev(self) -> float:
        """Convenience: actual position expressed in revolutions."""
        return self.read_position() / PULSES_PER_REV

    # -- NMT / enable / faults ----------------------------------------------
    def nmt_enter_operational(self) -> None:
        self._send(NMT_COB_ID, bytes([NMT_START_REMOTE, self.node]))

    def fault_reset(self) -> None:
        self._log("sending fault reset (controlword bit7)")
        self.sdo_download(OD_CONTROLWORD, 0, 2, CW_FAULT_RESET)
        time.sleep(0.2)

    def check(self, reset_fault: bool = True) -> DriveState:
        """
        Read the statusword before doing anything (mirrors the script's
        pre-flight check). Optionally clears a latched fault once.
        Returns the resulting DriveState. Raises on unresolved fault.
        """
        state = self.read_state()
        self._log(f"statusword = {state}")
        if state.fault and reset_fault:
            self.fault_reset()
            state = self.read_state()
            if state.fault:
                raise DriveFault(f"drive still faulted ({state}) — resolve before moving")
            self._log(f"fault cleared, statusword = {state}")
        elif state.fault:
            raise DriveFault(f"drive is faulted ({state})")
        return state

    def enable(self, enter_operational: bool = True) -> DriveState:
        """
        Run the CiA402 enable sequence (controlword 6 -> 7 -> 15, 100ms apart)
        and verify the drive reaches 'operation enabled'. Enters NMT operational
        first unless told otherwise.
        """
        if enter_operational:
            self._log("entering NMT operational")
            self.nmt_enter_operational()
            time.sleep(0.1)
        self._log("enabling drive (controlword 6 -> 7 -> 15)")
        for cw in (CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPERATION):
            self.sdo_download(OD_CONTROLWORD, 0, 2, cw)
            time.sleep(0.1)
        state = self.read_state()
        self._log(f"statusword after enable = {state}")
        if not state.operation_enabled:
            self.emergency_stop()
            raise DriveFault(
                f"drive did not reach 'operation enabled' ({state}) — stopped for safety"
            )
        return state

    def disable(self) -> None:
        """De-energize the drive (controlword = disable voltage)."""
        self._log("disabling drive (disable voltage)")
        self.sdo_download(OD_CONTROLWORD, 0, 2, CW_DISABLE_VOLTAGE)

    # -- stop paths ----------------------------------------------------------
    def quick_stop(self) -> None:
        """Quick-stop (controlword 0x0B). Motor decelerates on its quick-stop ramp."""
        self.sdo_download(OD_CONTROLWORD, 0, 2, CW_QUICK_STOP)

    def emergency_stop(self) -> None:
        """
        Quick-stop then disable voltage. Idempotent. This is the software E-stop
        the shell script's trap fires — it is NOT a substitute for real power cut.
        """
        if self._estopped:
            return
        self._estopped = True
        self._log("!!! EMERGENCY STOP !!!")
        try:
            self.quick_stop()
            time.sleep(0.05)
            self.disable()
        finally:
            self._log("quick-stop + disable sent — verify the motor has actually stopped")

    def clear_estop_latch(self) -> None:
        """Reset the internal 'already e-stopped' latch so E-stop can fire again."""
        self._estopped = False

    # -- speed limiting ------------------------------------------------------
    def set_speed_limit(self, max_velocity_pps: Optional[int]) -> None:
        """
        Set (or clear, with None) the software speed limit in pulses/s. Every
        subsequent commanded velocity is checked against this before it is sent.
        """
        self.max_velocity_pps = max_velocity_pps
        if max_velocity_pps is None:
            self._log("software speed limit DISABLED")
        else:
            self._log(
                f"software speed limit = {max_velocity_pps} pps "
                f"({pps_to_rpm(max_velocity_pps):.1f} rpm)"
            )

    def _check_velocity(self, velocity: int) -> int:
        """Raise SpeedLimitExceeded if |velocity| is over the software limit."""
        if self.max_velocity_pps is not None and abs(velocity) > self.max_velocity_pps:
            raise SpeedLimitExceeded(velocity, self.max_velocity_pps)
        return velocity

    def apply_drive_speed_limit(self, max_velocity_pps: Optional[int] = None) -> None:
        """
        Program a DRIVE-ENFORCED speed ceiling by writing max profile velocity
        (0x607F) and max motor speed (0x6080). Unlike the software check, the
        drive clamps to this internally, so it holds even against commands that
        bypass this library. Defaults to the current software limit.

        Raises SDOAbort if this drive doesn't implement these (optional) objects
        — in which case only the software limit is available. Run once after
        open(), before enabling.
        """
        limit = self.max_velocity_pps if max_velocity_pps is None else max_velocity_pps
        if limit is None:
            raise DS2CError("no speed limit set — pass max_velocity_pps or set_speed_limit() first")
        self._log(
            f"programming drive-enforced speed ceiling = {limit} pps "
            f"({pps_to_rpm(limit):.1f} rpm) into 0x607F / 0x6080"
        )
        self.sdo_download(OD_MAX_PROFILE_VELOCITY, 0, 4, limit)
        self.sdo_download(OD_MAX_MOTOR_SPEED, 0, 4, limit)

    # -- motion: profile position (relative) --------------------------------
    def configure_position_mode(
        self, velocity: int = 200, accel: int = 1000, decel: int = 1000
    ) -> None:
        """Set profile-position mode + profile velocity/accel/decel (pulses, pulses/s...)."""
        self._check_velocity(velocity)
        self._log(f"position mode: vel={velocity} pps, accel/decel={accel}/{decel} pps^2")
        self.sdo_download(OD_MODE_OF_OPERATION, 0, 1, MODE_PROFILE_POSITION)
        self.sdo_download(OD_PROFILE_VELOCITY, 0, 4, velocity)
        self.sdo_download(OD_PROFILE_ACCEL, 0, 4, accel)
        self.sdo_download(OD_PROFILE_DECEL, 0, 4, decel)

    def move_relative(
        self,
        pulses: int,
        velocity: Optional[int] = None,
        accel: Optional[int] = None,
        decel: Optional[int] = None,
        wait: bool = True,
        timeout: float = 10.0,
        label: str = "move",
    ) -> bool:
        """
        Perform one relative move of ``pulses`` (positive = physically DOWN).

        If velocity/accel/decel are given, profile-position mode is (re)configured
        first. Otherwise the drive must already be in position mode (call
        ``configure_position_mode`` or ``enable``+config once).

        Returns True if 'target reached' was seen (or wait=False). On timeout it
        fires an emergency stop and raises DriveFault.
        """
        if velocity is not None or accel is not None or decel is not None:
            self.configure_position_mode(
                velocity if velocity is not None else 200,
                accel if accel is not None else 1000,
                decel if decel is not None else 1000,
            )
        self._log(f"moving '{label}': {pulses} pulses relative")
        self.sdo_download(OD_TARGET_POSITION, 0, 4, pulses)
        # Rising edge on controlword bit4 starts the relative move.
        self.sdo_download(OD_CONTROLWORD, 0, 2, CW_START_REL_MOVE)

        if not wait:
            return True

        self._log(f"polling for 'target reached' (bit10), timeout {timeout}s")
        deadline = time.time() + timeout
        reached = False
        while time.time() < deadline:
            try:
                if self.read_state().target_reached:
                    reached = True
                    break
            except SDOTimeout:
                pass
            time.sleep(0.1)

        if reached:
            self._log(f"'{label}' reached target")
            # Drop bit4 so the next move gets its own rising edge.
            self.sdo_download(OD_CONTROLWORD, 0, 2, CW_ENABLE_OPERATION)
            return True

        self._log(f"timed out waiting for '{label}' — stopping for safety")
        self.emergency_stop()
        raise DriveFault(f"'{label}' did not reach target within {timeout}s")

    def move_absolute(
        self, target_pulses: int, wait: bool = True, timeout: float = 10.0, **kw
    ) -> bool:
        """
        Move to an absolute position by reading current position and issuing the
        equivalent relative move. (The drive is driven in relative mode, matching
        the shell script; this computes the delta for you.)
        """
        current = self.read_position()
        delta = target_pulses - current
        return self.move_relative(delta, wait=wait, timeout=timeout, label="abs", **kw)

    # -- motion: profile velocity (continuous jog) --------------------------
    def configure_velocity_mode(
        self, velocity: int, accel: int = 50000, decel: int = 50000
    ) -> None:
        """
        Set profile-velocity mode + target velocity/accel/decel.

        NOTE (from the vendor manual, echoed in the shell script): in profile
        velocity mode the motor starts moving as soon as the enable sequence
        completes — there is no separate edge-triggered start. So this must be
        called BEFORE enable() when you want a jog. ``velocity`` is signed;
        positive = physically DOWN.
        """
        self._check_velocity(velocity)
        self._log(f"velocity mode: vel={velocity} pps, accel/decel={accel}/{decel} pps^2")
        self.sdo_download(OD_MODE_OF_OPERATION, 0, 1, MODE_PROFILE_VELOCITY)
        self.sdo_download(OD_PROFILE_ACCEL, 0, 4, accel)
        self.sdo_download(OD_PROFILE_DECEL, 0, 4, decel)
        self.sdo_download(OD_TARGET_VELOCITY, 0, 4, velocity)

    def set_velocity(self, velocity: int) -> None:
        """Update target velocity on the fly (profile-velocity mode). Signed pps."""
        self._check_velocity(velocity)
        self.sdo_download(OD_TARGET_VELOCITY, 0, 4, velocity)

    def jog(
        self,
        velocity: int,
        accel: int = 20000,
        decel: int = 20000,
        max_seconds: Optional[float] = 20.0,
    ) -> None:
        """
        Continuous profile-velocity move. Configures velocity mode, enables the
        drive, and (if ``max_seconds`` is set) blocks up to that long before
        auto-stopping — a safety cap matching ``--jog-up``'s JOG_MAX_SECONDS.
        Pass ``max_seconds=None`` to configure+start and return immediately;
        you are then responsible for calling ``emergency_stop()``.

        ``velocity`` is signed: positive = physically DOWN, negative = UP.
        """
        self.configure_velocity_mode(velocity, accel, decel)
        time.sleep(0.05)
        self.enable()  # motor starts moving here
        if max_seconds is None:
            return
        self._log(f"jogging — auto-stop after {max_seconds}s")
        try:
            deadline = time.time() + max_seconds
            while time.time() < deadline:
                time.sleep(0.1)
        finally:
            self.emergency_stop()

    # -- direction-aware convenience (accounts for inverted polarity) --------
    # Per the shell script: positive pulses/velocity drive the carriage DOWN.
    # These helpers give you unambiguous physical up/down.
    def raise_by(self, pulses: int, **kw) -> bool:
        """Raise the robot by ``pulses`` (physically UP = negative target)."""
        return self.move_relative(-abs(pulses), label="raise", **kw)

    def lower_by(self, pulses: int, **kw) -> bool:
        """Lower the robot by ``pulses`` (physically DOWN = positive target)."""
        return self.move_relative(abs(pulses), label="lower", **kw)

    def jog_up(self, velocity: int = 5000, **kw) -> None:
        """Continuous jog UP (negative velocity). Default 5000 pps = 30 rpm."""
        self.jog(-abs(velocity), **kw)

    def jog_down(self, velocity: int = 5000, **kw) -> None:
        """Continuous jog DOWN (positive velocity). Default 5000 pps = 30 rpm."""
        self.jog(abs(velocity), **kw)

    # -- node discovery ------------------------------------------------------
    def detect_node(self, timeout: float = 3.0) -> Optional[int]:
        """
        Listen for a heartbeat/bootup frame (COB-ID 0x700-0x77F) and set/return
        the node id, mirroring the shell script's auto-detect. Returns None if
        nothing is heard (node id left unchanged).
        """
        bus = self._require_bus()
        self._log(f"listening {timeout}s for a heartbeat/bootup frame (700h-77Fh)")
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = bus.recv(timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if 0x700 <= msg.arbitration_id <= 0x77F:
                self.node = msg.arbitration_id - 0x700
                self._log(f"detected node id {self.node} (on {msg.arbitration_id:03X}h)")
                return self.node
        self._log("no heartbeat seen — node id left unchanged")
        return None
