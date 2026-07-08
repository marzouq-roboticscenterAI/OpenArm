"""CAN protocol definitions for the AgileX Ranger Air chassis.

All values are taken from the "Ranger Air" user manual, section 3.2
"CAN Interface Protocol".  The bus is CAN 2.0B, 500 kbit/s, and every
multi-byte field is big-endian ("MOTOROLA" format), signed unless noted.

Only the standard-library ``struct`` module is used here so the whole
package stays dependency-free.
"""

from __future__ import annotations

import enum
import struct
from dataclasses import dataclass, field

BITRATE = 500_000  # CAN bus baud rate (fixed by the chassis)


class CanId(enum.IntEnum):
    """CAN frame identifiers used by the Ranger Air."""

    # ---- Commands: host -> chassis ----
    CMD_MOTION = 0x111        # linear vel + angular vel + steering angle (DLC 8, 50 Hz)
    CMD_LIGHT = 0x121         # front/rear light control (DLC 8)
    CMD_MOTION_MODE = 0x141   # ackermann / tilt / spin / park (DLC 1)
    CMD_CONTROL_MODE = 0x421  # standby vs. CAN command mode (DLC 1)
    CMD_CLEAR_ERROR = 0x441   # clear faults / release e-stop (DLC 1)

    # ---- Feedback: chassis -> host ----
    FB_SYSTEM = 0x211         # system status, mode, voltage, fault bits
    FB_MOTION = 0x221         # body linear/angular velocity + steering angle
    FB_LIGHT = 0x231          # light control state
    FB_MOTION_MODE = 0x291    # current motion mode + switching flag
    FB_ACTUATOR_HS_BASE = 0x251  # 0x251..0x258 motor speed/current/position
    FB_ACTUATOR_LS_BASE = 0x261  # 0x261..0x268 motor voltage/temp/status
    FB_STEER_ANGLES = 0x271   # per-wheel steering angles (motors 5..8)
    FB_WHEEL_SPEEDS = 0x281   # per-wheel speeds (wheels 1..4)


class ControlMode(enum.IntEnum):
    """Value for the 0x421 control-mode command / high nibble of feedback."""

    STANDBY = 0x00       # default on power-up; only accepts mode commands
    CAN_COMMAND = 0x01   # accept motion/light commands over CAN


class SystemMode(enum.IntEnum):
    """byte[1] of the 0x211 system feedback frame."""

    STANDBY = 0x00
    CAN_COMMAND = 0x01
    REMOTE_CONTROL = 0x03


class MotionMode(enum.IntEnum):
    """Chassis kinematic mode (0x141 command / 0x291 feedback)."""

    ACKERMANN = 0x00  # front/rear Ackermann steering
    TILT = 0x01       # diagonal / parallel translation ("slide")
    SPIN = 0x02       # rotate in place
    PARK = 0x03       # wheels locked in an X (parking brake)


@dataclass(frozen=True)
class _Limits:
    """Command limits from the protocol tables (SI units)."""

    linear_mps: float = 1.5            # ±1500 mm/s
    linear_mps_steered: float = 0.525  # ±525 mm/s when steering angle > 20 deg
    angular_rps: float = 3.259         # ±3259 mrad/s (spin mode)
    steer_ackermann_rad: float = 0.698  # ±698 mrad
    steer_tilt_rad: float = 1.571       # ±1571 mrad (~90 deg)


LIMITS = _Limits()


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def encode_motion(
    linear_mps: float = 0.0,
    angular_rps: float = 0.0,
    steer_rad: float = 0.0,
    mode: MotionMode = MotionMode.ACKERMANN,
) -> bytes:
    """Build the 8-byte payload for a 0x111 motion command.

    The chassis expects big-endian ("MOTOROLA") signed 16-bit fields:
    ``[lin_hi lin_lo][ang_hi ang_lo][rsv rsv][steer_hi steer_lo]``. Inputs are
    clamped to the protocol limits for ``mode`` before scaling to the chassis'
    integer units (mm/s, mrad/s, mrad).

    Args:
        linear_mps: Forward (+) / reverse (-) body speed in m/s. Clamped to
            ±1.5 m/s. Effective in Ackermann and tilt modes.
        angular_rps: Body yaw rate in rad/s, CCW positive. Clamped to
            ±3.259 rad/s. Effective in spin mode.
        steer_rad: Inner steering angle in rad, left-turn positive. Clamped to
            ±0.698 rad (Ackermann) or ±1.571 rad (tilt), selected by ``mode``.
        mode: Active :class:`MotionMode`; only used here to pick the steering
            clamp limit.

    Returns:
        Exactly 8 bytes ready to send with CAN id ``CanId.CMD_MOTION``.
    """
    steer_limit = (
        LIMITS.steer_tilt_rad if mode == MotionMode.TILT else LIMITS.steer_ackermann_rad
    )
    lin = int(round(_clamp(linear_mps, LIMITS.linear_mps) * 1000.0))       # mm/s
    ang = int(round(_clamp(angular_rps, LIMITS.angular_rps) * 1000.0))     # mrad/s
    steer = int(round(_clamp(steer_rad, steer_limit) * 1000.0))           # mrad
    # >h h xx h  -> linear, angular, 2 reserved bytes, steering (all big-endian)
    return struct.pack(">hhxxh", lin, ang, steer)


def encode_control_mode(mode: ControlMode) -> bytes:
    """Build the 1-byte payload for a 0x421 control-mode command.

    Args:
        mode: :class:`ControlMode.STANDBY` or :class:`ControlMode.CAN_COMMAND`.

    Returns:
        A single byte for CAN id ``CanId.CMD_CONTROL_MODE``.
    """
    return bytes([int(mode)])


def encode_motion_mode(mode: MotionMode) -> bytes:
    """Build the 1-byte payload for a 0x141 motion-mode command.

    Args:
        mode: Target :class:`MotionMode` (ackermann/tilt/spin/park).

    Returns:
        A single byte for CAN id ``CanId.CMD_MOTION_MODE``.
    """
    return bytes([int(mode)])


def encode_clear_error(code: int = 0x00) -> bytes:
    """Build the 1-byte payload for a 0x441 clear-error command.

    Args:
        code: Fault selector. ``0x00`` clears all non-critical faults (including
            releasing the e-stop error); see the ``0x01``–``0x10`` codes in the
            manual for targeting specific faults. Masked to one byte.

    Returns:
        A single byte for CAN id ``CanId.CMD_CLEAR_ERROR``.
    """
    return bytes([code & 0xFF])


def encode_light(enable: bool, on: bool) -> bytes:
    """Build the 8-byte payload for a 0x121 light-control command.

    Args:
        enable: Whether the light-control command is active. If ``False`` the
            chassis ignores the light command and keeps its default behaviour.
        on: When ``enable`` is ``True``, ``True`` = steady on, ``False`` = off.

    Returns:
        Exactly 8 bytes for CAN id ``CanId.CMD_LIGHT`` (bytes 2–7 reserved 0).
    """
    ctrl = 0x01 if enable else 0x00
    mode = 0x01 if on else 0x00
    return bytes([ctrl, mode, 0, 0, 0, 0, 0, 0])


# --------------------------------------------------------------------------- #
# Feedback decoding
# --------------------------------------------------------------------------- #

@dataclass
class SystemFeedback:
    """Decoded 0x211 system-status frame.

    Attributes:
        normal: ``True`` when byte[0] == 0x00 (system normal), ``False`` on
            system error.
        mode: Current control mode reported by the chassis
            (:class:`SystemMode`: standby / CAN command / remote).
        voltage: Battery voltage in volts (byte[2:4] / 10).
        fault_bytes: Raw fault bytes byte[4..7]; decode with
            :func:`describe_faults`.
    """

    normal: bool           # byte[0] == 0x00
    mode: SystemMode       # byte[1]
    voltage: float         # V
    fault_bytes: tuple[int, int, int, int]  # byte[4..7]

    @property
    def estop(self) -> bool:
        """``True`` if the emergency-stop bit (byte[7] bit[7]) is set."""
        return bool(self.fault_bytes[3] & 0x80)  # byte[7] bit[7]

    @property
    def has_fault(self) -> bool:
        """``True`` if the system is not normal or any fault bit is set."""
        return (not self.normal) or any(self.fault_bytes)


@dataclass
class MotionFeedback:
    """Decoded 0x221 body-motion frame.

    Attributes:
        linear_mps: Body linear velocity in m/s (forward positive).
        angular_rps: Body yaw rate in rad/s (CCW positive).
        steer_rad: Reported inner steering angle in rad.
    """

    linear_mps: float   # m/s
    angular_rps: float  # rad/s
    steer_rad: float    # rad


def decode_system(data: bytes) -> SystemFeedback:
    """Decode a 0x211 system-status frame.

    Args:
        data: The 8-byte frame payload.

    Returns:
        A populated :class:`SystemFeedback`. Unknown mode values fall back to
        ``SystemMode.STANDBY``.
    """
    b0, b1 = data[0], data[1]
    voltage = struct.unpack(">H", data[2:4])[0] / 10.0
    try:
        mode = SystemMode(b1)
    except ValueError:
        mode = SystemMode.STANDBY
    return SystemFeedback(
        normal=(b0 == 0x00),
        mode=mode,
        voltage=voltage,
        fault_bytes=(data[4], data[5], data[6], data[7]),
    )


def decode_motion(data: bytes) -> MotionFeedback:
    """Decode a 0x221 body-motion frame.

    Args:
        data: The 8-byte frame payload.

    Returns:
        A :class:`MotionFeedback` with SI units (m/s, rad/s, rad).
    """
    lin = struct.unpack(">h", data[0:2])[0] / 1000.0
    ang = struct.unpack(">h", data[2:4])[0] / 1000.0
    steer = struct.unpack(">h", data[6:8])[0] / 1000.0
    return MotionFeedback(linear_mps=lin, angular_rps=ang, steer_rad=steer)


def decode_motion_mode(data: bytes) -> tuple[MotionMode, bool]:
    """Decode a 0x291 motion-mode frame.

    Args:
        data: The frame payload (at least 2 bytes).

    Returns:
        A ``(mode, switching_in_progress)`` tuple, where ``mode`` is the current
        :class:`MotionMode` (unknown values fall back to ``ACKERMANN``) and
        ``switching_in_progress`` is ``True`` while a mode change is underway.
    """
    try:
        mode = MotionMode(data[0])
    except ValueError:
        mode = MotionMode.ACKERMANN
    switching = bool(data[1])
    return mode, switching


def decode_wheel_speeds(data: bytes) -> tuple[float, float, float, float]:
    """Decode a 0x281 four-wheel speed frame.

    Args:
        data: The 8-byte frame payload.

    Returns:
        Per-wheel speeds for wheels 1..4 in m/s (RF, RR, LR, LF per manual).
    """
    w = struct.unpack(">hhhh", data)
    return tuple(v / 1000.0 for v in w)  # type: ignore[return-value]


def decode_steer_angles(data: bytes) -> tuple[float, float, float, float]:
    """Decode a 0x271 four-wheel steering-angle frame.

    Args:
        data: The 8-byte frame payload.

    Returns:
        Per-wheel steering angles for motors 5..8 in rad.
    """
    a = struct.unpack(">hhhh", data)
    return tuple(v / 1000.0 for v in a)  # type: ignore[return-value]


# Human-readable fault bit descriptions, keyed by (byte_index_4_7, bit).
FAULT_BITS: dict[tuple[int, int], str] = {
    (5, 0): "RF steering zero-calibration fault",
    (5, 1): "RR steering zero-calibration fault",
    (5, 2): "LR steering zero-calibration fault",
    (5, 3): "LF steering zero-calibration fault",
    (5, 4): "steering calibration timeout",
    (6, 0): "driver status error",
    (6, 2): "motor driver 5 comms fault",
    (6, 3): "motor driver 6 comms fault",
    (6, 4): "motor driver 7 comms fault",
    (6, 5): "motor driver 8 comms fault",
    (6, 6): "over-temperature protection",
    (6, 7): "over-current protection",
    (7, 0): "battery under-voltage fault",
    (7, 1): "over-voltage protection",
    (7, 2): "remote control disconnected",
    (7, 3): "motor driver 1 comms fault",
    (7, 4): "motor driver 2 comms fault",
    (7, 5): "motor driver 3 comms fault",
    (7, 6): "motor driver 4 comms fault",
    (7, 7): "EMERGENCY STOP triggered",
}


def describe_faults(fault_bytes: tuple[int, int, int, int]) -> list[str]:
    """Return the list of active fault descriptions for byte[4..7]."""
    out = []
    for byte_index, value in zip((4, 5, 6, 7), fault_bytes):
        for bit in range(8):
            if value & (1 << bit) and (byte_index, bit) in FAULT_BITS:
                out.append(FAULT_BITS[(byte_index, bit)])
    return out
