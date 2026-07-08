"""Discover and identify the two CAN adapters plugged into this computer.

Two USB-CAN adapters are attached at once — one goes to the **Ranger Air
chassis** and one to the **DS2-C servo** — and Linux does *not* guarantee which
becomes ``can0`` and which becomes ``can1``. We therefore cannot hard-code the
mapping; we have to work it out at runtime.

The reliable discriminator is the **bus bitrate plus the frame signature**:

===============  ===============  ==========================================
Device           Bitrate          Signature frames (CAN IDs)
===============  ===============  ==========================================
Ranger Air       500 000 bit/s    feedback 0x211 / 0x221 / 0x291 / 0x271 /
                                  0x281  (the 0x2xx block), broadcast freely
DS2-C servo      1 000 000 bit/s  CANopen heartbeat/bootup 0x700-0x77F, and
                                  SDO server responses 0x580-0x5FF
===============  ===============  ==========================================

The two devices run at *different* bitrates, so an interface configured for one
will hear nothing from the other — which is exactly what makes identification
robust. :func:`identify` brings each interface up at a candidate bitrate, listens
for the matching signature, and (for the servo, which may be quiet) actively
pokes it with an SDO read.

Manual override always wins: set ``ROVER_CAN`` / ``SERVO_CAN`` (and optionally
``ROVER_BITRATE`` / ``SERVO_BITRATE``) in the environment to skip auto-detection.

Everything here uses :mod:`python-can` for listening (already a dependency of the
servo library) and the ``ip`` command for configuration (needs ``sudo``).
"""

from __future__ import annotations

import os
import subprocess
import time
from dataclasses import dataclass
from typing import Optional

import can

# Canonical bitrates for the two devices (overridable via env).
ROVER_BITRATE = int(os.environ.get("ROVER_BITRATE", 500_000))
SERVO_BITRATE = int(os.environ.get("SERVO_BITRATE", 1_000_000))

# Frame-ID signatures used to classify a live bus.
_ROVER_FB_IDS = {0x211, 0x221, 0x231, 0x291, 0x271, 0x281}
_ROVER_FB_LO, _ROVER_FB_HI = 0x200, 0x2FF        # whole Ranger feedback block
_SERVO_HEARTBEAT_LO, _SERVO_HEARTBEAT_HI = 0x700, 0x77F
_SERVO_SDO_LO, _SERVO_SDO_HI = 0x580, 0x5FF


class InterfaceError(RuntimeError):
    """Raised when CAN interfaces cannot be discovered, configured, or identified."""


@dataclass
class Assignment:
    """The result of identification: which interface is which device.

    Either field may be ``None`` if that device was not found on any interface.
    """

    rover: Optional[str] = None
    servo: Optional[str] = None

    @property
    def both_found(self) -> bool:
        return self.rover is not None and self.servo is not None

    def __str__(self) -> str:
        return f"rover={self.rover or '—'}  servo={self.servo or '—'}"


# --------------------------------------------------------------------------- #
# `ip` helpers (interface listing / configuration)
# --------------------------------------------------------------------------- #
def list_can_interfaces() -> list[str]:
    """Return the names of all SocketCAN interfaces (up or down), e.g. ``["can0"]``."""
    try:
        out = subprocess.run(
            ["ip", "-br", "link", "show", "type", "can"],
            capture_output=True, text=True, check=True,
        ).stdout
    except FileNotFoundError as exc:
        raise InterfaceError("'ip' command not found — is iproute2 installed?") from exc
    except subprocess.CalledProcessError as exc:
        raise InterfaceError(f"'ip link show type can' failed: {exc.stderr}") from exc
    return [line.split()[0] for line in out.splitlines() if line.strip()]


def interface_status(iface: str) -> tuple[bool, Optional[int]]:
    """Return ``(is_up, bitrate)`` for ``iface`` (bitrate ``None`` if unknown)."""
    details = subprocess.run(
        ["ip", "-details", "link", "show", iface],
        capture_output=True, text=True,
    ).stdout
    is_up = "state UP" in details or "<UP" in details or ",UP" in details
    bitrate: Optional[int] = None
    # 'bitrate <N>' appears in the -details output for a configured CAN link.
    parts = details.split()
    if "bitrate" in parts:
        try:
            bitrate = int(parts[parts.index("bitrate") + 1])
        except (ValueError, IndexError):
            bitrate = None
    return is_up, bitrate


def configure_interface(iface: str, bitrate: int, txqueuelen: int = 1000) -> None:
    """Bring ``iface`` down, set ``bitrate``, and bring it back up (needs sudo).

    Idempotent-ish: safe to call repeatedly. Raises :class:`InterfaceError` on
    failure (most commonly: missing sudo rights, or the adapter unplugged).
    """
    is_up, rate = interface_status(iface)
    if is_up and rate == bitrate:
        return  # already exactly how we want it
    # (label, argv, must_succeed): 'down' and 'txqueuelen' are best-effort
    # (a fresh interface may not be up yet, or may reject txqueuelen); the
    # bitrate set and the final 'up' must succeed or configuration has failed.
    steps = [
        ("down", ["sudo", "ip", "link", "set", iface, "down"], False),
        ("bitrate", ["sudo", "ip", "link", "set", iface, "type", "can",
                     "bitrate", str(bitrate)], True),
        ("txqueuelen", ["sudo", "ip", "link", "set", iface, "txqueuelen",
                        str(txqueuelen)], False),
        ("up", ["sudo", "ip", "link", "set", iface, "up"], True),
    ]
    for _label, cmd, must_succeed in steps:
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0 and must_succeed:
            raise InterfaceError(
                f"failed to configure {iface} @ {bitrate} bit/s "
                f"(`{' '.join(cmd)}`): {res.stderr.strip()}"
            )


# --------------------------------------------------------------------------- #
# Listening / probing
# --------------------------------------------------------------------------- #
def _collect_ids(iface: str, duration: float) -> set[int]:
    """Passively collect every arbitration ID seen on ``iface`` for ``duration`` s."""
    seen: set[int] = set()
    try:
        bus = can.Bus(interface="socketcan", channel=iface)
    except OSError:
        return seen
    try:
        deadline = time.time() + duration
        while time.time() < deadline:
            msg = bus.recv(timeout=max(0.0, deadline - time.time()))
            if msg is not None and not msg.is_error_frame:
                seen.add(msg.arbitration_id)
    finally:
        bus.shutdown()
    return seen


def _classify_ids(ids: set[int]) -> Optional[str]:
    """Classify a set of observed CAN IDs as ``'rover'``, ``'servo'``, or ``None``."""
    for i in ids:
        if _ROVER_FB_LO <= i <= _ROVER_FB_HI:
            return "rover"
    for i in ids:
        if (_SERVO_HEARTBEAT_LO <= i <= _SERVO_HEARTBEAT_HI
                or _SERVO_SDO_LO <= i <= _SERVO_SDO_HI):
            return "servo"
    return None


def _servo_sdo_probe(iface: str, node: int = 1, timeout: float = 0.6) -> bool:
    """Actively confirm a DS2-C servo on ``iface`` by reading its statusword.

    Sends an expedited SDO upload request for 0x6041:00 to node ``node`` and
    waits for the server response on 0x580+node. A quiet drive (no heartbeat)
    still answers this, so it complements the passive listen. Assumes ``iface``
    is already up at the servo bitrate.
    """
    try:
        bus = can.Bus(interface="socketcan", channel=iface)
    except OSError:
        return False
    try:
        req = bytes([0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0])  # upload 0x6041:00
        bus.send(can.Message(arbitration_id=0x600 + node, data=req,
                             is_extended_id=False))
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = bus.recv(timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if msg.arbitration_id == 0x580 + node and len(msg.data) >= 4:
                return True
    except can.CanError:
        return False
    finally:
        bus.shutdown()
    return False


# --------------------------------------------------------------------------- #
# Top-level identification
# --------------------------------------------------------------------------- #
def _env_override() -> Optional[Assignment]:
    rover = os.environ.get("ROVER_CAN")
    servo = os.environ.get("SERVO_CAN")
    if rover or servo:
        return Assignment(rover=rover or None, servo=servo or None)
    return None


def identify(
    interfaces: Optional[list[str]] = None,
    configure: bool = True,
    listen_time: float = 1.5,
    verbose: bool = True,
) -> Assignment:
    """Work out which CAN interface is the rover and which is the servo.

    Args:
        interfaces: Interface names to consider. Defaults to every SocketCAN
            interface found by :func:`list_can_interfaces`.
        configure: If ``True`` (default), (re)configure each interface's bitrate
            while probing — this needs ``sudo`` but is the only way to hear a
            device whose bitrate differs from the interface's current setting.
            If ``False``, interfaces are probed at whatever bitrate they are
            already running (use when they are pre-configured and you lack sudo).
        listen_time: Seconds to listen passively at each candidate bitrate.
        verbose: Print a line per step.

    Returns:
        An :class:`Assignment`. Check ``.both_found`` before driving.

    Raises:
        InterfaceError: If no CAN interfaces exist at all.

    Environment override:
        If ``ROVER_CAN`` and/or ``SERVO_CAN`` are set, they are honoured verbatim
        and no probing happens.
    """
    override = _env_override()
    if override is not None:
        if verbose:
            print(f"[identify] using environment override: {override}")
        if configure:
            if override.rover:
                configure_interface(override.rover, ROVER_BITRATE)
            if override.servo:
                configure_interface(override.servo, SERVO_BITRATE)
        return override

    ifaces = interfaces if interfaces is not None else list_can_interfaces()
    if not ifaces:
        raise InterfaceError(
            "no SocketCAN interfaces found. Plug in the USB-CAN adapters and, if "
            "needed, load the driver (`sudo modprobe gs_usb`). Check `lsusb | grep -i can`."
        )

    def log(msg: str) -> None:
        if verbose:
            print(f"[identify] {msg}")

    log(f"candidate interfaces: {', '.join(ifaces)}")
    result = Assignment()

    for iface in ifaces:
        if result.both_found:
            break

        # --- try the rover first: 500 kbit/s, passive listen -------------- #
        if result.rover is None:
            if configure:
                log(f"{iface}: probing as ROVER @ {ROVER_BITRATE} bit/s")
                try:
                    configure_interface(iface, ROVER_BITRATE)
                except InterfaceError as exc:
                    log(f"  could not configure: {exc}")
            ids = _collect_ids(iface, listen_time)
            if _classify_ids(ids) == "rover":
                log(f"  -> {iface} is the ROVER (saw {sorted(hex(i) for i in ids)[:6]})")
                result.rover = iface
                continue

        # --- else try the servo: 1 Mbit/s, passive then active probe ------ #
        if result.servo is None:
            if configure:
                log(f"{iface}: probing as SERVO @ {SERVO_BITRATE} bit/s")
                try:
                    configure_interface(iface, SERVO_BITRATE)
                except InterfaceError as exc:
                    log(f"  could not configure: {exc}")
            ids = _collect_ids(iface, listen_time)
            if _classify_ids(ids) == "servo":
                log(f"  -> {iface} is the SERVO (heartbeat/SDO seen)")
                result.servo = iface
                continue
            if _servo_sdo_probe(iface):
                log(f"  -> {iface} is the SERVO (answered SDO probe)")
                result.servo = iface
                continue

        log(f"{iface}: no recognised traffic — unassigned for now")

    if verbose:
        log(f"result: {result}")
        if not result.both_found:
            missing = []
            if result.rover is None:
                missing.append("rover (Ranger Air)")
            if result.servo is None:
                missing.append("servo (DS2-C)")
            log(f"WARNING: did not find {', '.join(missing)}. "
                f"Is it powered on? Remote SWB in TOP for the rover? "
                f"You can force a mapping with ROVER_CAN=/SERVO_CAN= env vars.")
    return result
