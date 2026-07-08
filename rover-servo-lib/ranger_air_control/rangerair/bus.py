"""Minimal SocketCAN raw-socket transport, standard library only.

A classic (2.0B) CAN frame on Linux SocketCAN is a 16-byte struct::

    struct can_frame {
        canid_t can_id;   // u32, LE; bit31=EFF, bit30=RTR, bit29=ERR
        u8      can_dlc;  // 0..8
        u8      __pad, __res0, __res1;
        u8      data[8];
    };

We only use 11-bit standard identifiers, so no flag bits are set.
"""

from __future__ import annotations

import socket
import struct
from dataclasses import dataclass

CAN_FRAME_FMT = "=IB3x8s"          # can_id, dlc, 3 pad, 8 data bytes
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)  # 16

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF          # 11-bit standard id mask


@dataclass(frozen=True)
class CanFrame:
    """A single received CAN frame.

    Attributes:
        can_id: The 11-bit standard identifier, with SocketCAN flag bits
            already stripped.
        data: The payload bytes, trimmed to the frame's actual DLC (0–8 bytes).
    """

    can_id: int
    data: bytes

    @property
    def dlc(self) -> int:
        """Data length code — the number of payload bytes (0–8)."""
        return len(self.data)


class CanBus:
    """A raw SocketCAN socket bound to a single interface (e.g. ``can0``).

    Thin wrapper over ``socket(AF_CAN, SOCK_RAW, CAN_RAW)`` that packs/unpacks
    the 16-byte ``struct can_frame``. Suitable as a context manager.

    Args:
        interface: SocketCAN interface name to bind to (e.g. ``"can0"``).
        recv_timeout: Socket receive timeout in seconds; :meth:`recv` returns
            ``None`` when it elapses. Pass ``None`` for a blocking socket.

    Raises:
        OSError: If the interface cannot be bound (typically it is down or does
            not exist). The message includes the ``setup_can.sh`` hint.
    """

    def __init__(self, interface: str = "can0", recv_timeout: float | None = 1.0):
        self.interface = interface
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self._sock.bind((interface,))
        except OSError as exc:
            self._sock.close()
            raise OSError(
                f"Could not bind to CAN interface '{interface}': {exc}. "
                f"Is it up? Try: ./setup_can.sh {interface}"
            ) from exc
        if recv_timeout is not None:
            self._sock.settimeout(recv_timeout)

    def send(self, can_id: int, data: bytes) -> None:
        """Transmit one standard (11-bit) CAN frame.

        Args:
            can_id: Frame identifier; masked to 11 bits (``CAN_SFF_MASK``).
            data: 0–8 payload bytes. Right-padded with zeros to 8 bytes on the
                wire, but the DLC is set to ``len(data)``.

        Returns:
            None.

        Raises:
            ValueError: If ``data`` is longer than 8 bytes.
            OSError: If the underlying socket send fails (e.g. bus off).
        """
        if len(data) > 8:
            raise ValueError("CAN 2.0 payload cannot exceed 8 bytes")
        frame = struct.pack(
            CAN_FRAME_FMT, can_id & CAN_SFF_MASK, len(data), data.ljust(8, b"\x00")
        )
        self._sock.send(frame)

    def recv(self) -> CanFrame | None:
        """Receive one CAN frame.

        Returns:
            A :class:`CanFrame` with flag bits stripped and payload trimmed to
            the DLC, or ``None`` if the receive timeout elapsed first.

        Raises:
            OSError: On a socket error other than timeout (e.g. the interface
                was removed).
        """
        try:
            raw = self._sock.recv(CAN_FRAME_SIZE)
        except socket.timeout:
            return None
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        # strip flag bits; error frames are ignored by callers via can_id
        can_id &= CAN_SFF_MASK if not (can_id & CAN_EFF_FLAG) else ~CAN_EFF_FLAG
        return CanFrame(can_id=can_id, data=data[:dlc])

    def close(self) -> None:
        """Close the underlying socket. Idempotent and exception-safe."""
        try:
            self._sock.close()
        except OSError:
            pass

    def __enter__(self) -> "CanBus":
        """Enter the context manager; returns ``self``."""
        return self

    def __exit__(self, *exc) -> None:
        """Exit the context manager; calls :meth:`close`."""
        self.close()
