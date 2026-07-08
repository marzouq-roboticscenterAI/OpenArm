"""RangerAir — a zero-dependency Python interface to the AgileX Ranger Air UGV.

Talks to the chassis over the CAN 2.0B bus (500 kbit/s) via Linux SocketCAN,
using only the Python standard library.  No ROS, python-can or can-utils
required — just a SocketCAN interface that is already `up` (see setup_can.sh).

Typical use::

    from rangerair import RangerAir, MotionMode

    with RangerAir("can0") as bot:
        bot.enable()               # switch chassis into CAN command mode
        bot.clear_errors()         # release e-stop / clear non-critical faults
        bot.set_mode(MotionMode.ACKERMANN)
        bot.drive(linear=0.1)      # 0.1 m/s forward, held automatically at 50 Hz
        time.sleep(1.0)
        bot.stop()

The library keeps the required 50 Hz command heartbeat alive on a background
thread, and decodes chassis feedback frames into a snapshot you can read at any
time via ``bot.state``.
"""

from .protocol import (
    MotionMode,
    ControlMode,
    LIMITS,
    CanId,
)
from .driver import RangerAir, RangerState

__all__ = [
    "RangerAir",
    "RangerState",
    "MotionMode",
    "ControlMode",
    "LIMITS",
    "CanId",
]

__version__ = "0.1.0"
