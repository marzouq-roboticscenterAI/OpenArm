"""roverservo — one library to drive both halves of the demo rig.

Unifies the two cloned libraries:

* ``rangerair`` — the AgileX **Ranger Air** chassis (CAN 2.0B, 500 kbit/s)
* ``ds2c``      — the **DS2-C servo** lift (CANopen/CiA402, 1 Mbit/s)

Two USB-CAN adapters plug into one computer; :mod:`roverservo.interfaces`
figures out which adapter is which (they run at different bitrates), and
:class:`roverservo.Rover` gives you a single handle to command both.

Quick start::

    from roverservo import Rover

    with Rover() as rig:               # auto-detects both adapters
        rig.rover_enable()
        rig.rover_drive(linear=0.15)   # m/s forward (deadman: keep refreshing)
        rig.servo_enable()
        rig.servo_raise(20_000)        # lift up 2 revs
        print(rig.snapshot())          # unified JSON-friendly state

Or run the demo website::

    python -m roverservo            # serves http://localhost:8080
"""

from . import _deps  # noqa: F401  (make ds2c/rangerair importable)
from .robot import Rover, DEADMAN_TIMEOUT
from .interfaces import Assignment, InterfaceError, identify

__all__ = ["Rover", "Assignment", "InterfaceError", "identify", "DEADMAN_TIMEOUT"]
__version__ = "0.1.0"
