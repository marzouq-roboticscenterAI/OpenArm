"""Import shim for the two sibling libraries this package unifies.

The two cloned libraries live next to this package, not on ``sys.path``:

    RoverServoLib/
        ds2c-servo/            -> module ``ds2c``      (DS2-C servo, python-can, 1 Mbit/s)
        ranger_air_control/    -> package ``rangerair`` (Ranger Air chassis, 500 kbit/s)
        roverservo/            -> this package

``ds2c-servo`` has a hyphen in its name, so it cannot be imported as a package.
Instead we add both sibling directories to ``sys.path`` here, once, so the rest
of the package can simply ``import ds2c`` / ``import rangerair``.

Importing this module for its side effect is enough::

    from . import _deps  # noqa: F401
"""

from __future__ import annotations

import os
import sys

# RoverServoLib/  (parent of this package directory)
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Directory name -> the top-level module/package it provides (for error messages).
_SIBLINGS = {
    "ds2c-servo": "ds2c",
    "ranger_air_control": "rangerair",
}

for _sub in _SIBLINGS:
    _path = os.path.join(_ROOT, _sub)
    if os.path.isdir(_path) and _path not in sys.path:
        # insert(0) so these win over any similarly named site package
        sys.path.insert(0, _path)


def missing_siblings() -> list[str]:
    """Return the directory names of any expected sibling libs that are absent."""
    return [sub for sub in _SIBLINGS if not os.path.isdir(os.path.join(_ROOT, sub))]
