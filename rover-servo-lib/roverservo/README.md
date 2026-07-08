# roverservo — unified control + web dashboard for the rover & servo demo

One library and one website to drive **both** halves of the rig from a single
computer with **two USB-CAN adapters plugged in at once**:

| Device | Library it wraps | Bus | Bitrate |
|--------|------------------|-----|---------|
| AgileX **Ranger Air** chassis | [`rangerair`](../ranger_air_control) | CAN 2.0B | **500 kbit/s** |
| **DS2-C** servo lift | [`ds2c`](../ds2c-servo) | CANopen / CiA402 | **1 Mbit/s** |

`roverservo` sits next to those two cloned libraries and imports them directly:

```
RoverServoLib/
├── ds2c-servo/          # cloned lib  -> module `ds2c`
├── ranger_air_control/  # cloned lib  -> package `rangerair`
├── roverservo/          # ← this package
└── setup_can.sh         # bring up BOTH adapters
```

## The two-adapter problem (and how it's solved)

Linux does **not** guarantee which USB-CAN adapter becomes `can0` and which
becomes `can1` — it depends on plug order and enumeration. So we can't hard-code
"the rover is `can0`".

The reliable discriminator is that **the two devices run at different bitrates**,
each with a recognisable frame signature:

* **Rover** — 500 kbit/s, constantly broadcasts feedback in the `0x2xx` block
  (`0x211` system, `0x221` motion, `0x291` mode, …).
* **Servo** — 1 Mbit/s, emits CANopen heartbeat/bootup (`0x700`–`0x77F`) and
  answers SDO reads on `0x580+node`.

[`roverservo/interfaces.py`](interfaces.py) brings each interface up at a
candidate bitrate, listens for the matching signature, and — because a quiet
drive may not broadcast — actively pokes the servo with an SDO read of its
statusword. Whichever interface answers which protocol *is* that device. An
interface configured for one device is deaf to the other, which is exactly what
makes the match unambiguous.

You can always override auto-detection:

```bash
export ROVER_CAN=can0 SERVO_CAN=can1     # or --rover-can / --servo-can flags
```

## Setup

```bash
pip install python-can            # only hard dependency (for the DS2-C library)

cd RoverServoLib
./setup_can.sh                    # sudo: brings up every CAN interface
python3 -m roverservo             # detects both adapters, serves the dashboard
```

Open **http://localhost:8080**. The server prints the detected mapping on start.

Useful flags:

```bash
python3 -m roverservo --identify-only          # just print which iface is which
python3 -m roverservo --port 9000
python3 -m roverservo --rover-can can0 --servo-can can1   # force mapping
python3 -m roverservo --require-both           # fail unless BOTH devices are found
                                               # (default: start with whatever is connected)
python3 -m roverservo --no-configure           # don't sudo-set bitrates
```

> **Rover remote:** the FS remote's **SWB must be in the TOP position** or the
> chassis ignores all CAN commands (`enable()` won't take).

## The website

A single-page dashboard (served from [`static/`](static/)) with live telemetry
polled ~7×/s and a JSON control API:

* **Big red E-STOP ALL** (also the **spacebar**) stops both devices at once.
* **Rover panel** — battery, control/motion mode, faults; enable/disable, clear
  errors, lights, mode selector (Ackermann/Tilt/Spin/Park), speed + turn
  sliders, and a **hold-to-drive D-pad** (or **W A S D** / arrow keys).
* **Servo panel** — live position (rev), velocity (rpm), decoded CiA402 state;
  enable/disable, fault-reset, servo E-stop, discrete **Raise/Lower step**
  moves, and **hold-to-jog** up/down buttons.

### Deadman safety

Driving is **hold-to-move**. The browser refreshes a held setpoint ~10×/s; the
server runs a **deadman** ([`robot.py`](robot.py)) that zeroes rover motion or
servo jog if refreshes stop for >0.5 s — so if the tab closes, the network drops,
or you release the key, the rig halts on its own. (This is still a *software*
stop; keep a hand on real power / a physical E-stop, per both device libraries.)

## Using the library directly

```python
from roverservo import Rover

with Rover() as rig:                 # auto-detects both adapters
    rig.rover_enable()
    rig.rover_set_mode("ackermann")
    rig.rover_drive(linear=0.15, steer=0.2)   # deadman: refresh to keep moving

    rig.servo_enable()
    rig.servo_raise(20_000)          # lift up 2 rev (10000 pulses/rev)

    print(rig.snapshot())            # unified, JSON-friendly state of both
    rig.estop_all()
```

### How it stays safe under a multi-threaded web server

* The **rover** library already owns its bus with internal RX/TX threads and a
  thread-safe state snapshot — `Rover` just forwards to it.
* The **servo** library is single-threaded and not safe for concurrent SDO
  traffic, so `Rover` funnels *all* servo bus I/O through **one worker thread**
  with a command queue. Control calls enqueue commands; the same worker also
  refreshes the cached telemetry the dashboard reads. HTTP handler threads never
  touch the servo bus directly, so SDO requests/responses can't interleave.

## HTTP API

| Method & path | Body | Effect |
|---|---|---|
| `GET /api/state` | — | full rig snapshot (JSON) |
| `POST /api/estop` | — | stop **both** devices |
| `POST /api/rover/enable` \| `disable` \| `clear_errors` \| `stop` | — | rover lifecycle |
| `POST /api/rover/mode` | `{"mode":"ackermann\|tilt\|spin\|park"}` | kinematic mode |
| `POST /api/rover/light` | `{"on":true}` | light bars |
| `POST /api/rover/drive` | `{"linear":..,"angular":..,"steer":..}` | motion setpoint (deadman) |
| `POST /api/servo/enable` \| `disable` \| `check` \| `fault_reset` \| `stop` \| `estop` | — | servo lifecycle |
| `POST /api/servo/move` \| `raise` \| `lower` | `{"pulses":..,"velocity":..}` | discrete move |
| `POST /api/servo/jog` | `{"velocity":..}` | continuous jog (deadman) |

## Files

| File | Purpose |
|------|---------|
| [`interfaces.py`](interfaces.py) | discover & identify the two CAN adapters by bitrate + signature |
| [`robot.py`](robot.py) | `Rover` facade: unified control, deadman, servo worker, snapshot |
| [`server.py`](server.py) | stdlib HTTP server: JSON API + static dashboard |
| [`__main__.py`](__main__.py) | `python -m roverservo` entry point |
| [`static/`](static/) | dashboard: `index.html`, `style.css`, `app.js` |
| [`_deps.py`](_deps.py) | puts the sibling `ds2c` / `rangerair` libs on the import path |
```
