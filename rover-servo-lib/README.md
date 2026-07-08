# RoverServoLib

Unified control library and web dashboard for a demo rig that combines an
**AgileX Ranger Air** rover chassis and a **DS2-C servo** lift, both driven over
CAN from a single computer with **two USB-CAN adapters plugged in at once**.

The headline problem this solves: Linux doesn't guarantee which adapter becomes
`can0` vs `can1`, so the software has to work out — or be told — which interface
goes to which device. See [`roverservo/README.md`](roverservo/README.md) for the
full design and API.

## Quick start

```bash
pip install python-can          # only external dependency (for the servo lib)

./setup_can.sh                  # sudo: bring up both CAN adapters
python3 -m roverservo --identify-only   # confirm: rover=canX  servo=canY
python3 -m roverservo           # serve the dashboard at http://localhost:8080
```

For stable, plug-order-independent interface names (recommended, and required if
you add more CAN devices at the same bitrate), use the udev tooling:

```bash
./list_can_adapters.sh          # print each adapter's serial / USB port
# fill those into 99-roverservo-can.rules, then install it (see the file header)
```

## Layout

| Path | What it is |
|------|-----------|
| [`roverservo/`](roverservo/) | **This project** — the unified library + web dashboard |
| [`ds2c-servo/`](ds2c-servo/) | Vendored: DS2-C servo control library (`ds2c`), CANopen @ 1 Mbit/s |
| [`ranger_air_control/`](ranger_air_control/) | Vendored: Ranger Air chassis library (`rangerair`), CAN 2.0B @ 500 kbit/s |
| [`setup_can.sh`](setup_can.sh) | Bring up both CAN adapters |
| [`list_can_adapters.sh`](list_can_adapters.sh) | Print adapter identities for stable udev naming |
| [`99-roverservo-can.rules`](99-roverservo-can.rules) | udev template for stable interface names |

## Provenance

The two device libraries are vendored in from their standalone repositories and
include local edits (per-device interface resolution via `DS2C_CAN` / `RANGER_CAN`
environment variables so the two devices no longer both default to `can0`):

- `ds2c-servo` — https://github.com/jackbiggins-dev/ds2c-servo
- `ranger_air_control` — https://github.com/jackbiggins-dev/ranger_air_control

## Safety

This is **software control only**. If a CAN adapter is unplugged or a device
hangs, the software cannot stop the hardware. Keep a hand on real power / a
physical E-stop. The rover's FS remote **SWB must be in the TOP position** or the
chassis ignores CAN commands. The servo's direction is inverted (positive =
physically down); the Raise/Jog-up controls account for it.
