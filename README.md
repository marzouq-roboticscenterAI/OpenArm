# OpenArm-C

Control DaMiao DM‑series arm motors over **classic CAN 2.0**, in **C**, with a
built‑in web dashboard and automatic gamepad control.

The application lives in **[`openarm-c/`](openarm-c/)** — see its
[README](openarm-c/README.md) for full details.

## Quick start
```bash
./run.sh                 # build, bring up the CAN bus(es), launch the dashboard
```
Then open **http://localhost:8080**, press **Connect motors**, and:
- plug in a controller → **Control Scheme 1** (per‑joint jog) engages automatically, or
- use the per‑joint **sliders** (degrees) on the dashboard.

A controller also drives the **rover** and **lift servo** when present:
- **R2 / L2** — rover forward / back (AgileX Ranger Air, Ackermann)
- **L1 / R1** — rover spin left / right (SPIN mode)
- **Right thumbstick ↕** — DS2‑C lift servo up / down

Calibrate from the dashboard: **Auto‑Calibrate** (sweeps to hardstops) or
**Manual Calibrate** (jog each joint to its limits and press A / Mark). One
**E‑STOP** cuts everything. Actions and motor angles are logged to
`openarm-c/openarm.log` (viewable via the **Log** button).

Separate calibration script: `./calibrate.sh`.

## Notes
- Left arm = `can0`, right arm = `can1` (both 1 Mbit/s).
- Rover (Ranger Air) = `can2` (500 kbit/s); lift servo (DS2‑C, CANopen) = `can3`
  (1 Mbit/s). Override interface names with `RANGER_CAN` / `DS2C_CAN`. Both are
  ported from [RoverServoLib](https://github.com/jackbiggins-dev/RoverServoLib)
  and obey the same **E‑STOP** (rover → standby, lift → quick‑stop).
- Motors are forced into MIT mode on connect; per‑joint gains and an anti‑vibration
  Kd cap are applied; the sliders track the **actual** encoder angle.
- The previous Python WebUI (upstream OpenArm) has been superseded by this C app;
  it remains recoverable from the `upstream/main` remote if ever needed.

## Docs
Hardware reference images are in [`docs/`](docs/).
