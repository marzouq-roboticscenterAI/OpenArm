# OpenArm-C

A self-contained **C** re-implementation of the OpenArm control stack: DaMiao
motor control over **classic CAN 2.0**, gamepad auto-detection, calibration, and
a built-in **HTTP dashboard** — no Python, no external libraries (just libc +
pthreads).

## What it does
When you run it, it opens the CAN bus(es) and starts a web dashboard, but leaves
the arm **limp** — nothing is energized until you press **Connect motors** in the
dashboard. After connecting, every present joint holds in place; **plug in a game
controller and it takes over with Control Scheme 1** (per-joint jog), or use the
per-joint **sliders** in the dashboard when there's no controller. Calibration,
sliders, and gamepad driving are all gated behind Connect.

## Build
```
make
```
Produces `./openarm`. Needs only a C compiler and pthreads.

## Run — one command
```
./run.sh                 # builds, brings up every can* (sudo), launches on :8080
./run.sh --port 9000 can0 can1
```
`run.sh` `exec`s the app, so **Ctrl-C stops everything cleanly** — the SIGINT
handler disables every motor and joins the control/HTTP threads before exit.

Or do the steps by hand:
```
sudo ./canup.sh can0 1000000        # bring the bus up (repeat for can1)
./openarm                            # auto-detects can*, serves on :8080
# or: ./openarm --port 8080 --web web can0 can1
```
Open **http://localhost:8080**. Plug in an Xbox/8BitDo/PS pad — it engages
Scheme 1 automatically (server-side via evdev; no browser gamepad API needed).

Reading the controller needs access to `/dev/input/event*` — run from a native
terminal and be in the `input` group (`sudo usermod -aG input $USER`, re-login),
or run the binary with sudo.

## Control Scheme 1 (gamepad)
- **D-pad up/down** — select joint (J1…J8)
- **D-pad left/right** — switch CAN bus / arm
- **Left stick X** — jog the selected joint within its calibrated range
- **A** — (reserved) select
- **START** — E-STOP (disable all)

## Calibration
Click **Calibrate** on the dashboard (or it can be triggered via `POST
/api/calibrate`). Each present joint is swept to its hardstops with a bounded,
no-hang routine (absent/stuck joints fail fast), zeroed, and the calibrated
range is saved to `arm_calib.txt` and used to bound jogging.

## HTTP API
- `GET  /api/status` — full JSON state (buses, motors, gamepad, selection)
- `POST /api/estop` · `/api/clear` · `/api/calibrate`
- `POST /api/jog?bus=&motor=&delta=`
- `POST /api/select?bus=&motor=`
- `POST /api/gain?scale=` — scale all kp (Kd stays capped at 2.5, anti-vibration)

## Design notes
- Motors are forced into **MIT mode** (`CTRL_MODE=1`) on enable — they can power
  up in PosVel mode and silently ignore MIT frames otherwise.
- **Per-joint gains** by model tier (8009 base/shoulder kp=40, 4340 kp=22, 4310
  wrist kp=12); **Kd is hard-capped at 2.5** because ≥~3.5 limit-cycles at 50 Hz.
- One control thread owns the CAN bus; the HTTP thread only reads a mutex-guarded
  snapshot and posts commands. Ctrl-C disables every motor on the way out.

## Modules
`damiao` (protocol) · `socketcan` (transport) · `gamepad` (evdev) ·
`calib` (calibration) · `control` (engine + Scheme 1) · `httpd` (dashboard) · `main`.
