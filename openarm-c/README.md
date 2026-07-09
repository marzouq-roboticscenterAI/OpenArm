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

## Controlling without a controller (input architecture)
**A gamepad is optional.** The engine ([`control.h`](control.h)) exposes a
thread-safe *command API* that has no knowledge of any input device:
`control_connect` · `control_disconnect` · `control_jog` · `control_set_target`
(absolute) · `control_select` · `control_estop` · `control_clear_estop` ·
`control_request_calibration` · `control_manual_*` · `control_set_gain_scale`.

Two **independent, peer** inputs feed that API — neither depends on the other:
1. **Gamepad** — read inside the control loop, but every gamepad action is gated
   behind `pad_on` (`if (pad_on && S.connected …)` in [`control.c`](control.c)).
   With no pad plugged in, `pad_on == 0` and the block is skipped; the arm runs
   exactly the same.
2. **Web / HTTP** — [`httpd.c`](httpd.c) maps routes straight onto the same
   command API with zero gamepad involvement. This is the path a website, a
   script, or `curl` uses.

So the **arm is fully drivable with no controller**: use the dashboard sliders,
hit the HTTP API directly, or run the headless one-shot `--move` (below). The
gamepad is a convenience input, not a dependency, and [`gamepad.c`](gamepad.c) is
a standalone module you can ignore or remove without touching arm control.

Headless / programmatic options:
- `./openarm --move IFACE ID TARGET [KP KD]` — move one joint and exit (no UI).
- `./openarm --scan` — list present motors; `--calibrate` — calibrate and exit.
- Any HTTP client against the API above (e.g. `curl -X POST
  'localhost:8080/api/target?bus=0&motor=3&pos=0.5'`).

### ⚠️ Rover and lift are currently gamepad-only
Unlike the arm, the **Ranger Air rover (can2)** and **DS2-C lift servo (can3)**
are wired *only* to the gamepad in the control loop (`rover_go = pad_on && …` in
[`control.c`](control.c)). There are **no HTTP command endpoints** to drive them —
[`httpd.c`](httpd.c) only *reports* their status in `/api/status`. Without a
physical controller they are commanded to stop and cannot be moved from a website.

This is a wiring gap, not a design limit: the underlying drivers are already
decoupled the same way the arm is — [`ranger.h`](ranger.h) exposes
`ranger_drive(r, mode, lin, ang)` and [`ds2c.h`](ds2c.h) exposes
`ds2c_set_velocity(d, pps)`. To make them web-drivable, add thread-safe command
setters in `control.c` (mirroring `control_jog` / `control_set_target`, applied
by the loop when `!pad_on`) and matching endpoints in `httpd.c` (e.g.
`/api/rover?lin=&ang=`, `/api/lift?vel=`).

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
