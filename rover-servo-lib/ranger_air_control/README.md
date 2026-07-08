# Ranger Air — CAN control library

A zero-dependency Python interface to the AgileX **Ranger Air** UGV, talking
directly to the chassis over CAN 2.0B (500 kbit/s) via Linux SocketCAN.

No ROS, no `python-can`, no `can-utils` — just the Python standard library and
a USB-CAN adapter (the gs_usb one that ships with the robot).

## Contents

- [Connecting to the robot](#connecting-to-the-robot)
- [Layout](#layout)
- [Using the library](#using-the-library)
- [API reference](#api-reference)
  - [`RangerAir`](#class-rangerairinterface--can0-auto_start--true)
  - [`RangerState`](#class-rangerstate)
  - [Enums](#enums)
  - [`rangerair.protocol` — codec functions](#rangerairprotocol--codec-functions)
  - [`rangerair.bus` — transport](#rangerairbus--transport)
- [Operating notes from the manual](#operating-notes-from-the-manual)
- [CAN protocol reference](#can-protocol-reference)

## Connecting to the robot

Follow these steps **in order**. Steps 1–2 are one-time per power-up or adapter
re-plug; after that you're connected.

### 1. Physical setup (robot + remote)

1. **Power on** the Ranger Air (rear power button) and wait for it to boot.
2. **Release the e-stop** — twist the emergency-stop button clockwise if it's
   pressed in.
3. On the FS remote, set **SWB to the TOP position** (command-control mode).
   The remote always overrides CAN, so if SWB is in remote mode the robot will
   **ignore every command you send**.
4. **Plug the bundled USB-CAN adapter** into your computer.

### 2. Bring up the CAN interface  *(needs `sudo`; once per boot / re-plug)*

```bash
cd ranger_air_control
./setup_can.sh                 # configures & brings up can0 @ 500 kbit/s
```

You should see the interface reported as `UP`. If it says the interface wasn't
found, the adapter isn't plugged in — check `lsusb | grep -i can`. If it comes
up as something other than `can0`, pass the name: `./setup_can.sh can1`
(list them with `ip -br link show type can`).

### 3. Verify the link  *(read-only — sends nothing to the robot)*

```bash
python3 monitor.py             # Ctrl-C to stop
```

Expect live lines showing `sys=OK`, a battery voltage (~24–29 V), and
`faults=[none]`. That confirms the wiring, the 500 kbit/s baud rate, and that
the robot is powered and healthy. If you instead see *“No feedback received”*,
the robot isn't powered on, or the CAN wiring / bring-up isn't right.

### 4. Drive it

```bash
python3 demo_move.py           # small, safe forward nudge (asks to confirm)
```

…or drive from your own code — see [Using the library](#using-the-library).

### Reconnecting later

After a reboot or unplugging the adapter, the interface configuration is lost.
Just re-run **step 2** (`./setup_can.sh`) — nothing else needs redoing.

### The whole happy path, condensed

```bash
cd ranger_air_control
./setup_can.sh        # 1) bring up CAN (sudo, once per boot/replug)
python3 monitor.py    # 2) confirm the robot is alive (Ctrl-C to exit)
python3 demo_move.py  # 3) move
```

## Layout

| File | Purpose |
|------|---------|
| `rangerair/` | The library package |
| `rangerair/driver.py` | `RangerAir` high-level control + `RangerState` |
| `rangerair/protocol.py` | Frame encode/decode, enums, limits (source of truth for units) |
| `rangerair/bus.py` | `CanBus` raw SocketCAN transport + `CanFrame` |
| `setup_can.sh` | Bring up the CAN interface at 500 kbit/s (needs `sudo`) |
| `monitor.py` | Read-only listener — prints chassis feedback, sends nothing |
| `demo_move.py` | Short, safe proof-of-movement nudge |

> Every public class, method, and function has a full docstring with argument
> and return details. This README summarises them; `help(rangerair.RangerAir)`
> (or IDE hover) gives the same information inline while you code.

## Using the library

```python
import time
from rangerair import RangerAir, MotionMode

with RangerAir("can0") as bot:
    bot.wait_for_feedback()          # confirm the link is live
    bot.enable()                     # enter CAN command mode
    bot.clear_errors()               # release e-stop error / clear faults
    bot.set_mode(MotionMode.ACKERMANN)

    bot.drive(linear=0.15)           # 0.15 m/s forward, held at 50 Hz
    time.sleep(1.0)
    bot.drive(linear=0.15, steer=0.3)  # curve left
    time.sleep(1.0)
    bot.stop()
# leaving the `with` block stops the robot and drops it to standby
```

The driver runs two background threads: a **50 Hz transmit heartbeat** (the
chassis stops if it sees no command for 500 ms) and a **receive thread** that
decodes feedback into `bot.state` (battery voltage, faults, per-wheel speeds…).

---

## API reference

Import surface:

```python
from rangerair import RangerAir, RangerState, MotionMode, ControlMode, LIMITS, CanId
```

### `class RangerAir(interface="can0", auto_start=True)`

The main control interface for one chassis. Opens a raw CAN socket and, by
default, starts the RX/TX threads immediately. **Construction transmits
nothing** — no motion is possible until you call `enable()`.

**Constructor**

| Arg | Type | Default | Meaning |
|-----|------|---------|---------|
| `interface` | `str` | `"can0"` | SocketCAN interface (must be up at 500 kbit/s) |
| `auto_start` | `bool` | `True` | Start RX/TX threads in `__init__`; else call `start()` |

Raises `OSError` if the interface can't be bound (usually it's down — run
`setup_can.sh`).

Use it as a context manager (`with RangerAir() as bot:`) so the robot is always
stopped and returned to standby on exit.

**Lifecycle methods**

| Method | Returns | Description |
|--------|---------|-------------|
| `start()` | `None` | Start RX/TX threads (only needed if `auto_start=False`). |
| `close()` | `None` | Stop the robot, drop to standby, join threads, close socket. Idempotent; called automatically on context-manager exit. |

**Mode / error commands**

| Method | Returns | Description |
|--------|---------|-------------|
| `enable()` | `None` | Send 0x421 → CAN command mode and arm the 50 Hz heartbeat. Required before any motion. Raises `OSError` on send failure. |
| `disable()` | `None` | Zero the command, stop the heartbeat, send 0x421 → standby. |
| `clear_errors(code=0x00)` | `None` | Send 0x441 to clear faults. `code=0x00` clears all non-critical faults incl. releasing the e-stop *error*. Other codes target specific faults (see docstring / manual). |
| `set_mode(mode, settle=0.5)` | `None` | Send 0x141 to switch kinematic mode; zeroes motion during the switch and sleeps `settle` seconds for the wheels to re-orient. Pass `settle=0` to poll `state.switching_mode` yourself. |
| `set_light(on)` | `None` | Send 0x121 to turn the light bars steady-on (`True`) or off (`False`). |
| `park()` | `None` | Convenience for `set_mode(MotionMode.PARK)` — wheels lock in an X. |

**Motion commands** (non-blocking; they set a setpoint the heartbeat streams)

| Method | Returns | Description |
|--------|---------|-------------|
| `drive(linear=0.0, angular=0.0, steer=0.0)` | `None` | Set the target motion. Held until the next `drive`/`stop`/`disable`/`close`. Only effective after `enable()`. |
| `stop()` | `None` | Set velocity to zero but keep the heartbeat alive (stays in command mode). Preferred in-motion stop. |

`drive()` arguments (clamped automatically to the limits below):

| Arg | Unit | Sign / range | Active in |
|-----|------|--------------|-----------|
| `linear` | m/s | forward +, ±1.5 | Ackermann, tilt |
| `angular` | rad/s | CCW +, ±3.259 | spin |
| `steer` | rad | left +, ±0.698 (Ackermann) / ±1.571 (tilt) | Ackermann, tilt |

**State / status**

| Member | Type | Description |
|--------|------|-------------|
| `state` (property) | `RangerState` | Thread-safe **snapshot copy** of the latest decoded feedback. Read fields without extra locking. |
| `wait_for_feedback(timeout=2.0)` | `bool` | Block until a 0x211 system frame is decoded. `True` if it arrived, `False` on timeout. Call after construction to confirm the link and to guarantee `state.system` is populated. |
| `bus` | `CanBus` | The underlying transport (rarely needed directly). |

### `class RangerState`

A dataclass snapshot returned by `RangerAir.state`. Any field may be `None`
until its feedback frame has arrived once.

| Field / property | Type | Meaning |
|------------------|------|---------|
| `connected` | `bool` | `True` once any frame has ever been received |
| `system` | `SystemFeedback \| None` | 0x211: status, control mode, voltage, fault bytes |
| `motion` | `MotionFeedback \| None` | 0x221: body `linear_mps`, `angular_rps`, `steer_rad` |
| `motion_mode` | `MotionMode \| None` | 0x291: current kinematic mode |
| `switching_mode` | `bool` | `True` while mid mode-switch (motion ignored) |
| `wheel_speeds` | `tuple[float×4] \| None` | 0x281: per-wheel speed (wheels 1–4), m/s |
| `steer_angles` | `tuple[float×4] \| None` | 0x271: per-wheel steering angle (motors 5–8), rad |
| `last_rx_monotonic` | `float` | `time.monotonic()` of the last frame (staleness check) |
| `voltage` (property) | `float \| None` | Battery volts, or `None` if no system frame yet |
| `faults` (property) | `list[str]` | Active fault descriptions, e.g. `["EMERGENCY STOP triggered"]`; empty when healthy |

`SystemFeedback` also exposes `.estop` (`bool`), `.has_fault` (`bool`) and the
raw `.fault_bytes`.

### Enums

- **`MotionMode`** — `ACKERMANN` (0x00), `TILT` (0x01), `SPIN` (0x02), `PARK` (0x03).
- **`ControlMode`** — `STANDBY` (0x00), `CAN_COMMAND` (0x01).
- **`CanId`** — every frame identifier (`CMD_MOTION = 0x111`, `FB_SYSTEM = 0x211`, …).
- **`LIMITS`** — a frozen dataclass of SI command limits (`linear_mps=1.5`,
  `angular_rps=3.259`, `steer_ackermann_rad=0.698`, `steer_tilt_rad=1.571`, …).

### `rangerair.protocol` — codec functions

Pure functions (no I/O). Encoders return `bytes` ready for `CanBus.send`;
decoders take an 8-byte payload and return typed objects/tuples in SI units.

| Function | Returns | Purpose |
|----------|---------|---------|
| `encode_motion(linear_mps=0, angular_rps=0, steer_rad=0, mode=ACKERMANN)` | `bytes` (8) | Build a 0x111 payload; clamps to limits for `mode` |
| `encode_control_mode(mode)` | `bytes` (1) | 0x421 standby / CAN command |
| `encode_motion_mode(mode)` | `bytes` (1) | 0x141 kinematic mode |
| `encode_clear_error(code=0x00)` | `bytes` (1) | 0x441 clear-error selector |
| `encode_light(enable, on)` | `bytes` (8) | 0x121 light control |
| `decode_system(data)` | `SystemFeedback` | 0x211 |
| `decode_motion(data)` | `MotionFeedback` | 0x221 |
| `decode_motion_mode(data)` | `(MotionMode, bool)` | 0x291 → (mode, switching) |
| `decode_wheel_speeds(data)` | `tuple[float×4]` | 0x281, m/s |
| `decode_steer_angles(data)` | `tuple[float×4]` | 0x271, rad |
| `describe_faults(fault_bytes)` | `list[str]` | Human-readable active faults |

### `rangerair.bus` — transport

| Symbol | Description |
|--------|-------------|
| `CanBus(interface="can0", recv_timeout=1.0)` | Raw SocketCAN socket. `send(can_id, data)` (≤8 bytes, raises `ValueError`/`OSError`), `recv() -> CanFrame \| None` (`None` on timeout), `close()`. Context manager. |
| `CanFrame(can_id, data)` | Received frame; `.dlc` = payload length. |

---

## Operating notes from the manual

* **The remote has priority.** For CAN control, the FS remote's **SWB must be
  in the TOP position** (command-control mode). In remote mode, CAN motion
  commands are ignored and `enable()` won't take effect.
* On power-up the chassis is in **standby** and only accepts the control-mode
  command — you must `enable()` (0x421 → CAN command mode) before it moves.
* After releasing the physical e-stop button, send `clear_errors()` (0x441) to
  clear the e-stop *fault* in command mode.
* The chassis stops if it sees no 0x111 command for **500 ms** — the driver's
  heartbeat handles this, so keep the `RangerAir` object alive while driving.
* Command limits are clamped automatically (see `LIMITS`): linear ±1.5 m/s
  (±0.525 above 20° steer), spin ±3.259 rad/s, steering ±0.698 rad (Ackermann)
  / ±1.571 rad (tilt).

## CAN protocol reference

Implemented from *Ranger Air Docs EN*, §3.2 "CAN Interface Protocol"
(CAN 2.0B, 500 kbit/s, big-endian "MOTOROLA" fields):

| ID | Dir | Meaning |
|------|-----|---------|
| 0x111 | → | motion command (linear, angular, steering) |
| 0x141 | → | motion mode (ackermann/tilt/spin/park) |
| 0x421 | → | control mode (standby / CAN command) |
| 0x441 | → | clear errors / release e-stop |
| 0x121 | → | light control |
| 0x211 | ← | system status, mode, battery, fault bits |
| 0x221 | ← | body velocity + steering angle |
| 0x291 | ← | current motion mode |
| 0x271 / 0x281 | ← | per-wheel steering angles / speeds |
| 0x251–8 / 0x261–8 | ← | per-motor high/low-speed telemetry |
