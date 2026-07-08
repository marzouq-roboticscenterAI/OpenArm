# ds2c — Python control library for the DS2-C servo

A Python port of `ds2c_test_move.sh`. It speaks the same CANopen / CiA402
protocol your friend's script uses, but as an importable library built on
[`python-can`](https://python-can.readthedocs.io/) instead of shelling out to
`cansend`/`candump`. It also **reads live data the script never did**: actual
position, actual velocity, mode, and a decoded drive state.

## ⚠️ Safety

This is a **software stop only**. If the CAN adapter is unplugged or the drive
hangs, this library cannot stop the motor. Keep a hand on real mains/DC power
(or a physical E-stop) whenever you run it.

**Direction is inverted** on this rig (confirmed on hardware, per the script):
positive pulses / positive velocity move the carriage **physically DOWN**.
Use `raise_by`/`lower_by`/`jog_up`/`jog_down` for unambiguous up/down.

## Install / setup

```bash
pip install python-can          
```

Bring the CAN interface up once (needs sudo — same commands the script runs):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

or from Python: `DS2CServo(...).bring_up_interface()`.

## Quick start

```python
from ds2c import DS2CServo

with DS2CServo(channel="can0", node=1) as servo:   # auto E-stop on error/Ctrl+C
    servo.check()                 # pre-flight: statusword + clear fault
    print(servo.read_state())     # e.g. 0x0237 (Operation enabled)
    print(servo.read_position())  # live position in pulses

    servo.enable()                # CiA402 enable sequence (6 -> 7 -> 15)
    servo.configure_position_mode(velocity=10000, accel=10000, decel=10000)

    servo.lower_by(20000)           # raise/lower helpers hide the inverted polarity
    servo.raise_by(20000)
```

## Reading data (new vs. the shell script)

| Method | Object | Meaning |
|---|---|---|
| `read_statusword()` | 0x6041 | raw statusword (u16) |
| `read_state()` | 0x6041 | decoded `DriveState` (name + fault/enabled/target-reached flags) |
| `read_position()` | 0x6064 | actual position, pulses (signed) |
| `read_position_rev()` | 0x6064 | actual position, revolutions |
| `read_velocity()` | 0x606C | actual velocity, pulses/s (signed) |
| `read_mode()` | 0x6061 | active mode of operation |

Raw access is available too: `sdo_upload(index, sub)` returns the data bytes,
`sdo_download(index, sub, size, value)` writes (handles signed values + framing).

**Is this a real measurement?** Yes — `read_position` (0x6064) and
`read_velocity` (0x606C) are the drive's own **encoder feedback**, read live over
SDO. They are not computed from the pulses we commanded. (These are standard but
*optional* CiA402 objects; if this particular DS2-C doesn't implement one, the
read raises `SDOAbort` rather than returning a bogus value. `--status` on real
hardware is the definitive confirmation.)

## Speed limiting (two layers)

Neither the shell script nor motors themselves impose a speed limit — whatever
velocity you command is what runs. This library adds two guards:

1. **Software limit** (`max_velocity_pps`, default **20000 pps = 120 rpm**). Any
   commanded velocity above it raises `SpeedLimitExceeded` and sends nothing.
   ```python
   servo = DS2CServo(max_velocity_pps=2000)   # 12 rpm — very gentle
   servo.set_speed_limit(5000)                # change it live (30 rpm)
   servo.set_speed_limit(None)                # disable (not recommended)
   ```
2. **Drive-enforced ceiling** — `apply_drive_speed_limit()` writes max profile
   velocity (0x607F) and max motor speed (0x6080), so the **drive clamps speed
   internally**, even against commands that bypass this library. Call it once
   after `open()`, before enabling. Raises `SDOAbort` if the drive lacks these
   optional objects (then only the software limit applies).

Unit helpers `pps_to_rpm()` / `rpm_to_pps()` convert using the 10000 pulses/rev
gearing, so you can reason in rpm. Jog defaults were lowered to 5000 pps (30 rpm).

## Motion API

- **Relative move** (profile position): `move_relative(pulses, velocity=, accel=, decel=, wait=True, timeout=10)`
  — waits for the *target-reached* bit, then drops controlword bit4 so the next
  move gets a fresh rising edge (exactly like the script).
- **Absolute move**: `move_absolute(target_pulses)` — reads current position and
  issues the equivalent relative move.
- **Continuous jog** (profile velocity): `jog(velocity, max_seconds=20)` — configures
  velocity mode *before* enabling (required by this drive), then auto-stops after
  the cap. `jog_up()`/`jog_down()` wrap it with the correct sign.
- **Direction helpers**: `raise_by`, `lower_by`, `jog_up`, `jog_down`.

## Stopping

- `quick_stop()` — controlword 0x0B (decelerates on the quick-stop ramp).
- `emergency_stop()` — quick-stop **then** disable voltage; idempotent. This is
  the software E-stop the `with` block fires automatically on any exception.
- `disable()` — de-energize.

## Example CLI

`example.py` mirrors the script's flags:

```bash
python3 example.py                # check + tiny down/up move (reads position)
python3 example.py --check-only   # connectivity + fault check only
python3 example.py --status       # print live state/position/velocity, exit
python3 example.py --jog-up       # continuous jog UP, auto-stops after 20s
python3 example.py --estop        # quick-stop + disable, exit
```

## Protocol reference (matches the script)

| Object | Index | Notes |
|---|---|---|
| Controlword | 0x6040 | enable 6→7→15, rel-move start 0x5F, quick-stop 0x0B, fault-reset 0x80 |
| Statusword | 0x6041 | fault bit3, target-reached bit10, op-enabled `(sw & 0x6F)==0x27` |
| Mode of operation | 0x6060 | 1 = profile position, 3 = profile velocity |
| Target position | 0x607A | relative, pulses |
| Profile velocity/accel/decel | 0x6081 / 0x6083 / 0x6084 | pulses/s, pulses/s² |
| Target velocity | 0x60FF | profile-velocity mode, pulses/s |

Node 1, 1,000,000 bps, 10000 pulses/rev. SDO client COB-ID `0x600+node`,
server responses `0x580+node`.
