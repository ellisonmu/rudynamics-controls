# Robot control (GO-M8010-6 on Jetson Orin Nano)

Python control layer on top of the Unitree SDK. All commands and readings are in
**output-shaft units** (rad, rad/s, N.m); gear ratio (6.33) and rotor-side gain
scaling are handled for you.

| File | What it is |
|---|---|
| `motors.py` | `Motor`, `Robot`, `RateLoop`, safety checks, backends (SDK / pyserial / sim) |
| `robot_config.py` | Which motors exist: name, ID, port, direction, limits. **Edit this.** |
| `scan.py` | Find which motor IDs respond on a port |
| `demo.py` | `read`, `hold`, `sine`, `spin` bring-up demos, optional CSV logging |
| `test_motors.py` | Hardware-free tests (`python3 test_motors.py`) |

| `leg.py` | 3-DOF leg kinematics (FK/IK) and the motion library (stand, extend, tuck, crawl, ...) |
| `leg_server.py` | Web control panels + 500 Hz control loop for the leg |
| `run_panel.sh` | Start/restart/stop the panel in the background |
| `test_leg.py` | Hardware-free tests for kinematics, motions and the panel controller |
| `gamepad.py` | Drive the leg with a PS4 controller (talks to the panel server) |
| `test_gamepad.py` | Hardware-free tests for the controller logic |

Everything runs without hardware by adding `--sim`.

## Leg control panels

```bash
cd ~/unitree_actuator_sdk/robot_control
./run_panel.sh --sim      # simulated leg     (./run_panel.sh for real motors, ./run_panel.sh stop)
```
Open **http://192.168.55.1:8080** in a browser on the Mac.

- **Manual** (`/manual`): a slider per joint. Targets are approached at 3 rad/s max, so
  yanking a slider can't snap the leg. Green tick = measured position. kp/kd sliders.
- **Moves** (`/moves`): Stand, Extend, Tuck, Kick (one-shot) and Crawl, Step in place,
  Circle, Sway (looping), each with speed/stride/height settings. Every move eases in from
  the current pose; all foot targets are kept inside joint limits and short of full extension.

Safety behaviour:
- **E-STOP** button or **Space**: damping mode (no stiffness, gentle resistance).
- **Disable**: motors limp. **Enable** starts by holding the current pose.
- If the browser stops polling for 1 s (tab closed, Wi-Fi drop) the leg stops the move and holds.
- Motor error / over-temp / missing motor -> fault mode, motors braked until **Clear fault**.

**Calibration (real leg):** the panel won't enable until calibrated. Disable the leg,
hang it straight down (all joints at zero, see `leg.py` for conventions), press
**Calibrate**. Do this every power-up. Then check each slider moves its joint the way
the side/front views show; if one is backwards, set `direction=-1` for it in
`robot_config.py`. Also set the real thigh/shank lengths in `LEG` there.

## PS4 controller

Start the panel server first, then the controller script, both on the Jetson:
```bash
./run_panel.sh --sim          # or ./run_panel.sh for real motors
python3 gamepad.py --show     # check the controller is seen and buttons are right
python3 gamepad.py            # drive the leg
```

**L1 is the dead-man switch:** nothing moves unless you hold it.

| Input | Does |
|---|---|
| L1 + left stick | foot forward/back, left/right |
| L1 + right stick (up/down) | foot up/down |
| L1 + X / Triangle / Square | Stand / Crawl / Step in place |
| L1 + Options | Enable |
| L1 + D-pad up/down | move speed faster/slower |
| R1 | stop move, hold |
| let go of L1 | stop move, hold |
| **Circle** or **PS** | **E-STOP** (no L1 needed) |
| Share | Disable (limp) |

The web panel keeps working alongside the controller. If the controller disconnects, any
playing move stops and the leg holds.

**Pairing over Bluetooth (once):**
```bash
bluetoothctl
  scan on               # hold Share + PS on the controller until the light bar flashes fast
  pair XX:XX:XX:XX:XX:XX   # the "Wireless Controller" address
  trust XX:XX:XX:XX:XX:XX
  connect XX:XX:XX:XX:XX:XX
  quit
```
After that, pressing PS reconnects it. Or just plug it in with a USB cable.
If `ls /dev/input/js*` shows nothing, run `sudo modprobe joydev`.

## Jetson setup (already done)

- JetPack 6.2 (L4T R36.4.3), user `rudynamics`, reachable at `192.168.55.1` over the USB-C cable
- SDK built in `~/unitree_actuator_sdk/build`, ARM64 Python module in `lib/`
- pyserial + pip installed (`~/.local`), user in `dialout`, udev rule `/etc/udev/rules.d/99-unitree-motor.rules`
- Verified: the pure-Python driver in `../macos/go_m8010.py` produces byte-identical packets to the official SDK

## First time with a motor

1. **Clamp the motor down.** Nothing attached to the output shaft.
2. Power the motor (24 V) and plug the RS-485 adapter into a Jetson USB-A port.
3. Check the adapter shows up:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```
   - Nothing at all: the adapter's chip may be unsupported (`dmesg | tail` to see). FTDI and CP210x work out of the box; CH340 does not (and can't do 4 Mbaud anyway).
   - `/dev/ttyACM0` instead of `ttyUSB0`: use `--port /dev/ttyACM0` and update `robot_config.py`.
4. Official SDK sanity check:
   ```bash
   cd ~/unitree_actuator_sdk/build && ./example_goM8010_6_motor   # spins ~1 rev/s, Ctrl-C to stop
   ```
5. Then the control layer:
   ```bash
   cd ~/unitree_actuator_sdk/robot_control
   python3 scan.py                  # expect: motors found: [0]
   python3 demo.py read             # turn the shaft by hand, q should change
   python3 demo.py hold             # it should resist you like a spring
   python3 demo.py sine --amp 0.3 --freq 0.5 --log sine.csv
   ```

**No reply from the motor?** Swap the RS-485 A/B wires, check 24 V, check the
LED (slow blink = motor mode; fast blink = bootloader, run `motor_tools` `swmotor`),
and make sure the ID matches (new motors are ID 0).

## Multiple motors

Motors sharing one RS-485 bus need unique IDs (0-14). Connect **one motor at a
time** and run `build/changeID`, then add it to `robot_config.py`.

## Things to know

- **Position zero is wherever the motor powered up** (the 6.33:1 gearbox means the
  output angle isn't absolute). Use `motor.set_zero_here()` after moving to a
  known pose, or add a homing routine.
- `tau_max` only limits the feed-forward torque. The PD part is computed inside the
  motor, so keep `kp`/`kd` limits sensible. Bring-up defaults are conservative.
- Faults (motor error, over-temperature at 75 C, 20 missed replies) raise
  `SafetyFault`; `Robot` brakes every motor on exit.
- The Jetson has no RTC battery: its clock resets when unplugged until it gets
  internet. Connect Wi-Fi with
  `sudo nmcli device wifi connect "NAME" password "PASS"` (or a phone hotspot).

## Writing robot code

```python
from motors import Robot, RateLoop
from robot_config import MOTORS

with Robot(MOTORS) as robot:              # Robot(MOTORS, "sim") for no hardware
    robot.read_all()
    for t in RateLoop(500, duration=5):
        s = robot["joint0"].command(q=0.5, kp=10, kd=0.5)
```
