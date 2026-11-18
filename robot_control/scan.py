"""
Find motors on a serial port: tries IDs 0-14 with a limp (zero-gain) command.

    python3 scan.py                       # /dev/ttyUSB0, official SDK
    python3 scan.py --port /dev/ttyUSB1
    python3 scan.py --backend pyserial    # pure-Python driver
    python3 scan.py --sim                 # no hardware
"""

import argparse
import glob

from motors import GEAR_RATIO, MODE_FOC, SimBackend, make_backend


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--backend", choices=["sdk", "pyserial"], default="sdk")
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()

    if not args.sim:
        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        print("serial ports found:", ports or "NONE (is the RS-485 adapter plugged in?)")

    backend = SimBackend(ids=[0]) if args.sim else make_backend(args.port, args.backend)
    found = []
    for motor_id in range(15):
        fb = None
        for _ in range(3):
            fb = backend.send_recv(motor_id, MODE_FOC)
            if fb:
                break
        if fb:
            found.append(motor_id)
            print(f"  id {motor_id:2d}: output q={fb['q'] / GEAR_RATIO:+.3f} rad  "
                  f"temp={fb['temp']}C  error={fb['error']}")
    print(f"motors found: {found}" if found else "no motors replied")
    backend.close()


if __name__ == "__main__":
    main()
