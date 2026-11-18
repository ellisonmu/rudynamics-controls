"""
Bring-up demos for every motor in robot_config.MOTORS. Ctrl-C brakes all motors.

    python3 demo.py read                  # limp, print state (turn shaft by hand)
    python3 demo.py hold                  # hold the current position like a spring
    python3 demo.py sine --amp 0.5 --freq 0.5
    python3 demo.py spin --speed 1.0      # velocity control, rad/s at the output
    add --sim to run any of these without hardware
    add --log run.csv to record state
"""

import argparse
import csv
import math

from motors import Robot, RateLoop, SafetyFault
from robot_config import MOTORS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["read", "hold", "sine", "spin"])
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--backend", choices=["sdk", "pyserial"], default="sdk")
    ap.add_argument("--rate", type=float, default=500.0, help="control loop Hz")
    ap.add_argument("--duration", type=float, default=None, help="seconds (default: until Ctrl-C)")
    ap.add_argument("--kp", type=float, default=10.0, help="output-side stiffness, N.m/rad")
    ap.add_argument("--kd", type=float, default=0.5, help="output-side damping, N.m.s/rad")
    ap.add_argument("--amp", type=float, default=0.5, help="sine amplitude, rad")
    ap.add_argument("--freq", type=float, default=0.5, help="sine frequency, Hz")
    ap.add_argument("--speed", type=float, default=1.0, help="spin speed, rad/s")
    ap.add_argument("--log", help="CSV file to record state")
    args = ap.parse_args()

    log = csv.writer(open(args.log, "w", newline="")) if args.log else None
    if log:
        log.writerow(["t", "motor", "q_des", "q", "dq", "tau", "temp"])

    with Robot(MOTORS, "sim" if args.sim else args.backend) as robot:
        start = robot.read_all()
        if any(s is None for s in start.values()):
            missing = [n for n, s in start.items() if s is None]
            print(f"no reply from {missing}; check wiring/IDs (try scan.py)")
        home = {n: (s.q if s else 0.0) for n, s in start.items()}

        loop = RateLoop(args.rate, args.duration)
        print_every = max(1, int(args.rate / 10))
        try:
            for i, t in enumerate(loop):
                for m in robot:
                    name = m.cfg.name
                    q_des = home[name]
                    if args.mode == "read":
                        s = m.read()
                    elif args.mode == "hold":
                        s = m.command(q=q_des, kp=args.kp, kd=args.kd)
                    elif args.mode == "sine":
                        w = 2 * math.pi * args.freq
                        q_des = home[name] + args.amp * math.sin(w * t)
                        dq_des = args.amp * w * math.cos(w * t)
                        s = m.command(q=q_des, dq=dq_des, kp=args.kp, kd=args.kd)
                    else:  # spin
                        s = m.command(dq=args.speed, kd=args.kd)
                    if s is None:
                        continue
                    if log:
                        log.writerow([f"{t:.4f}", name, f"{q_des:.4f}", f"{s.q:.4f}",
                                      f"{s.dq:.4f}", f"{s.tau:.3f}", s.temp])
                    if i % print_every == 0:
                        print(f"t={t:6.2f}s {name}: q={s.q:+7.3f} (des {q_des:+7.3f}) "
                              f"dq={s.dq:+6.2f} tau={s.tau:+6.2f}Nm temp={s.temp}C")
        except KeyboardInterrupt:
            pass
        except SafetyFault as e:
            print(f"SAFETY FAULT: {e}")
        finally:
            if loop.overruns:
                print(f"loop overran {loop.overruns} times (rate too high?)")
            print("braking all motors")


if __name__ == "__main__":
    main()
