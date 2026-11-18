"""
Pure-Python driver for the Unitree GO-M8010-6 motor (works on macOS, no SDK .so needed).

Protocol (see include/unitreeMotor/include/motor_msg_GO-M8010-6.h):
  command  17 bytes: FE EE | mode | tau q8 | dq q7 | q q15 | kp | kd | CRC16
  feedback 16 bytes: FD EE | mode | tau q8 | dq q7 | q q15 | temp | err/force | CRC16
  RS-485 at 4,000,000 baud, 8N1, little-endian, CRC-CCITT (Linux crc_ccitt, init 0).

All commanded/returned values are on the ROTOR side. Output = rotor / GEAR_RATIO.

Usage:
  python3 go_m8010.py ports                 # list serial ports
  python3 go_m8010.py read                  # read state, motor stays limp (safe)
  python3 go_m8010.py spin --speed 1.0      # spin output shaft at 1.0 rad/s
  python3 go_m8010.py hold                  # hold current position (spring-like)
"""

import argparse
import math
import struct
import sys
import time

import serial
import serial.tools.list_ports

BAUD = 4_000_000
GEAR_RATIO = 6.33
MODE_BRAKE, MODE_FOC, MODE_CALIBRATE = 0, 1, 2
ERRORS = {0: "ok", 1: "overheat", 2: "overcurrent", 3: "overvoltage", 4: "encoder fault"}


def crc_ccitt(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def pack_cmd(motor_id=0, mode=MODE_FOC, tau=0.0, dq=0.0, q=0.0, kp=0.0, kd=0.0) -> bytes:
    """Build a 17-byte command. All values rotor-side (N.m, rad/s, rad)."""
    mode_byte = (motor_id & 0x0F) | ((mode & 0x07) << 4)
    tau = clamp(tau, -127.99, 127.99)
    dq = clamp(dq, -804.0, 804.0)
    q = clamp(q, -411774.0, 411774.0)
    kp = clamp(kp, 0.0, 25.599)
    kd = clamp(kd, 0.0, 25.599)
    body = struct.pack(
        "<BBBhhiHH",
        0xFE, 0xEE, mode_byte,
        int(tau * 256),
        int(dq / (2 * math.pi) * 256),
        int(q / (2 * math.pi) * 32768),
        int(kp / 25.6 * 32768),
        int(kd / 25.6 * 32768),
    )
    return body + struct.pack("<H", crc_ccitt(body))


def unpack_fbk(pkt: bytes):
    """Parse a 16-byte feedback packet. Returns dict or None if invalid."""
    if len(pkt) != 16 or pkt[0] != 0xFD or pkt[1] != 0xEE:
        return None
    if crc_ccitt(pkt[:14]) != struct.unpack_from("<H", pkt, 14)[0]:
        return None
    mode_byte, tau, dq, q, temp, bits = struct.unpack_from("<BhhibH", pkt, 2)
    return {
        "id": mode_byte & 0x0F,
        "mode": (mode_byte >> 4) & 0x07,
        "tau": tau / 256,                         # rotor N.m
        "dq": dq / 256 * 2 * math.pi,             # rotor rad/s
        "q": q / 32768 * 2 * math.pi,             # rotor rad
        "temp": temp,                             # deg C
        "error": bits & 0x07,
        "force": (bits >> 3) & 0x0FFF,
    }


class GoM8010:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, timeout=0.01)
        self.ser.reset_input_buffer()

    def send_recv(self, **cmd):
        self.ser.reset_input_buffer()
        self.ser.write(pack_cmd(**cmd))
        buf = self.ser.read(32)
        # Find a valid 16-byte frame anywhere in what came back (skips any echo/noise).
        for i in range(len(buf) - 15):
            if buf[i] == 0xFD and buf[i + 1] == 0xEE:
                fb = unpack_fbk(buf[i:i + 16])
                if fb:
                    return fb
        return None

    def stop(self, motor_id=0):
        self.send_recv(motor_id=motor_id, mode=MODE_BRAKE)

    def close(self):
        self.ser.close()


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports()
             if "usb" in p.device.lower() and p.device.startswith("/dev/cu.")]
    if not ports:
        sys.exit("No USB serial adapter found. Is the RS-485 adapter plugged in? Try: python3 go_m8010.py ports")
    return ports[0]


def print_state(fb):
    if fb is None:
        print("no valid reply")
        return
    print(f"id={fb['id']} mode={fb['mode']}  "
          f"out_q={fb['q'] / GEAR_RATIO:+8.3f} rad  "
          f"out_dq={fb['dq'] / GEAR_RATIO:+7.3f} rad/s  "
          f"out_tau={fb['tau'] * GEAR_RATIO:+6.2f} Nm  "
          f"temp={fb['temp']}C  err={ERRORS.get(fb['error'], fb['error'])}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["ports", "read", "spin", "hold"])
    ap.add_argument("--port", help="e.g. /dev/cu.usbserial-XXXX (auto-detected if omitted)")
    ap.add_argument("--id", type=int, default=0, help="motor ID (default 0)")
    ap.add_argument("--speed", type=float, default=1.0, help="output-shaft speed for spin, rad/s")
    ap.add_argument("--rate", type=float, default=200.0, help="loop rate, Hz")
    args = ap.parse_args()

    if args.action == "ports":
        for p in serial.tools.list_ports.comports():
            print(f"{p.device:40s} {p.description}")
        return

    port = args.port or find_port()
    print(f"Opening {port} at {BAUD} baud. Ctrl-C to stop.")
    motor = GoM8010(port)
    period = 1.0 / args.rate
    hold_q = None
    misses = 0

    try:
        while True:
            if args.action == "read":
                # FOC mode with zero gains = motor is limp, just reports state.
                fb = motor.send_recv(motor_id=args.id, mode=MODE_FOC)
            elif args.action == "spin":
                # Velocity control: kd is rotor-side damping gain.
                fb = motor.send_recv(motor_id=args.id, mode=MODE_FOC,
                                     dq=args.speed * GEAR_RATIO, kd=0.01)
            else:  # hold
                if hold_q is None:
                    first = motor.send_recv(motor_id=args.id, mode=MODE_FOC)
                    if first is None:
                        misses += 1
                        time.sleep(period)
                        continue
                    hold_q = first["q"]
                fb = motor.send_recv(motor_id=args.id, mode=MODE_FOC,
                                     q=hold_q, kp=0.02, kd=0.01)
            if fb is None:
                misses += 1
                if misses % 50 == 1:
                    print(f"no valid reply ({misses} misses) - check wiring, power, ID, baud")
            else:
                print_state(fb)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        motor.stop(args.id)
        motor.close()
        print("\nStopped (brake mode sent).")


if __name__ == "__main__":
    main()
