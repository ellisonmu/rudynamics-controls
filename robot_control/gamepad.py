"""
Drive the leg with a PS4 (DualShock 4) controller.

Runs next to the panel server and sends it the same commands the web panel does:

    ./run_panel.sh --sim            # or ./run_panel.sh for real motors
    python3 gamepad.py              # controller over Bluetooth or USB
    python3 gamepad.py --show       # just print buttons/sticks (check the mapping)

Controls (L1 is the dead-man switch: nothing moves unless you hold it):

    hold L1 + left stick      move the foot forward/back and left/right
    hold L1 + right stick     move the foot up/down
    hold L1 + X               stand
    hold L1 + Triangle        crawl (loop)
    hold L1 + Square          step in place (loop)
    hold L1 + Options         enable (holds the current pose)
    hold L1 + D-pad up/down   move speed faster/slower (restarts a playing move)
    R1                        stop the move and hold
    let go of L1              any playing move stops and the leg holds
    Circle or PS button       E-STOP (damping), works without L1
    Share                     disable (motors limp)

If the controller disconnects, a playing move stops and the leg holds. If this script
dies, the server stops getting polled and holds by itself after 1 s.
"""

import argparse
import array
import errno
import fcntl
import glob
import json
import os
import struct
import sys
import time
import urllib.request
from urllib.error import HTTPError, URLError

# Linux joystick API (/dev/input/jsN): struct js_event { u32 time; s16 value; u8 type; u8 number; }
JS_EVENT = struct.Struct("<IhBB")
JS_BUTTON, JS_AXIS, JS_INIT = 0x01, 0x02, 0x80

# DualShock 4 layout as Linux's hid-playstation / hid-sony drivers report it.
# If something is mixed up on your controller, run --show and fix these numbers.
BUTTONS = {
    0: "cross", 1: "circle", 2: "triangle", 3: "square", 4: "l1", 5: "r1",
    6: "l2", 7: "r2", 8: "share", 9: "options", 10: "ps", 11: "l3", 12: "r3",
}
AXES = {0: "lx", 1: "ly", 2: "l2", 3: "rx", 4: "ry", 5: "r2", 6: "dpad_x", 7: "dpad_y"}


def js_name(path):
    """Device name via the JSIOCGNAME ioctl, e.g. 'Wireless Controller'."""
    buf = array.array("B", [0] * 128)
    jsiocgname = (2 << 30) | (len(buf) << 16) | (ord("j") << 8) | 0x13
    try:
        with open(path, "rb") as f:
            fcntl.ioctl(f, jsiocgname, buf)
    except OSError:
        return ""
    return buf.tobytes().split(b"\0")[0].decode(errors="replace")


def find_controller():
    """The controller's own jsN, not the extra motion-sensor/touchpad devices it creates."""
    for path in sorted(glob.glob("/dev/input/js*")):
        name = js_name(path)
        if "Motion" in name or "Touchpad" in name:
            continue
        return path, name
    return None, None


class Pad:
    """Buttons and sticks of one joystick device. Reopens it if it disconnects."""

    def __init__(self, device=None):
        self.device = device
        self.fd = None
        self.name = ""
        self.buttons = {}
        self.axes = {}
        self.pressed = set()        # buttons pressed since the last poll()
        self._next_try = 0.0

    @property
    def connected(self):
        return self.fd is not None

    def _open(self):
        path, name = (self.device, js_name(self.device)) if self.device else find_controller()
        if not path:
            return
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            return
        self.name = name or path
        self.buttons, self.axes = {}, {}

    def _close(self):
        if self.fd is not None:
            os.close(self.fd)
        self.fd = None
        self.buttons, self.axes = {}, {}

    def feed(self, data):
        """Apply raw js_event bytes (also used by the tests)."""
        for i in range(0, len(data) - len(data) % JS_EVENT.size, JS_EVENT.size):
            _, value, kind, number = JS_EVENT.unpack_from(data, i)
            init = kind & JS_INIT
            kind &= ~JS_INIT
            if kind == JS_BUTTON:
                name = BUTTONS.get(number, f"b{number}")
                if value and not self.buttons.get(name) and not init:
                    self.pressed.add(name)
                self.buttons[name] = bool(value)
            elif kind == JS_AXIS:
                self.axes[AXES.get(number, f"a{number}")] = value / 32767.0

    def poll(self):
        self.pressed = set()
        if self.fd is None:
            if time.monotonic() >= self._next_try:
                self._next_try = time.monotonic() + 1.0
                self._open()
            return
        while True:
            try:
                data = os.read(self.fd, JS_EVENT.size * 64)
            except BlockingIOError:
                return
            except OSError as e:
                if e.errno in (errno.ENODEV, errno.EIO, errno.EBADF):
                    self._close()       # unplugged / Bluetooth dropped
                    return
                raise
            if not data:
                return
            self.feed(data)

    def held(self, name):
        return self.buttons.get(name, False)

    def axis(self, name):
        return self.axes.get(name, 0.0)


class Api:
    """Talks to leg_server.py over HTTP."""

    def __init__(self, url):
        self.url = url.rstrip("/")

    def state(self):
        with urllib.request.urlopen(self.url + "/api/state", timeout=0.5) as r:
            return json.loads(r.read())

    def __call__(self, path, body=None):
        req = urllib.request.Request(
            self.url + path, data=json.dumps(body or {}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=0.5) as r:
                return json.loads(r.read())
        except HTTPError as e:
            raise ValueError(json.loads(e.read() or b"{}").get("error", f"HTTP {e.code}"))


def stick(v, deadzone):
    """Deadzone, rescaled so the output still reaches +-1."""
    if abs(v) < deadzone:
        return 0.0
    return (abs(v) - deadzone) / (1 - deadzone) * (1 if v > 0 else -1)


class Teleop:
    """Turns controller input + server state into panel commands. No I/O of its own."""

    MOVES = {"cross": "stand", "triangle": "crawl", "square": "step"}

    def __init__(self, api, foot_speed=0.15, deadzone=0.12):
        self.api = api
        self.foot_speed = foot_speed    # m/s at full stick
        self.deadzone = deadzone
        self.speed = 1.0                # move playback speed
        self.foot = None                # foot target while driving with the sticks
        self.was_connected = False
        self.l1_was_held = False
        self.last_move = None
        self.dpad_prev = 0
        self.note = ""

    def _cmd(self, path, body=None):
        try:
            return self.api(path, body)
        except ValueError as e:
            self.note = str(e)
            return None

    def step(self, pad, snap, dt):
        mode = snap["mode"]
        if not pad.connected:
            if self.was_connected and mode == "motion":
                self._cmd("/api/stop_motion")
            self.was_connected = False
            self.foot = None
            self.l1_was_held = False
            return
        self.was_connected = True
        p = pad.pressed
        l1 = pad.held("l1")

        # Safety buttons first; they never need the dead-man.
        if "circle" in p or "ps" in p:
            self._cmd("/api/estop")
            self.note = "E-STOP"
            mode = "damping"
        elif "share" in p:
            self._cmd("/api/disable")
            self.note = "disabled"
            mode = "disabled"
        elif "r1" in p and mode == "motion":
            self._cmd("/api/stop_motion")
            mode = "manual"

        if self.l1_was_held and not l1 and mode == "motion":
            self._cmd("/api/stop_motion")
            mode = "manual"
            self.note = "released L1: holding"
        self.l1_was_held = l1

        if l1 and mode in ("disabled", "damping", "manual", "motion"):
            if "options" in p and mode in ("disabled", "damping"):
                if self._cmd("/api/enable") is not None:
                    mode = "manual"
                    self.note = "enabled"
            if mode in ("manual", "motion"):
                for button, key in self.MOVES.items():
                    if button in p:
                        self._start(key)
                        mode = "motion"
                change = self._dpad_edge(pad)
                if change:
                    self.speed = max(0.25, min(2.0, self.speed + 0.25 * change))
                    self.note = f"move speed {self.speed:.2f}x"
                    if mode == "motion" and snap.get("motion"):
                        self._start(snap["motion"]["key"])
        self.dpad_prev = round(pad.axis("dpad_y"))

        # Driving the foot with the sticks (manual mode only, L1 held).
        vx = -stick(pad.axis("ly"), self.deadzone)      # stick up = foot forward
        vy = -stick(pad.axis("lx"), self.deadzone)      # stick right = foot right (-y)
        vz = -stick(pad.axis("ry"), self.deadzone)      # stick up = foot up
        if not (l1 and mode == "manual"):
            self.foot = None
            return
        if vx == vy == vz == 0.0:
            return
        if self.foot is None:
            pts = snap.get("cmd_pts")
            if not pts:
                return
            self.foot = list(pts["foot"])
        step = self.foot_speed * dt
        want = [self.foot[0] + vx * step, self.foot[1] + vy * step, self.foot[2] + vz * step]
        r = self._cmd("/api/foot", {"xyz": want})
        if r is not None:
            self.foot = r["foot"]   # where it really goes, so we never wind up past a limit

    def _dpad_edge(self, pad):
        """+1 on a fresh D-pad up press, -1 on down, else 0 (D-pad is an axis, up = -1)."""
        now = round(pad.axis("dpad_y"))
        return -now if now != 0 and now != self.dpad_prev else 0

    def _start(self, key):
        if self._cmd("/api/motion", {"key": key, "params": {"speed": self.speed}}) is not None:
            self.last_move = key
            self.note = f"{key} (speed {self.speed:.2f}x)"


def show(pad):
    """Print live controller input, to check the button/axis numbers."""
    print("waiting for a controller on /dev/input/js* (Ctrl-C to quit)")
    last = None
    while True:
        pad.poll()
        if not pad.connected:
            line = "no controller"
        else:
            held = " ".join(sorted(k for k, v in pad.buttons.items() if v)) or "-"
            axes = " ".join(f"{k}={v:+.2f}" for k, v in sorted(pad.axes.items()))
            line = f"{pad.name} | buttons: {held} | {axes}"
        if line != last:
            print("\r" + line.ljust(150), end="", flush=True)
            last = line
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="http://localhost:8080", help="panel server")
    ap.add_argument("--device", help="joystick device, e.g. /dev/input/js0 (default: find it)")
    ap.add_argument("--rate", type=float, default=50.0, help="Hz")
    ap.add_argument("--foot-speed", type=float, default=0.15, help="foot speed at full stick, m/s")
    ap.add_argument("--show", action="store_true", help="only print controller input")
    args = ap.parse_args()

    if not sys.platform.startswith("linux"):
        sys.exit("gamepad.py reads /dev/input/js*, so run it on the Jetson (Linux).")
    pad = Pad(args.device)
    if args.show:
        try:
            show(pad)
        except KeyboardInterrupt:
            print()
        return

    api = Api(args.url)
    teleop = Teleop(api, foot_speed=args.foot_speed)
    dt = 1.0 / args.rate
    next_t = time.monotonic()
    last_line = None
    print("PS4 teleop. Hold L1 to move, Circle = E-STOP. Ctrl-C to quit.")
    try:
        while True:
            pad.poll()
            try:
                snap = api.state()          # also the server's heartbeat
            except (URLError, OSError, ValueError):
                snap = None
            if snap is None:
                line = f"cannot reach the panel server at {args.url} (is ./run_panel.sh running?)"
            else:
                teleop.step(pad, snap, dt)
                ctrl = pad.name if pad.connected else "NO CONTROLLER"
                foot = snap.get("cmd_pts", snap.get("measured_pts", {})).get("foot")
                foot_s = " ".join(f"{v:+.3f}" for v in foot) if foot else "--"
                l1 = "L1" if pad.held("l1") else "--"
                line = f"[{snap['mode']:8s}] {l1} {ctrl} | foot {foot_s} | {teleop.note or snap['message']}"
            if line != last_line:
                print("\r" + line[:150].ljust(150), end="", flush=True)
                last_line = line
            next_t += dt
            time.sleep(max(0.0, next_t - time.monotonic()))
            if time.monotonic() - next_t > 0.5:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if api.state()["mode"] == "motion":
                api("/api/stop_motion")
        except Exception:
            pass
        print("\nbye (leg left holding its pose)")


if __name__ == "__main__":
    main()
