"""
Web control panels for the 3-motor leg.

    python3 leg_server.py --sim          # simulated motors
    python3 leg_server.py                # real motors via the official SDK

Then open http://192.168.55.1:8080 on the Mac (or http://localhost:8080 locally).
  /manual  sliders per joint
  /moves   preset motions (stand, extend, crawl, ...)

Modes:
  disabled  motors limp, state is only read
  damping   E-STOP: kp = 0, motors resist motion gently (safe to touch)
  manual    holds slider targets
  motion    runs a preset motion
  fault     a SafetyFault happened; motors braked until cleared
"""

import argparse
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from leg import MOTIONS, fk, ik, motion_catalog, within_reach
from motors import Robot, RateLoop, SafetyFault
from robot_config import LEG, MOTORS

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
HEARTBEAT_TIMEOUT = 1.0     # s without a browser poll -> stop motion and hold
DAMPING_KD = 1.0            # output-side kd used in E-STOP damping mode


class LegController:
    def __init__(self, robot, geom, rate=500.0, calibrated=False, clock=time.monotonic):
        self.clock = clock
        self.robot = robot
        self.motors = list(robot)
        self.geom = geom
        self.rate = rate
        self.lock = threading.Lock()
        self.mode = "disabled"
        self.message = ""
        self.calibrated = calibrated
        self.kp, self.kd = 15.0, 0.6
        self.manual_slew = 3.0      # rad/s, how fast manual targets are approached
        self.motion_slew = 20.0     # rad/s, safety cap on motion commands
        self.target = None          # desired joint targets (manual)
        self.cmd = None             # slew-limited targets actually sent
        self.motion = None
        self.motion_t0 = 0.0
        self.motion_done = False
        self.last_seen = self.clock()
        self.loop_hz = 0.0
        self.states = [None] * len(self.motors)
        self.running = True

    # ---------------- requests from the web UI (all take the lock) ----------------

    def touch(self):
        self.last_seen = self.clock()

    def _measured(self):
        if any(s is None for s in self.states):
            raise ValueError("no state from every motor yet")
        return [s.q for s in self.states]

    def _clamp(self, q):
        return [max(m.cfg.q_min, min(m.cfg.q_max, v)) for m, v in zip(self.motors, q)]

    def enable(self):
        with self.lock:
            if self.mode == "fault":
                raise ValueError("clear the fault first")
            if not self.calibrated:
                raise ValueError("calibrate first: pose the leg straight down, then press Calibrate")
            q = self._measured()
            self.cmd = list(q)
            self.target = list(q)       # hold exactly where it is; sliders are clamped to limits
            self.motion = None
            self.mode = "manual"
            self.message = "enabled: holding current pose"

    def disable(self):
        with self.lock:
            if self.mode != "fault":
                self.mode = "disabled"
            self.motion = None
            self.message = "disabled: motors limp"

    def estop(self):
        with self.lock:
            self.mode = "damping"
            self.motion = None
            self.message = "E-STOP: damping mode"

    def set_manual(self, q):
        with self.lock:
            if self.mode not in ("manual", "motion"):
                raise ValueError("enable the leg first")
            if len(q) != len(self.motors) or not all(math.isfinite(v) for v in q):
                raise ValueError("bad joint target")
            self.motion = None
            self.mode = "manual"
            self.target = self._clamp(q)

    def set_foot(self, xyz):
        """Move the foot to (x, y, z) metres via IK. Returns where the foot will actually go
        after reach and joint limits are applied."""
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            raise ValueError("bad foot target")
        self.set_manual(ik(*within_reach(xyz, self.geom), self.geom))
        with self.lock:
            return fk(self.target, self.geom)[1]

    def start_motion(self, key, params):
        with self.lock:
            if self.mode not in ("manual", "motion"):
                raise ValueError("enable the leg first")
            if key not in MOTIONS:
                raise ValueError(f"unknown motion {key!r}")
            self.motion = MOTIONS[key](self.cmd, self.geom, **params)
            self.motion_t0 = self.clock()
            self.motion_done = False
            self.mode = "motion"
            self.message = f"running {self.motion.label}"

    def stop_motion(self):
        with self.lock:
            if self.mode == "motion":
                self.motion = None
                self.target = self._clamp(self.cmd)
                self.mode = "manual"
                self.message = "motion stopped: holding"

    def set_gains(self, kp=None, kd=None, slew=None):
        with self.lock:
            if slew is not None:
                self.manual_slew = max(0.1, min(10.0, float(slew)))
            if kp is not None:
                self.kp = float(kp)
            if kd is not None:
                self.kd = float(kd)

    def calibrate(self):
        """Declare the current physical pose to be all-zero (leg straight down)."""
        with self.lock:
            if self.mode not in ("disabled", "damping"):
                raise ValueError("disable the leg before calibrating")
            for m in self.motors:
                m.set_zero_here(0.0)
            self.calibrated = True
            self.message = "calibrated: current pose is now straight down"

    def clear_fault(self):
        with self.lock:
            for m in self.motors:
                m.missed = 0
            self.mode = "disabled"
            self.message = "fault cleared"

    # ---------------- control loop ----------------

    def _slew(self, desired, limit, dt):
        step = limit * dt
        self.cmd = [c + max(-step, min(step, d - c)) for c, d in zip(self.cmd, desired)]

    def run(self):
        dt = 1.0 / self.rate
        count, t_rate = 0, self.clock()
        for _ in RateLoop(self.rate):
            if not self.running:
                break
            self.tick(dt)
            count += 1
            now = self.clock()
            if now - t_rate >= 0.5:
                self.loop_hz = count / (now - t_rate)
                count, t_rate = 0, now

    def tick(self, dt):
        """One control cycle; any error drops into fault mode and brakes the motors."""
        with self.lock:
            try:
                self._step(dt)
            except Exception as e:  # never let the loop die silently
                self.mode = "fault"
                self.motion = None
                self.message = f"FAULT: {e}" if isinstance(e, SafetyFault) else f"FAULT: {type(e).__name__}: {e}"
                self.robot.brake_all()

    def _step(self, dt):
        if self.mode in ("manual", "motion") and self.clock() - self.last_seen > HEARTBEAT_TIMEOUT:
            self.motion = None
            self.target = list(self.cmd)
            self.mode = "manual"
            self.message = "lost contact with the panel: holding position"

        if self.mode == "fault":
            for i, m in enumerate(self.motors):
                try:
                    self.states[i] = m.read()
                except SafetyFault:
                    pass
            return

        if self.mode == "disabled":
            self.states = [m.read() for m in self.motors]
            return

        if self.mode == "damping":
            self.states = [m.command(kd=DAMPING_KD) for m in self.motors]
            return

        dq_ff = [0.0] * len(self.motors)
        if self.mode == "motion":
            t = self.clock() - self.motion_t0
            q, done = self.motion.joints(t)
            q2, _ = self.motion.joints(t + 0.005)
            dq_ff = [(b - a) / 0.005 for a, b in zip(q, q2)]
            self.motion_done = done
            self._slew(self._clamp(q), self.motion_slew, dt)
        else:
            self._slew(self.target, self.manual_slew, dt)
            dq_ff = [0.0] * len(self.motors)

        self.states = [m.command(q=c, dq=v, kp=self.kp, kd=self.kd)
                       for m, c, v in zip(self.motors, self.cmd, dq_ff)]

    # ---------------- snapshot for the UI ----------------

    def snapshot(self):
        with self.lock:
            joints = []
            for m, s, i in zip(self.motors, self.states, range(len(self.motors))):
                joints.append({
                    "name": m.cfg.name, "id": m.cfg.id,
                    "q": s.q if s else None, "dq": s.dq if s else None,
                    "tau": s.tau if s else None, "temp": s.temp if s else None,
                    "cmd": self.cmd[i] if self.cmd else None,
                    "target": self.target[i] if self.target else None,
                    "min": m.cfg.q_min, "max": m.cfg.q_max, "missed": m.missed,
                })
            measured = [j["q"] for j in joints]
            snap = {
                "mode": self.mode, "message": self.message, "calibrated": self.calibrated,
                "kp": self.kp, "kd": self.kd, "slew": self.manual_slew, "loop_hz": round(self.loop_hz),
                "joints": joints,
                "motion": None,
            }
            if None not in measured:
                knee, foot = fk(measured, self.geom)
                snap["measured_pts"] = {"knee": knee, "foot": foot}
            if self.cmd and self.mode in ("manual", "motion"):
                knee, foot = fk(self.cmd, self.geom)
                snap["cmd_pts"] = {"knee": knee, "foot": foot}
            if self.mode == "motion" and self.motion:
                snap["motion"] = {
                    "key": self.motion.key, "label": self.motion.label,
                    "t": self.clock() - self.motion_t0, "done": self.motion_done,
                }
            return snap

    def config(self):
        return {
            "joints": [{"name": m.cfg.name, "id": m.cfg.id, "min": m.cfg.q_min, "max": m.cfg.q_max}
                       for m in self.motors],
            "geometry": {"thigh": self.geom.thigh, "shank": self.geom.shank},
            "motions": motion_catalog(),
        }


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------

PAGES = {"/": "manual.html", "/manual": "manual.html", "/moves": "moves.html"}
TYPES = {".html": "text/html", ".css": "text/css", ".js": "application/javascript"}


ROUTES = {
    "/api/enable": lambda c, b: c.enable(),
    "/api/disable": lambda c, b: c.disable(),
    "/api/estop": lambda c, b: c.estop(),
    "/api/manual": lambda c, b: c.set_manual([float(v) for v in b["q"]]),
    "/api/foot": lambda c, b: {"foot": c.set_foot([float(v) for v in b["xyz"]])},
    "/api/motion": lambda c, b: c.start_motion(b["key"], b.get("params", {})),
    "/api/stop_motion": lambda c, b: c.stop_motion(),
    "/api/gains": lambda c, b: c.set_gains(b.get("kp"), b.get("kd"), b.get("slew")),
    "/api/calibrate": lambda c, b: c.calibrate(),
    "/api/clear_fault": lambda c, b: c.clear_fault(),
}


def dispatch(ctl, path, body):
    """Run one POST command. Raises ValueError/KeyError/TypeError on a bad request."""
    return ROUTES[path](ctl, body)


def make_handler(ctl: LegController):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/api/state":
                ctl.touch()
                return self._send(200, ctl.snapshot())
            if path == "/api/config":
                return self._send(200, ctl.config())
            name = PAGES.get(path) or path.lstrip("/")
            file = os.path.normpath(os.path.join(WEB_DIR, name))
            if not file.startswith(WEB_DIR) or not os.path.isfile(file):
                return self._send(404, {"error": "not found"})
            with open(file, "rb") as f:
                self._send(200, f.read(), TYPES.get(os.path.splitext(file)[1], "text/plain"))

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._send(400, {"error": "bad json"})
            if self.path not in ROUTES:
                return self._send(404, {"error": "not found"})
            ctl.touch()
            try:
                result = dispatch(ctl, self.path, body)
            except (ValueError, KeyError, TypeError) as e:
                return self._send(400, {"error": str(e)})
            self._send(200, {"ok": True, **(result or {})})

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="simulated motors")
    ap.add_argument("--backend", choices=["sdk", "pyserial"], default="sdk")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--kp", type=float, default=None, help="starting stiffness (default: 5 real, 15 sim)")
    ap.add_argument("--kd", type=float, default=None, help="starting damping (default: 0.3 real, 0.6 sim)")
    ap.add_argument("--slew", type=float, default=None,
                    help="max slider speed, rad/s (default 3; adjustable in the panel)")
    args = ap.parse_args()

    robot = Robot(MOTORS, "sim" if args.sim else args.backend)
    if args.sim:
        # Start the simulated leg in a natural crouch instead of perfectly straight.
        for m, q in zip(robot, [0.0, 0.8, -1.6]):
            m.backend.motors[m.cfg.id].q = q
    ctl = LegController(robot, LEG, rate=args.rate, calibrated=args.sim)
    # Gentle defaults on real hardware until the leg is tuned.
    ctl.kp = args.kp if args.kp is not None else (15.0 if args.sim else 5.0)
    ctl.kd = args.kd if args.kd is not None else (0.6 if args.sim else 0.3)
    ctl.manual_slew = args.slew if args.slew is not None else 3.0
    loop = threading.Thread(target=ctl.run, daemon=True)
    loop.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(ctl))
    print(f"leg panel on http://{args.host}:{args.port}  ({'SIM' if args.sim else args.backend})")
    print("from the Mac: http://192.168.55.1:%d   Ctrl-C to quit" % args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        ctl.running = False
        loop.join(timeout=1)
        robot.close()
        print("motors braked, bye")


if __name__ == "__main__":
    main()
