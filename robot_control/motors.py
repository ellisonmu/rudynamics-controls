"""
Motor control layer for Unitree GO-M8010-6 actuators.

Everything above the backend works in OUTPUT-SHAFT units (rad, rad/s, N.m, and
output-side kp/kd). The backends speak the motor's native ROTOR-side units, and
Motor does the conversion:

    q_rotor  = q_out  * r          kp_rotor = kp_out / r^2
    dq_rotor = dq_out * r          kd_rotor = kd_out / r^2
    tau_rotor = tau_out / r

The motor firmware runs this PD law itself (rotor side) at high rate:

    tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq)

Backends:
    SdkBackend       official unitree_actuator_sdk (Linux / Jetson)
    PySerialBackend  pure-Python driver in ../macos/go_m8010.py (any OS)
    SimBackend       simulated motors, no hardware needed
"""

import math
import os
import sys
import time
from dataclasses import dataclass, field

GEAR_RATIO = 6.33
MODE_BRAKE, MODE_FOC = 0, 1

# GO-M8010-6 datasheet limits (output side)
PEAK_TORQUE = 23.7      # N.m
MAX_SPEED = 30.0        # rad/s
ROTOR_GAIN_MAX = 25.599 # protocol limit for rotor-side kp/kd

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class SafetyFault(Exception):
    pass


# --------------------------------------------------------------------------
# Backends: send_recv(motor_id, mode, tau, dq, q, kp, kd) -> dict | None
# All values rotor side. Returned dict keys: q, dq, tau, temp, error.
# --------------------------------------------------------------------------

class SdkBackend:
    """Official Unitree SDK (needs the built lib/unitree_actuator_sdk*.so)."""

    def __init__(self, port="/dev/ttyUSB0"):
        sys.path.append(os.path.join(REPO_ROOT, "lib"))
        import unitree_actuator_sdk as u
        self.u = u
        if not os.path.exists(port):
            raise SystemExit(f"{port} not found: is the RS-485 adapter plugged in? "
                             f"(ls /dev/ttyUSB* /dev/ttyACM*)")
        self.serial = u.SerialPort(port)
        self.cmd = u.MotorCmd()
        self.data = u.MotorData()
        self.cmd.motorType = u.MotorType.GO_M8010_6
        self.data.motorType = u.MotorType.GO_M8010_6

    def send_recv(self, motor_id, mode, tau=0.0, dq=0.0, q=0.0, kp=0.0, kd=0.0):
        c = self.cmd
        c.id, c.mode = motor_id, mode
        c.tau, c.dq, c.q, c.kp, c.kd = tau, dq, q, kp, kd
        d = self.data
        # The SDK can leave `correct` set and the previous motor's data in place when nobody
        # replies, so reset it first and check the reply really came from this motor.
        d.correct = False
        d.motor_id = 255
        self.serial.sendRecv(c, d)
        if not d.correct or d.motor_id != motor_id:
            return None
        return {"q": d.q, "dq": d.dq, "tau": d.tau, "temp": d.temp, "error": d.merror}

    def close(self):
        pass


class PySerialBackend:
    """Pure-Python driver (works on macOS and Linux, needs pyserial)."""

    def __init__(self, port):
        sys.path.append(os.path.join(REPO_ROOT, "macos"))
        import go_m8010
        self.dev = go_m8010.GoM8010(port)

    def send_recv(self, motor_id, mode, tau=0.0, dq=0.0, q=0.0, kp=0.0, kd=0.0):
        return self.dev.send_recv(motor_id=motor_id, mode=mode, tau=tau, dq=dq, q=q, kp=kp, kd=kd)

    def close(self):
        self.dev.close()


@dataclass
class _SimMotor:
    q: float = 0.0          # output side
    dq: float = 0.0
    tau: float = 0.0
    temp: float = 30.0
    last_t: float = None


class SimBackend:
    """
    Simulated GO-M8010-6 motors. Runs the same rotor-side PD law as the real
    firmware, with a simple inertia + friction load on the output shaft.

    dt=None integrates using wall-clock time between calls (for live demos);
    a fixed dt makes tests deterministic.
    """

    def __init__(self, inertia=0.02, damping=0.05, dt=None, ids=range(15)):
        self.inertia = inertia      # kg.m^2, output side
        self.damping = damping      # N.m.s/rad, output side
        self.dt = dt
        self.motors = {i: _SimMotor() for i in ids}

    def send_recv(self, motor_id, mode, tau=0.0, dq=0.0, q=0.0, kp=0.0, kd=0.0):
        m = self.motors.get(motor_id)
        if m is None:
            return None  # no motor with that ID on the bus
        now = time.perf_counter()
        if self.dt is not None:
            dt = self.dt
        else:
            dt = 0.0 if m.last_t is None else min(now - m.last_t, 0.02)
        m.last_t = now

        r = GEAR_RATIO
        steps = max(1, int(dt / 0.0005))
        h = dt / steps
        for _ in range(steps):
            if mode == MODE_FOC:
                kp_c = min(max(kp, 0.0), ROTOR_GAIN_MAX)
                kd_c = min(max(kd, 0.0), ROTOR_GAIN_MAX)
                tau_rotor = tau + kp_c * (q - m.q * r) + kd_c * (dq - m.dq * r)
                tau_out = max(-PEAK_TORQUE, min(PEAK_TORQUE, tau_rotor * r))
            else:
                tau_out = 0.0
            acc = (tau_out - self.damping * m.dq) / self.inertia
            m.dq += acc * h
            m.q += m.dq * h
            m.tau = tau_out
        m.temp += (abs(m.tau) * 0.002 - (m.temp - 30.0) * 0.01) * dt
        return {"q": m.q * r, "dq": m.dq * r, "tau": m.tau / r, "temp": int(m.temp), "error": 0}

    def close(self):
        pass


def make_backend(port, kind="sdk"):
    if kind == "sim":
        return SimBackend()
    if kind == "pyserial":
        return PySerialBackend(port)
    return SdkBackend(port)


# --------------------------------------------------------------------------
# Motor: one actuator, output-side units, joint conventions, safety limits
# --------------------------------------------------------------------------

@dataclass
class MotorConfig:
    name: str
    id: int = 0
    port: str = "/dev/ttyUSB0"
    direction: int = 1              # +1 or -1: flips joint positive direction
    offset: float = 0.0             # joint angle (rad) when motor output reads 0
    q_min: float = -math.inf        # joint position limits, rad
    q_max: float = math.inf
    tau_max: float = 5.0            # feed-forward torque limit, N.m (bring-up default)
    dq_max: float = 10.0            # velocity command limit, rad/s
    kp_max: float = 50.0            # output-side gain limits
    kd_max: float = 5.0
    temp_max: int = 75              # deg C; firmware protection trips at 90
    max_missed: int = 20            # consecutive missed replies before fault


@dataclass
class MotorState:
    q: float        # joint position, rad
    dq: float       # joint velocity, rad/s
    tau: float      # joint torque, N.m
    temp: int       # deg C
    error: int      # 0 ok, 1 overheat, 2 overcurrent, 3 overvoltage, 4 encoder


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class Motor:
    def __init__(self, cfg: MotorConfig, backend):
        self.cfg = cfg
        self.backend = backend
        self.state = None
        self.missed = 0

    # --- joint <-> motor-output conversions ---
    def _to_out(self, q_joint):
        return self.cfg.direction * (q_joint - self.cfg.offset)

    def _from_out(self, q_out):
        return self.cfg.direction * q_out + self.cfg.offset

    def command(self, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0):
        """Send a joint-space command, return the new MotorState (or the last one on a miss)."""
        c = self.cfg
        q = _clamp(q, c.q_min, c.q_max)
        dq = _clamp(dq, -c.dq_max, c.dq_max)
        tau = _clamp(tau, -c.tau_max, c.tau_max)
        kp = _clamp(kp, 0.0, c.kp_max)
        kd = _clamp(kd, 0.0, c.kd_max)

        r = GEAR_RATIO
        d = c.direction
        fb = self.backend.send_recv(
            c.id, MODE_FOC,
            tau=d * tau / r,
            dq=d * dq * r,
            q=self._to_out(q) * r,
            kp=kp / r**2,
            kd=kd / r**2,
        )
        return self._update(fb)

    def read(self):
        """Read state with zero gains (motor is limp)."""
        return self.command()

    def brake(self):
        fb = self.backend.send_recv(self.cfg.id, MODE_BRAKE)
        if fb is not None:
            self._update(fb)

    def set_zero_here(self, joint_angle=0.0):
        """Define the current position as `joint_angle` (call after read())."""
        if self.state is None:
            raise SafetyFault(f"{self.cfg.name}: no state yet, call read() first")
        q_out = self._to_out(self.state.q)
        self.cfg.offset = joint_angle - self.cfg.direction * q_out

    def _update(self, fb):
        c = self.cfg
        if fb is None:
            self.missed += 1
            if self.missed >= c.max_missed:
                raise SafetyFault(f"{c.name} (id {c.id}): {self.missed} missed replies")
            return self.state
        self.missed = 0
        r = GEAR_RATIO
        d = c.direction
        self.state = MotorState(
            q=self._from_out(fb["q"] / r),
            dq=d * fb["dq"] / r,
            tau=d * fb["tau"] * r,
            temp=fb["temp"],
            error=fb["error"],
        )
        if self.state.error:
            raise SafetyFault(f"{c.name}: motor error code {self.state.error}")
        if self.state.temp >= c.temp_max:
            raise SafetyFault(f"{c.name}: temperature {self.state.temp}C >= {c.temp_max}C")
        return self.state


# --------------------------------------------------------------------------
# Robot: several motors, shared serial ports, fixed-rate loop, safe shutdown
# --------------------------------------------------------------------------

class Robot:
    def __init__(self, configs, backend_kind="sdk"):
        self.backends = {}
        self.motors = {}
        for cfg in configs:
            if cfg.port not in self.backends:
                self.backends[cfg.port] = make_backend(cfg.port, backend_kind)
            self.motors[cfg.name] = Motor(cfg, self.backends[cfg.port])

    def __getitem__(self, name):
        return self.motors[name]

    def __iter__(self):
        return iter(self.motors.values())

    def read_all(self):
        return {m.cfg.name: m.read() for m in self}

    def brake_all(self):
        for m in self:
            try:
                m.brake()
            except Exception:
                pass

    def close(self):
        self.brake_all()
        for b in self.backends.values():
            b.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


class RateLoop:
    """Sleep-based fixed-rate loop. Usage: for t in RateLoop(500): ..."""

    def __init__(self, hz, duration=None):
        self.period = 1.0 / hz
        self.duration = duration
        self.overruns = 0

    def __iter__(self):
        start = time.perf_counter()
        next_t = start
        while self.duration is None or next_t - start < self.duration:
            yield next_t - start
            next_t += self.period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                self.overruns += 1
                next_t = time.perf_counter()
