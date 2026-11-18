"""
Hardware-free tests. Run: python3 test_motors.py   (or pytest test_motors.py)
"""

import math
import os
import sys

from motors import GEAR_RATIO, Motor, MotorConfig, SafetyFault, SimBackend

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "macos"))
import go_m8010  # noqa: E402


def make_motor(**kw):
    sim = SimBackend(dt=0.002)
    return Motor(MotorConfig(name="m", **kw), sim), sim


def run(motor, steps, **cmd):
    s = None
    for _ in range(steps):
        s = motor.command(**cmd)
    return s


def test_packets_match_official_sdk():
    # Bytes captured from the official SDK (lib/unitree_actuator_sdk) on the Jetson.
    sdk = bytes.fromhex("feee108001adf91d3d000080021900e4a2")
    assert go_m8010.pack_cmd(0, 1, 1.5, -6.28 * 6.33, 3.0, 0.5, 0.02) == sdk
    sdk = bytes.fromhex("feee13c0ff97015b01ffff000a80002556")
    assert go_m8010.pack_cmd(3, 1, -0.25, 10.0, -12.5, 2.0, 0.1) == sdk


def test_hold_position_converges():
    m, _ = make_motor()
    s = run(m, 1500, q=0.8, kp=20, kd=1.0)
    assert abs(s.q - 0.8) < 0.01 and abs(s.dq) < 0.05


def test_velocity_control():
    m, _ = make_motor()
    s = run(m, 1500, dq=2.0, kd=1.0)
    assert abs(s.dq - 2.0) < 0.1


def test_rotor_side_conversion():
    sent = {}

    class Recorder:
        def send_recv(self, motor_id, mode, **cmd):
            sent.update(cmd)
            return {"q": 2.0 * GEAR_RATIO, "dq": 0.0, "tau": 1.0, "temp": 30, "error": 0}

    m = Motor(MotorConfig(name="m"), Recorder())
    s = m.command(q=1.0, dq=2.0, tau=3.0, kp=40.0, kd=4.0)
    r = GEAR_RATIO
    assert math.isclose(sent["q"], 1.0 * r) and math.isclose(sent["dq"], 2.0 * r)
    assert math.isclose(sent["tau"], 3.0 / r)
    assert math.isclose(sent["kp"], 40.0 / r**2) and math.isclose(sent["kd"], 4.0 / r**2)
    assert math.isclose(s.q, 2.0) and math.isclose(s.tau, r)


def test_direction_and_offset():
    m, sim = make_motor(direction=-1, offset=0.3)
    s = run(m, 1500, q=0.5, kp=20, kd=1.0)
    assert abs(s.q - 0.5) < 0.01
    assert abs(sim.motors[0].q - (-(0.5 - 0.3))) < 0.01  # physical output shaft


def test_set_zero_here():
    m, _ = make_motor()
    run(m, 1000, q=1.2, kp=20, kd=1.0)
    m.set_zero_here(0.0)
    assert abs(m.read().q) < 0.01


def test_limits_clamp_commands():
    m, _ = make_motor(q_min=-0.5, q_max=0.5, dq_max=1.0)
    assert abs(run(m, 2000, q=3.0, kp=20, kd=1.0).q - 0.5) < 0.01
    assert run(m, 1500, dq=50.0, kd=1.0).dq < 1.05


def test_missed_replies_fault():
    m = Motor(MotorConfig(name="ghost", id=7, max_missed=5), SimBackend(dt=0.002, ids=[0]))
    try:
        for _ in range(10):
            m.read()
    except SafetyFault:
        return
    raise AssertionError("expected SafetyFault")


def test_overtemp_fault():
    m, sim = make_motor(temp_max=60)
    sim.motors[0].temp = 70
    try:
        m.read()
    except SafetyFault:
        return
    raise AssertionError("expected SafetyFault")


if __name__ == "__main__":
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_")]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS {name}")
        except Exception as e:
            failed += 1
            print(f"FAIL {name}: {e!r}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
