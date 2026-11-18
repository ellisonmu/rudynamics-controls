"""
Hardware-free tests for leg kinematics, motions and the panel controller.
Run: python3 test_leg.py   (or pytest test_leg.py)
"""

import math
import random
import sys

import leg_server
from leg import MOTIONS, LegGeometry, fk, ik
from motors import MotorConfig, Robot, SafetyFault
from robot_config import LEG, MOTORS

DT = 0.002


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make_controller(ids=None, start=(0.0, 0.8, -1.6)):
    ids = [c.id for c in MOTORS] if ids is None else ids
    configs = [MotorConfig(**{**c.__dict__}) for c in MOTORS]  # fresh copies
    robot = Robot(configs, "sim")
    backend = next(iter(robot.backends.values()))
    backend.dt = DT
    backend.motors = {i: m for i, m in backend.motors.items() if i in ids}
    for m, q in zip(robot, start):
        if m.cfg.id in backend.motors:
            backend.motors[m.cfg.id].q = q
    clock = FakeClock()
    ctl = leg_server.LegController(robot, LEG, rate=1 / DT, calibrated=True, clock=clock)
    return ctl, clock


def run(ctl, clock, seconds, touch=True):
    for _ in range(int(seconds / DT)):
        clock.t += DT
        if touch:
            ctl.touch()
        ctl.tick(DT)


def test_fk_ik_roundtrip():
    g = LegGeometry(0.2, 0.2)
    rng = random.Random(1)
    for _ in range(500):
        q = [rng.uniform(-0.6, 0.6), rng.uniform(-1.2, 1.8), rng.uniform(-2.6, -0.15)]
        _, foot = fk(q, g)
        q2 = ik(*foot, g)
        _, foot2 = fk(q2, g)
        assert all(abs(a - b) < 1e-6 for a, b in zip(foot, foot2)), (q, q2)


def test_ik_unreachable_is_clamped():
    g = LegGeometry(0.2, 0.2)
    _, foot = fk(ik(0.0, 0.0, -1.0, g), g)
    assert abs(math.hypot(*foot) - g.reach) < 1e-3


def test_motions_stay_within_joint_limits():
    start = [0.0, 0.8, -1.6]
    for key, cls in MOTIONS.items():
        for extreme in ("lo", "default", "hi"):
            params = {p.key: getattr(p, extreme) for p in cls.params}
            m = cls(start, LEG, **params)
            for i in range(0, 8000, 7):
                q, _ = m.joints(i * 0.001)
                for qi, cfg in zip(q, MOTORS):
                    assert cfg.q_min - 1e-6 <= qi <= cfg.q_max + 1e-6, (key, extreme, cfg.name, qi)


def test_one_shot_motions_finish():
    for key, cls in MOTIONS.items():
        if cls.cyclic:
            continue
        m = cls([0.0, 0.8, -1.6], LEG)
        assert m.joints(30.0)[1], key


def test_motion_blends_from_start_pose():
    start = [0.1, 0.5, -1.0]
    for cls in MOTIONS.values():
        q, _ = cls(start, LEG).joints(0.0)
        assert all(abs(a - b) < 1e-9 for a, b in zip(q, start)), cls.key


def test_enable_requires_calibration():
    ctl, clock = make_controller()
    ctl.calibrated = False
    run(ctl, clock, 0.05)
    try:
        ctl.enable()
    except ValueError:
        return
    raise AssertionError("enable should require calibration")


def test_manual_target_reached():
    ctl, clock = make_controller()
    run(ctl, clock, 0.05)
    ctl.enable()
    target = [0.2, 0.5, -1.2]
    ctl.set_manual(target)
    run(ctl, clock, 2.0)
    q = [s.q for s in ctl.states]
    assert all(abs(a - b) < 0.02 for a, b in zip(q, target)), q


def test_manual_targets_are_clamped_and_slewed():
    ctl, clock = make_controller()
    run(ctl, clock, 0.05)
    ctl.enable()
    before = list(ctl.cmd)
    ctl.set_manual([5.0, 5.0, 5.0])
    assert ctl.target == [m.cfg.q_max for m in ctl.motors]
    run(ctl, clock, 0.1)
    assert all(abs(a - b) <= ctl.manual_slew * 0.1 + 1e-6 for a, b in zip(ctl.cmd, before))


def test_crawl_tracks_foot_path():
    ctl, clock = make_controller()
    run(ctl, clock, 0.05)
    ctl.enable()
    ctl.start_motion("crawl", {})
    run(ctl, clock, 2.0)          # blend in
    xs, err = [], 0.0
    for _ in range(int(1.6 / DT)):
        run(ctl, clock, DT)
        _, foot = fk([s.q for s in ctl.states], LEG)
        _, want = fk(ctl.cmd, LEG)
        xs.append(foot[0])
        err = max(err, math.dist(foot, want))
    assert max(xs) - min(xs) > 0.1, "foot should sweep a stride"
    assert err < 0.02, f"tracking error {err:.3f} m"


def test_estop_and_disable():
    ctl, clock = make_controller()
    run(ctl, clock, 0.05)
    ctl.enable()
    ctl.start_motion("circle", {})
    run(ctl, clock, 0.5)
    ctl.estop()
    run(ctl, clock, 0.1)
    assert ctl.mode == "damping" and ctl.motion is None
    try:
        ctl.start_motion("crawl", {})
        raise AssertionError("motion should need enable after E-STOP")
    except ValueError:
        pass
    ctl.disable()
    assert ctl.mode == "disabled"


def test_lost_heartbeat_holds_position():
    ctl, clock = make_controller()
    run(ctl, clock, 0.05)
    ctl.enable()
    ctl.start_motion("crawl", {})
    run(ctl, clock, 0.5)
    run(ctl, clock, leg_server.HEARTBEAT_TIMEOUT + 0.2, touch=False)
    assert ctl.mode == "manual" and ctl.motion is None
    assert "lost contact" in ctl.message


def test_missing_motor_faults():
    ctl, clock = make_controller(ids=[c.id for c in MOTORS[:2]])   # knee not on the bus
    run(ctl, clock, 0.2)
    assert ctl.mode == "fault" and "knee" in ctl.message, ctl.message


def test_calibrate_sets_zero():
    ctl, clock = make_controller(start=(0.3, 0.4, -0.5))
    ctl.calibrated = False
    run(ctl, clock, 0.05)
    ctl.calibrate()
    run(ctl, clock, 0.05)
    assert all(abs(s.q) < 1e-6 for s in ctl.states)
    assert ctl.calibrated


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
