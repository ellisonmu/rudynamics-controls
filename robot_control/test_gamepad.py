"""
Hardware-free tests for gamepad.py: raw joystick events -> Teleop -> simulated leg.
Run: python3 test_gamepad.py   (or pytest test_gamepad.py)
"""

import sys

import leg_server
from gamepad import AXES, BUTTONS, JS_AXIS, JS_BUTTON, JS_EVENT, JS_INIT, Pad, Teleop, stick
from test_leg import DT, make_controller, run

BTN = {v: k for k, v in BUTTONS.items()}
AX = {v: k for k, v in AXES.items()}
PAD_DT = 0.02           # teleop runs at 50 Hz


def ev(kind, number, value, init=False):
    return JS_EVENT.pack(0, value, kind | (JS_INIT if init else 0), number)


class FakePad(Pad):
    def __init__(self):
        super().__init__("/dev/null")
        self.fd = -1                    # pretend connected
        self.queue = b""

    def press(self, name):
        self.queue += ev(JS_BUTTON, BTN[name], 1)

    def release(self, name):
        self.queue += ev(JS_BUTTON, BTN[name], 0)

    def move(self, axis, value):
        self.queue += ev(JS_AXIS, AX[axis], int(value * 32767))

    def poll(self):
        self.pressed = set()
        self.feed(self.queue)
        self.queue = b""


def setup(enabled=True):
    ctl, clock = make_controller()
    run(ctl, clock, 0.05)
    api = lambda path, body=None: leg_server.dispatch(ctl, path, body or {})
    pad, tele = FakePad(), Teleop(api)
    if enabled:
        ctl.enable()
    return ctl, clock, pad, tele


def drive(ctl, clock, pad, tele, seconds):
    """Run teleop at 50 Hz and the controller at 500 Hz, like on the robot."""
    for _ in range(int(seconds / PAD_DT)):
        pad.poll()
        tele.step(pad, ctl.snapshot(), PAD_DT)
        ctl.touch()
        run(ctl, clock, PAD_DT)


def foot(ctl):
    return ctl.snapshot()["measured_pts"]["foot"]


def test_event_parsing():
    pad = Pad()
    pad.feed(ev(JS_BUTTON, BTN["l1"], 1, init=True) + ev(JS_AXIS, AX["ly"], -32767)
             + ev(JS_BUTTON, BTN["cross"], 1) + b"\x01\x02")      # trailing partial event ignored
    assert pad.held("l1") and pad.held("cross")
    assert pad.pressed == {"cross"}, "init events are state, not presses"
    assert abs(pad.axis("ly") + 1.0) < 1e-3


def test_deadzone():
    assert stick(0.05, 0.12) == 0.0
    assert abs(stick(1.0, 0.12) - 1.0) < 1e-9 and abs(stick(-1.0, 0.12) + 1.0) < 1e-9


def test_sticks_need_l1():
    ctl, clock, pad, tele = setup()
    start = foot(ctl)
    pad.move("ly", -1.0)                  # left stick up, no L1
    drive(ctl, clock, pad, tele, 1.0)
    assert abs(foot(ctl)[0] - start[0]) < 0.005, "foot moved without the dead-man"


def test_left_stick_moves_foot_forward():
    ctl, clock, pad, tele = setup()
    start = foot(ctl)
    pad.press("l1")
    pad.move("ly", -1.0)
    drive(ctl, clock, pad, tele, 0.5)
    pad.move("ly", 0.0)
    drive(ctl, clock, pad, tele, 1.0)     # let it settle
    moved = foot(ctl)[0] - start[0]
    assert 0.04 < moved < 0.1, f"foot moved {moved:.3f} m forward (expected ~0.075)"


def test_right_stick_moves_foot_up():
    ctl, clock, pad, tele = setup()
    start = foot(ctl)
    pad.press("l1")
    pad.move("ry", -1.0)
    drive(ctl, clock, pad, tele, 0.5)
    pad.move("ry", 0.0)
    drive(ctl, clock, pad, tele, 1.0)
    assert foot(ctl)[2] - start[2] > 0.04


def test_foot_target_does_not_wind_up_past_reach():
    ctl, clock, pad, tele = setup()
    pad.press("l1")
    pad.move("ry", 1.0)                   # push down for a long time: leg hits full reach
    drive(ctl, clock, pad, tele, 4.0)
    pad.move("ry", -1.0)                  # reversing should respond immediately
    before = foot(ctl)[2]
    drive(ctl, clock, pad, tele, 0.5)
    assert foot(ctl)[2] - before > 0.03, "foot target wound up past the reach limit"


def test_l1_triangle_starts_crawl_and_release_holds():
    ctl, clock, pad, tele = setup()
    pad.press("l1")
    pad.press("triangle")
    drive(ctl, clock, pad, tele, 1.0)
    assert ctl.mode == "motion" and ctl.motion.key == "crawl"
    pad.release("l1")
    drive(ctl, clock, pad, tele, 0.1)
    assert ctl.mode == "manual", "releasing L1 should stop the move"


def test_move_buttons_need_l1():
    ctl, clock, pad, tele = setup()
    pad.press("triangle")
    drive(ctl, clock, pad, tele, 0.2)
    assert ctl.mode == "manual"


def test_circle_estops_without_l1():
    ctl, clock, pad, tele = setup()
    pad.press("l1")
    pad.press("cross")
    drive(ctl, clock, pad, tele, 0.2)
    pad.release("l1")
    pad.press("circle")
    drive(ctl, clock, pad, tele, 0.1)
    assert ctl.mode == "damping"


def test_options_enables_only_with_l1():
    ctl, clock, pad, tele = setup(enabled=False)
    pad.press("options")
    drive(ctl, clock, pad, tele, 0.1)
    assert ctl.mode == "disabled"
    pad.release("options")
    pad.press("l1")
    pad.press("options")
    drive(ctl, clock, pad, tele, 0.1)
    assert ctl.mode == "manual"


def test_dpad_changes_speed():
    ctl, clock, pad, tele = setup()
    pad.press("l1")
    pad.move("dpad_y", -1.0)              # up
    drive(ctl, clock, pad, tele, 0.2)     # held: counts once
    pad.move("dpad_y", 0.0)
    drive(ctl, clock, pad, tele, 0.1)
    pad.move("dpad_y", -1.0)
    drive(ctl, clock, pad, tele, 0.1)
    assert tele.speed == 1.5
    pad.press("square")
    drive(ctl, clock, pad, tele, 0.1)
    assert ctl.motion.key == "step" and ctl.motion.speed == 1.5


def test_disconnect_stops_motion():
    ctl, clock, pad, tele = setup()
    pad.press("l1")
    pad.press("triangle")
    drive(ctl, clock, pad, tele, 0.5)
    assert ctl.mode == "motion"
    pad.fd = None                         # Bluetooth dropped
    drive(ctl, clock, pad, tele, 0.1)
    assert ctl.mode == "manual"


def test_errors_do_not_crash():
    ctl, clock, pad, tele = setup(enabled=False)
    pad.press("l1")
    pad.move("ly", -1.0)
    pad.press("r1")
    drive(ctl, clock, pad, tele, 0.2)     # disabled: nothing to do, no exception
    assert ctl.mode == "disabled"


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
