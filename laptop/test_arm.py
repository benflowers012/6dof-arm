"""Plain-assert checks for arm_core. Run: python test_arm.py"""
from arm_core import *

LIM_LINE = "LIMITS " + "20 160 90 " * 4 + "0 180 90 20 160 90"
LIM = parse_limits(LIM_LINE)


def test_parse_limits():
    assert LIM[0] == (20, 160, 90) and LIM[4] == (0, 180, 90) and len(LIM) == 6
    for bad in ["READY", "LIMITS 1 2 3", "LIMITS " + "x " * 18]:
        try:
            parse_limits(bad)
            assert False, bad
        except ValueError:
            pass


def test_clamp_and_format():
    assert clamp_pose([0, 200, 90, 90, -5, 90], LIM) == [20, 160, 90, 90, 0, 90]
    assert format_pose([90.4, 89.6, 20, 160, 0, 180]) == "P 90 90 20 160 0 180\n"
    assert home_pose(LIM) == [90] * 6


def test_step_teleop():
    p = [90.0] * 6
    no_inv = [False] * 6
    assert step_teleop(p, [1, 0, 0, 0, 0, 0], 0.5, LIM, no_inv)[0] == 120
    assert step_teleop(p, [0.1, 0, 0, 0, 0, 0], 0.5, LIM, no_inv) == p  # deadzone
    assert step_teleop(p, [1, 0, 0, 0, 0, 0], 0.5, LIM, [True] + [False] * 5)[0] == 60
    assert step_teleop(p, [1, 0, 0, 0, 0, 0], 10, LIM, no_inv)[0] == 160  # clamped
    assert any_active([0, 0.2, 0, 0, 0, 0]) and not any_active([0.1] * 6)


def test_player():
    frames = [{"t": 0, "pose": [0] * 6}, {"t": 2, "pose": [100] * 6}]
    pl = Player(frames)
    assert pl.pose_at(1) == [50] * 6
    assert pl.pose_at(0.5) == [15.625] * 6  # smoothstep(0.25)
    assert pl.pose_at(3) == [100] * 6 and pl.done(2.0) and not pl.done(1.9)
    lead = Player(frames, start_pose=[50] * 6)
    assert lead.pose_at(0) == [50] * 6 and lead.pose_at(1.5) == [0] * 6
    assert lead.done(3.5) and not lead.done(3.4)
    one = Player([{"t": 0, "pose": [10] * 6}])
    assert one.pose_at(5) == [10] * 6 and one.done(0)


def test_recorder():
    r = Recorder()
    r.start(0.0)
    pose = [90] * 6
    for t in (0.0, 0.02, 0.06, 0.12):
        r.sample(t, pose)
    pose[0] = 1  # must not alias stored frames
    assert [round(f["t"], 3) for f in r.frames] == [0, 0.06, 0.12]
    assert r.frames[0]["pose"][0] == 90


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("all checks passed")
