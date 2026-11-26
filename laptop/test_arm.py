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


def test_files():
    import json, tempfile
    from datetime import datetime
    from pathlib import Path

    d = Path(tempfile.mkdtemp())
    assert latest_recording(d / "nope") is None
    assert save_recording([{"t": 0, "pose": [90] * 6}], d) is None  # too short
    frames = [{"t": 0, "pose": [90] * 6}, {"t": 1, "pose": [100] * 6}]
    p1 = save_recording(frames, d, datetime(2026, 11, 26, 9, 0, 0))
    p2 = save_recording(frames, d, datetime(2026, 11, 26, 9, 0, 5))
    assert p1.name == "rec-20261126-090000.json" and latest_recording(d) == p2
    assert load_program(p2) == frames
    for bad in [{"frames": []}, {"name": "x"}, {"frames": [{"t": 0, "pose": [1, 2]}]}]:
        f = d / "bad.json"
        f.write_text(json.dumps(bad))
        try:
            load_program(f)
            assert False, bad
        except ValueError as e:
            assert "bad.json" in str(e)


def test_flip_file_is_valid():
    from pathlib import Path

    frames = load_program(Path(__file__).parent / "programs" / "flip.json")
    assert len(frames) >= 6 and frames[0]["t"] == 0
    assert all(a["t"] < b["t"] for a, b in zip(frames, frames[1:]))


def test_parse_pose():
    assert parse_pose("POSE 90 150 20 160 0 180") == [90, 150, 20, 160, 0, 180]
    for bad in ["POSE 1 2", "LIMITS 1 2 3 4 5 6", "POSE a b c d e f"]:
        try:
            parse_pose(bad)
            assert False, bad
        except ValueError:
            pass


def test_program_times_must_increase():
    import json, tempfile
    from pathlib import Path

    f = Path(tempfile.mkdtemp()) / "flip.json"
    f.write_text(json.dumps({"frames": [{"t": 0, "pose": [90] * 6},
                                        {"t": 6.0, "pose": [90] * 6},
                                        {"t": 5.5, "pose": [90] * 6}]}))
    try:
        load_program(f)
        assert False, "out-of-order t accepted"
    except ValueError as e:
        assert "flip.json" in str(e)


def test_unplugged_usb_exits_cleanly():
    import serial
    import arm

    class DeadPort:
        def write(self, data):
            raise serial.SerialException("device disconnected")

    link = object.__new__(arm.Link)
    link.ser = DeadPort()
    try:
        link.send([90] * 6)
        assert False, "no exit"
    except SystemExit as e:
        assert "USB" in str(e.code)


def test_connect_starts_from_current_pose_without_reset():
    import arm

    class FakeUno:  # an Uno that is NOT reset: silent until asked "?"
        def __init__(self):
            self.out, self.dtr_at_open = [], None

        def open(self):
            self.dtr_at_open = self.dtr

        def write(self, data):
            if data == b"?\n":
                self.out += [LIM_LINE.encode() + b"\n", b"POSE 150 40 90 90 0 90\n"]

        def readline(self):
            return self.out.pop(0) if self.out else b""

    real, arm.serial.Serial = arm.serial.Serial, FakeUno
    try:
        link = arm.Link("COM_TEST")
    finally:
        arm.serial.Serial = real
    assert link.ser.dtr_at_open is False  # opening must not reset the Uno
    assert link.limits == LIM and link.start_pose == [150, 40, 90, 90, 0, 90]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("all checks passed")
