# Robot Arm Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Uno firmware plus a laptop Python program that let an Xbox controller drive a 6-servo arm, record and replay moves, and run a one-button "flip" program.

**Architecture:** The laptop (`arm.py`) reads the controller, records and replays, and runs keyframe programs. 50 times a second it sends a `P a0..a5` line over USB serial. The Uno (`arm.ino`) clamps each target to its joint limits, moves the servos toward the targets at a capped speed through a PCA9685 driver over I2C, and freezes if the laptop goes silent. All the testable logic lives in `arm_core.py`, which has no hardware imports.

**Tech Stack:** Arduino C++ (Adafruit PWM Servo Driver Library), Python 3.14, `pygame-ce` 2.5.x, `pyserial`.

**Spec:** `docs/superpowers/specs/2026-11-26-arm-control-design.md` (wiring: `docs/wiring.html`)

## Global Constraints

- Joint order everywhere: 0 Base, 1 Shoulder, 2 Elbow, 3 Wrist, 4 Claw rotate, 5 Claw grip. These are also PCA9685 channels 0–5.
- Serial is 115200 baud. Laptop → Uno: `P a0 a1 a2 a3 a4 a5\n`, integer degrees. Uno → laptop on boot: `READY`, then `LIMITS min0 max0 home0 … min5 max5 home5`.
- The joint limits are defined only in the firmware. The laptop learns them from the `LIMITS` line.
- Starting limits: 20–160°, home 90°. Claw rotate: 0–180°, home 90°. Pulse range 500–2500 µs. `OSC_HZ` 27000000. `MAX_DEG_PER_S` 90. Watchdog 500 ms.
- Laptop loop 50 Hz. Stick speed up to 60°/s, deadzone 0.15. Recording at 20 Hz. Every playback starts with a 1.5 s ease from the current pose (lead-in). Easing is smoothstep.
- Recordings go to `laptop/recordings/rec-YYYYMMDD-HHMMSS.json`. Programs go in `laptop/programs/`.
- Program/recording format: `{"name": str, "frames": [{"t": float, "pose": [6 numbers]}]}`.
- Tests are plain `assert`s in `laptop/test_arm.py`, run with `python test_arm.py`. No test framework.
- **Plan's call, deviating from the spec:** there is no `home.json`. "Go home" builds a one-frame program from the `LIMITS` home values at runtime, so home is still defined in one place.

## Review Focus

1. **Uno reset when the port opens.** Opening serial reboots the Uno. The laptop must not send `P` lines until it has seen `READY` and parsed `LIMITS`. If they don't arrive within 5 s, it exits with a clear message. (Task 4, manual check.)
2. **Empty or tiny recordings, and pressing Y with none saved.** Pressing X twice fast gives 0–1 frames, which must be discarded with a message rather than saved. Y with no recordings prints "no recordings yet" and doesn't crash. (Task 2 tests.)
3. **Bad program files.** A pose with the wrong number of values, or a missing `frames` key, must raise `ValueError` naming the file. Out-of-range angles get clamped rather than rejected. (Task 2 tests.)
4. **Garbage on the serial line.** A partial line, extra spaces, or noise at boot must be ignored by the Uno, and must not move anything or reset the watchdog. (Task 3, manual check with Serial Monitor.)
5. **Controller unplugged mid-run.** `arm.py` stops sending and exits with a message. The Uno watchdog then freezes the arm in place. (Task 4, manual check.)

---

## File Structure

```
firmware/arm/arm.ino        # Uno: serial parse, clamp, rate-limit, watchdog, PCA9685 output
laptop/arm_core.py          # pure logic: limits, clamp, teleop step, Player, Recorder, file I/O
laptop/test_arm.py          # assert-based checks for arm_core
laptop/arm.py               # serial link, controller loop, button actions
laptop/programs/flip.json   # placeholder flip keyframes (tuned on hardware later)
laptop/requirements.txt     # pygame-ce, pyserial
README.md                   # setup, controller map, tuning, bring-up checklist
```

---

### Task 1: Core motion logic (`arm_core.py`)

**Files:**
- Create: `laptop/arm_core.py`
- Create: `laptop/requirements.txt` (`pygame-ce>=2.5`, `pyserial>=3.5`)
- Test: `laptop/test_arm.py`

**Interfaces:**
- Produces:
  - `Limits = list[tuple[int, int, int]]`: six `(min, max, home)`.
  - `parse_limits(line: str) -> Limits`: raises `ValueError` unless the line is `LIMITS` followed by exactly 18 ints.
  - `clamp_pose(pose: list[float], limits: Limits) -> list[float]`
  - `format_pose(pose: list[float]) -> str`: `"P 90 90 90 90 90 90\n"`, values rounded to int.
  - `home_pose(limits: Limits) -> list[float]`
  - `step_teleop(pose, speeds: list[float], dt: float, limits: Limits, invert: list[bool], max_speed=60.0, deadzone=0.15) -> list[float]`. `speeds` holds six values in [-1, 1]. Values inside the deadzone count as 0. Inverted joints flip sign. The result is clamped.
  - `any_active(speeds: list[float], deadzone=0.15) -> bool`
  - `smoothstep(x: float) -> float`: `3x² − 2x³`, with x clamped to [0, 1].
  - `class Player(frames: list[dict], start_pose: list[float] | None = None, lead_in=1.5)`, with `.pose_at(t: float) -> list[float]` and `.done(t: float) -> bool`. When `start_pose` is given, every frame's `t` shifts by `lead_in` and `{t: 0, pose: start_pose}` goes in front.
  - `class Recorder(interval=0.05)`, with `.start(t: float)`, `.sample(t: float, pose: list[float])`, and `.frames: list[dict]`. Each frame's `t` is relative to `start`. A sample is added only if at least `interval` has passed since the last one. The first sample is always added. Poses are copied.

- [ ] **Step 1: Write the failing tests** in `laptop/test_arm.py`, as functions called from `if __name__ == "__main__":`, which prints `all checks passed`.

```python
from arm_core import *

LIM_LINE = "LIMITS " + "20 160 90 " * 4 + "0 180 90 20 160 90"
LIM = parse_limits(LIM_LINE)

def test_parse_limits():
    assert LIM[0] == (20, 160, 90) and LIM[4] == (0, 180, 90) and len(LIM) == 6
    for bad in ["READY", "LIMITS 1 2 3", "LIMITS " + "x " * 18]:
        try: parse_limits(bad); assert False, bad
        except ValueError: pass

def test_clamp_and_format():
    assert clamp_pose([0, 200, 90, 90, -5, 90], LIM) == [20, 160, 90, 90, 0, 90]
    assert format_pose([90.4, 89.6, 20, 160, 0, 180]) == "P 90 90 20 160 0 180\n"
    assert home_pose(LIM) == [90] * 6

def test_step_teleop():
    p = [90.0] * 6
    no_inv = [False] * 6
    assert step_teleop(p, [1, 0, 0, 0, 0, 0], 0.5, LIM, no_inv)[0] == 120
    assert step_teleop(p, [0.1, 0, 0, 0, 0, 0], 0.5, LIM, no_inv) == p          # deadzone
    assert step_teleop(p, [1, 0, 0, 0, 0, 0], 0.5, LIM, [True] + [False] * 5)[0] == 60
    assert step_teleop(p, [1, 0, 0, 0, 0, 0], 10, LIM, no_inv)[0] == 160         # clamped
    assert any_active([0, 0.2, 0, 0, 0, 0]) and not any_active([0.1] * 6)

def test_player():
    frames = [{"t": 0, "pose": [0] * 6}, {"t": 2, "pose": [100] * 6}]
    pl = Player(frames)
    assert pl.pose_at(1) == [50] * 6
    assert pl.pose_at(0.5) == [15.625] * 6        # smoothstep(0.25)
    assert pl.pose_at(3) == [100] * 6 and pl.done(2.0) and not pl.done(1.9)
    lead = Player(frames, start_pose=[50] * 6)
    assert lead.pose_at(0) == [50] * 6 and lead.pose_at(1.5) == [0] * 6
    assert lead.done(3.5) and not lead.done(3.4)
    one = Player([{"t": 0, "pose": [10] * 6}])
    assert one.pose_at(5) == [10] * 6 and one.done(0)

def test_recorder():
    r = Recorder(); r.start(0.0)
    pose = [90] * 6
    for t in (0.0, 0.02, 0.06, 0.12):
        r.sample(t, pose)
    pose[0] = 1                                   # must not alias stored frames
    assert [round(f["t"], 3) for f in r.frames] == [0, 0.06, 0.12]
    assert r.frames[0]["pose"][0] == 90
```

- [ ] **Step 2: Run them and watch them fail**

Run: `cd laptop && python test_arm.py`
Expected: `ModuleNotFoundError: No module named 'arm_core'`

- [ ] **Step 3: Implement everything listed under Produces** in `laptop/arm_core.py`, using the standard library only. `Player.pose_at` finds the surrounding pair of frames and blends them with `smoothstep` on the local fraction. Before the first frame it returns the first pose, and after the last frame it returns the last pose.

- [ ] **Step 4: Run them and watch them pass**

Run: `cd laptop && python test_arm.py`
Expected: `all checks passed`

- [ ] **Step 5: Commit**

```bash
git add laptop/arm_core.py laptop/test_arm.py laptop/requirements.txt
git commit -m "feat: core arm motion logic with checks"
```

---

### Task 2: Program and recording files

**Files:**
- Modify: `laptop/arm_core.py`
- Create: `laptop/programs/flip.json`
- Test: `laptop/test_arm.py`

**Interfaces:**
- Consumes: `Recorder.frames` from Task 1.
- Produces:
  - `load_program(path: Path) -> list[dict]`: returns the frames. Raises `ValueError` with the file name in the message if `frames` is missing or empty, or if any pose isn't six numbers.
  - `save_recording(frames: list[dict], folder: Path, now: datetime | None = None) -> Path | None`. Returns `None` and writes nothing when there are fewer than 2 frames. Otherwise it creates `folder` if needed and writes `rec-YYYYMMDD-HHMMSS.json` with `name` set to the file stem.
  - `latest_recording(folder: Path) -> Path | None`: the newest `rec-*.json` by name, or `None` if the folder is missing or empty.

- [ ] **Step 1: Write the failing tests** (call them from `__main__`):

```python
import json, tempfile
from datetime import datetime
from pathlib import Path

def test_files():
    d = Path(tempfile.mkdtemp())
    assert latest_recording(d / "nope") is None
    assert save_recording([{"t": 0, "pose": [90] * 6}], d) is None   # too short
    frames = [{"t": 0, "pose": [90] * 6}, {"t": 1, "pose": [100] * 6}]
    p1 = save_recording(frames, d, datetime(2026, 11, 26, 9, 0, 0))
    p2 = save_recording(frames, d, datetime(2026, 11, 26, 9, 0, 5))
    assert p1.name == "rec-20261126-090000.json" and latest_recording(d) == p2
    assert load_program(p2) == frames
    for bad in [{"frames": []}, {"name": "x"}, {"frames": [{"t": 0, "pose": [1, 2]}]}]:
        f = d / "bad.json"; f.write_text(json.dumps(bad))
        try: load_program(f); assert False, bad
        except ValueError as e: assert "bad.json" in str(e)

def test_flip_file_is_valid():
    frames = load_program(Path(__file__).parent / "programs" / "flip.json")
    assert len(frames) >= 6 and frames[0]["t"] == 0
    assert all(a["t"] < b["t"] for a, b in zip(frames, frames[1:]))
```

- [ ] **Step 2: Run them and watch them fail**

Run: `cd laptop && python test_arm.py`
Expected: `NameError: name 'latest_recording' is not defined`

- [ ] **Step 3: Implement the three functions.** Then write `programs/flip.json` with placeholder poses. These get tuned on the arm later.

```json
{"name": "flip", "frames": [
  {"t": 0.0, "pose": [90, 90, 90, 90, 90, 90]},
  {"t": 1.5, "pose": [90, 60, 120, 60, 90, 90]},
  {"t": 2.5, "pose": [90, 60, 120, 60, 90, 40]},
  {"t": 3.5, "pose": [90, 100, 90, 90, 90, 40]},
  {"t": 6.0, "pose": [90, 140, 60, 120, 0, 40]},
  {"t": 7.5, "pose": [90, 150, 50, 130, 0, 40]},
  {"t": 8.5, "pose": [90, 150, 50, 130, 0, 90]},
  {"t": 10.0, "pose": [90, 90, 90, 90, 90, 90]}
]}
```

The sequence: home → open over the pickup → close (grip 40) → lift → swing back with the claw rotating 90→0 (half a turn) → lower behind → open → home.

- [ ] **Step 4: Run them and watch them pass**

Run: `cd laptop && python test_arm.py`
Expected: `all checks passed`

- [ ] **Step 5: Commit**

```bash
git add laptop/arm_core.py laptop/test_arm.py laptop/programs/flip.json
git commit -m "feat: program/recording files and placeholder flip"
```

---

### Task 3: Uno firmware

**Files:**
- Create: `firmware/arm/arm.ino`

**Interfaces:**
- Produces: the serial protocol in Global Constraints. Nothing else depends on the firmware's internals.

Tooling: the Arduino IDE ships a CLI at
`"$LOCALAPPDATA/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"` (call it `$CLI` below). The AVR core and the library aren't installed yet.

- [ ] **Step 1: Install the core and library**

Run: `"$CLI" core install arduino:avr && "$CLI" lib install "Adafruit PWM Servo Driver Library"`
Expected: both report installed, with no errors.

- [ ] **Step 2: Write `firmware/arm/arm.ino`**

The calibration block at the top of the file must be easy to find and edit:
- `struct Joint { int lo, hi, home, pulseMin, pulseMax; };` and `Joint J[6]` with the starting values from Global Constraints
- `OSC_HZ 27000000`, `SERVO_HZ 50`, `MAX_DEG_PER_S 90`, `WATCHDOG_MS 500`, `BAUD 115200`

Behavior:
- `setup()`: `pwm.begin()`, `pwm.setOscillatorFrequency(OSC_HZ)`, `pwm.setPWMFreq(SERVO_HZ)`. Set `cur` and `tgt` to home and write all six. Then print `READY`, then the `LIMITS` line.
- Serial: read characters into a 48-byte buffer. When a line exceeds the buffer, discard it up to the next `\n`. On `\n`, parse with `sscanf(buf, "P %d %d %d %d %d %d", …)`. Only when that returns exactly 6, constrain each value to `[lo, hi]`, set `tgt`, and set `lastMsg = millis()`. Anything else is ignored.
- Every 20 ms: if at least one valid line has arrived and `millis() - lastMsg > WATCHDOG_MS`, set `tgt = cur` (freeze). Move each `cur` toward `tgt` by at most `MAX_DEG_PER_S × dt`. Then call `pwm.writeMicroseconds(ch, pulseMin + (pulseMax − pulseMin) × cur / 180)`.

- [ ] **Step 3: Compile**

Run: `"$CLI" compile --fqbn arduino:avr:uno firmware/arm`
Expected: `Sketch uses … bytes`, no errors. Flash use under 60%.

- [ ] **Step 4: Commit**

```bash
git add firmware/arm/arm.ino
git commit -m "feat: Uno firmware with limits, smoothing, watchdog"
```

(The hardware check is in Task 5, once the arm is wired.)

---

### Task 4: Laptop program (`arm.py`)

**Files:**
- Create: `laptop/arm.py`

**Interfaces:**
- Consumes: everything in `arm_core` from Tasks 1–2.
- Produces: the CLI `python arm.py [--port COM5] [--dry-run]`.

Constants at the top of the file (calibration knobs, since controller layouts vary):
- `AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, AXIS_LT, AXIS_RT = 0, 1, 2, 3, 4, 5`
- `BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, BTN_BACK, BTN_START = 0, 1, 2, 3, 4, 5, 6, 7`. This is the SDL2 Xbox mapping on Windows.
- `INVERT = [False, True, True, False, False, False]`. Stick-up gives a negative Y value, so the shoulder and elbow flip.
- `DRY_LIMITS`: the starting limits from Global Constraints, used with `--dry-run`.

- [ ] **Step 1: Serial link.** A `Link` class with `__init__(port)` and `send(pose)`, plus `find_port() -> str`.
  - `find_port()`: the first `serial.tools.list_ports` entry whose description contains "Arduino" or "CH340", or whose VID is `0x2341`. Otherwise the first port. With no ports, exit with "No serial port found. Is the Uno plugged in?"
  - `Link.__init__`: opens the port at 115200 and reads lines for up to 5 s until it has seen `READY` and parsed `LIMITS` into `self.limits`. If not, it exits with "Uno didn't report READY/LIMITS — is arm.ino uploaded?"
  - `send(pose)`: writes `format_pose(pose)`.
  - `DryLink`: `limits = DRY_LIMITS`, and `send` prints the line once every 25 ticks (about twice a second).

- [ ] **Step 2: Main loop**

Start pygame. With no joystick, exit with "No controller found." Then start at `pose = home_pose(limits)`, `player = None`, `recorder = None`, and run at `clock.tick(50)`.
- Speeds, in joint order: `[LX, LY, RY, RX, RB−LB, trig(RT)−trig(LT)]` (base, shoulder, elbow, wrist, claw rotate, grip), where `trig(v) = (v + 1) / 2`. RT opens the grip (higher angle).
- Button-down events:
  - **A:** `player = Player(load_program(programs/flip.json), start_pose=pose)`. A `ValueError` prints and is ignored.
  - **B:** `player = None` and print "stopped".
  - **X:** toggle `recorder`. On stop, call `save_recording`, then print the path or "recording too short, discarded".
  - **Y:** play `latest_recording`, or print "no recordings yet".
  - **Start:** `Player([{"t": 0, "pose": home_pose(limits)}], start_pose=pose)`.
  - **Back:** print `json.dumps([round(a) for a in pose])`.
- Each tick:
  - If `player`: when `any_active(speeds)`, abort and print "stopped (stick moved)". Otherwise `pose = player.pose_at(now − play_start)`, and clear `player` when it's done.
  - If not playing: `pose = step_teleop(...)`.
  - Then `pose = clamp_pose(pose, limits)`. If `recorder`, call `recorder.sample(now, pose)`. Finally `link.send(pose)`.
- `JOYDEVICEREMOVED`: print "Controller unplugged — stopping. Arm will freeze." and exit.
- Ctrl+C: exit cleanly.

- [ ] **Step 3: Install the libraries and run the checks**

Run: `cd laptop && python -m pip install -r requirements.txt && python test_arm.py`
Expected: install succeeds (pygame-ce 2.5.8 has a cp314 wheel), then `all checks passed`.

- [ ] **Step 4: Dry run with the controller plugged in**

Run: `cd laptop && python arm.py --dry-run`

Expected:
- With the sticks centered, `P 90 90 90 90 90 90` repeats.
- Pushing the left stick right makes the first number climb to 160 and stop there.
- Back prints the pose.
- A plays the flip: the numbers ease through the keyframes over about 11.5 s.
- B stops it.
- X, wiggle, X saves a `rec-*.json`. Y replays it.
- Without a controller, it prints "No controller found."

- [ ] **Step 5: Commit**

```bash
git add laptop/arm.py
git commit -m "feat: laptop controller loop, record/replay, flip button"
```

---

### Task 5: README and hardware bring-up

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Write the README** with these sections:
  1. **What it is:** one paragraph.
  2. **Parts:** the list from the spec, with a link to `docs/wiring.html`.
  3. **Setup:** install Arduino IDE, the "Adafruit PWM Servo Driver Library", upload `firmware/arm`, then `pip install -r laptop/requirements.txt`.
  4. **Running:** the `python arm.py` and `--dry-run` options.
  5. **Controller map:** the table from the spec.
  6. **Tuning:**
     - Joint limits and home live in the `J[]` table in `arm.ino`. Re-upload after changing them.
     - If a joint can't reach its full travel, adjust `pulseMin`/`pulseMax`.
     - If every joint is off by the same amount, adjust `OSC_HZ`.
     - If a stick moves a joint the wrong way, flip `INVERT` in `arm.py`.
     - To tune the flip: drive to each pose, press Back, and paste the printed pose into `flip.json`.
  7. **Bring-up checklist:** Step 2 below.

- [ ] **Step 2: Bring-up on hardware** (the user does this with the arm wired per `docs/wiring.html`):
  1. Upload the firmware with the 5 V supply unplugged. In Serial Monitor at 115200 you should see `READY` and the `LIMITS` line.
  2. Send `hello`, then `P 90 90`. Nothing should happen, which confirms that garbage lines are ignored.
  3. Plug in the 5 V supply, then press the Uno's reset button. The arm should ease to home, not slam.
  4. Close Serial Monitor and run `python arm.py`. Jog each joint to its ends and tighten `J[]` wherever a joint hits something.
  5. Pull the USB cable mid-move. The arm should freeze within half a second.
  6. Tune the flip and test it with the real object.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: setup, controller map, tuning, bring-up"
```
