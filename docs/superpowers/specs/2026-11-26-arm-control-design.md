# Robot Arm Control: Design

Date: 2026-11-26

## Intent

Control code for a 6-servo desktop robot arm built as a **class project**. The demo
has to show two things: driving the arm by hand with an Xbox controller, and the arm
running programs on its own. One of those programs is the "flip": pick up an object
in front of the arm, swing it backwards over the top, and set it down behind the arm
still upright.

## Hardware (as given)

- 6× MG996R servos, direct drive, no gear reduction. They give no position feedback.
- Arduino Uno R3 as the controller.
- Adafruit 16-channel PWM servo driver (PCA9685). It talks to the Uno over I2C, and
  the servos plug into it on channels 0–5.
- 5 V 10 A switching power supply with a barrel plug, a female barrel-jack-to-screw-terminal
  adapter, a 1000 µF 16 V electrolytic capacitor, and servo extension cables.
- Joint order, base upward (wiring diagram: [`docs/wiring.html`](../../wiring.html)):

| # | Joint       | Motion                                   |
|---|-------------|------------------------------------------|
| 0 | Base        | Rotates the whole arm                    |
| 1 | Shoulder    | Lifts the arm up and down                |
| 2 | Elbow       | Bends between bicep and forearm          |
| 3 | Wrist       | Tilts the claw                           |
| 4 | Claw rotate | Twists the claw                          |
| 5 | Claw grip   | Opens and closes the claw                |

The arm can bend past vertical, so it can reach behind itself.

**Power (required):** the 5 V 10 A supply feeds the driver board's blue V+/GND screw
terminal through the barrel adapter, using 18–20 AWG wire (jumper wires are too thin).
The 1000 µF capacitor goes across that terminal, stripe leg to GND. The Uno's 5 V and
GND pins feed only the driver's logic (`VCC`/`GND`), and that also gives the two boards
a shared ground. Never power the servos from the Uno's 5 V pin.

**Torque risk:** an MG996R at 5 V gives roughly 9 kg·cm. With direct drive, the shoulder
holds the whole arm, and the arm is stretched out partway through the flip. If the
shoulder sags or buzzes there, shorten or lighten the arm, or tune the flip to keep
the elbow folded while it passes over the top.

## Goals

1. **Teleop:** drive all six joints live from an Xbox controller plugged into a laptop.
2. **Record and replay:** record what you did while driving, save it, and play it back.
3. **Pre-programmed moves on one button:** the flip, plus "go home".
4. **Safety:** per-joint angle limits that always apply, a stop button, and no wild
   jump when the arm powers on.
5. **Easy tuning:** the flip's poses get tuned on the real arm without re-flashing the
   Uno.

## Non-goals

- Inverse kinematics or moving the claw in x/y/z coordinates. Every motion is
  joint angles.
- Running without a laptop. A laptop is present at the demo.
- Sensors, cameras, or detecting objects. The flip assumes the object sits in a
  known spot.
- A graphical UI.

## Anti-goals (failure even if it "works")

- The arm hits itself or the table during the demo because a command went past
  a joint's safe range.
- The arm slams to a random pose at power-on or when the laptop connects.
- The arm keeps moving after the laptop crashes or the cable is pulled.

## Approaches considered

| Approach | Trade-off | Verdict |
|----------|-----------|---------|
| **A. Laptop runs the logic, Uno drives the servos** | Python reads the controller, records and replays, and runs programs. It streams target angles to the Uno, which smooths the motion and enforces limits. Programs and recordings are text files, so tuning needs no re-flash. Needs Python plus `pygame-ce` and `pyserial`. | **Chosen** |
| B. Uno runs the logic, laptop only forwards input | Every tweak to the flip needs a re-flash. Recordings must fit in 1 KB of EEPROM (~150 poses). Harder to debug. | Rejected |
| C. Browser page (Web Serial and the Gamepad API) instead of Python | Nicer on screen for a demo, but only works in Chrome or Edge, and the UI is more code. | Rejected |

## Design

### Split of responsibilities

```
Xbox controller --USB--> Laptop (arm.py) --USB serial--> Uno (arm.ino) --I2C--> PCA9685 --> 6 servos
                          - teleop                        - clamp to joint limits
                          - record / replay               - smooth motion (max deg/s)
                          - programs (flip, home)         - watchdog: freeze if silent
```

### Uno firmware (`firmware/arm/arm.ino`)

- Uses the **Adafruit PWM Servo Driver Library** (Arduino Library Manager). The driver
  is at I2C address 0x40, wired to the Uno's SDA/SCL pins (A4/A5). Servos go on
  channels **0–5**, in joint order. PWM is 50 Hz.
- Per-joint calibration table at the top of the file: `min`, `max`, `home` (degrees),
  plus `pulseMin`/`pulseMax` (µs, starting at 500/2500), which map 0–180° to that
  servo's real travel. These are the calibration knobs. The arm will need them tuned.
- `OSC_HZ` constant for the PCA9685's oscillator (starting at 27 MHz). These chips run
  a few percent off nominal, which shifts every angle. Tune it once if all the joints
  read off by the same amount.
- **Power-on:** the PCA9685 sends no pulses until it's told to, so the servos stay
  limp. `setup()` sends the `home` pose first, so the first thing the arm does is go home.
- **Protocol (laptop → Uno),** 115200 baud, one line per update:
  `P a0 a1 a2 a3 a4 a5\n`, integer degrees. A malformed line is ignored.
- **Uno → laptop:** on boot it prints `READY` and then
  `LIMITS min0 max0 home0 ... min5 max5 home5`. The laptop reads the limits from this
  line, so the limits are defined in one place only.
- Every target is clamped to `[min, max]`. Each loop, every servo moves toward its
  target at no more than a set speed (`MAX_DEG_PER_S`, one constant, starting around
  90°/s). This keeps the motion smooth even when the laptop jumps a target.
- **Watchdog:** if no valid `P` line arrives for 500 ms, the targets freeze at the
  current positions. Since the arm is direct-drive, the servos then hold that pose.

### Laptop program (`laptop/arm.py`)

One script, run as `python arm.py [--port COM5] [--dry-run]`.
- With no `--port`, it auto-picks the first Arduino-looking serial port.
- `--dry-run` skips the serial link and prints the poses it would send instead, for
  testing without the arm.
- After opening the port, it waits for `READY` (opening the port resets the Uno), then
  parses `LIMITS`.
- Main loop at 50 Hz: work out the commanded pose, clamp it to the limits, and send
  a `P` line. Clamping on the laptop side means holding a stick past a limit doesn't
  "wind up".

**Controller mapping (builder's call, listed here as the default):**

| Input | Action |
|-------|--------|
| Left stick X | Base |
| Left stick Y | Shoulder |
| Right stick Y | Elbow |
| Right stick X | Wrist |
| LB / RB | Claw rotate − / + |
| LT / RT | Claw close / open |
| **A** | Run the flip |
| **B** | **Stop:** abort any program or playback and hold the current pose |
| **X** | Start / stop recording |
| **Y** | Play the most recent recording |
| Start | Go home |
| Back | Print the current pose to the terminal (used for tuning) |

- Sticks and triggers set joint **speed**, not position. Full deflection is about
  60°/s, with a deadzone of about 0.15.
- Moving any stick or trigger past the deadzone while a program plays aborts it, the
  same as pressing B.

### Programs and recordings: one format, one player

Both are JSON lists of keyframes:

```json
{"name": "flip", "frames": [
  {"t": 0.0, "pose": [90, 90, 90, 90, 90, 40]},
  {"t": 1.5, "pose": [90, 60, 120, 70, 90, 40]}
]}
```

`t` is seconds from the start. The player eases between frames (smoothstep) and
streams the result at 50 Hz.

- **Programs** live in `laptop/programs/`: `flip.json` and `home.json`. `home.json`
  is generated from the Uno's `home` values. If a program needs a pose outside the
  limits, the clamp still applies.
- **Recordings:** pressing X samples the commanded pose at 20 Hz. Pressing X again
  saves the frames to `laptop/recordings/rec-YYYYMMDD-HHMMSS.json`, and Y plays the
  newest one. Before a recording plays, the arm first eases to its first frame over
  1.5 s, so it never jumps.
- **The flip:** `flip.json` ships with placeholder poses that follow this sequence:
  open claw over the pickup spot → close → lift → swing the shoulder and elbow back
  past vertical while the claw rotates 180° → lower behind → open → return home.
  The real angles get tuned on the arm by driving to each pose, pressing Back, and
  pasting the printed pose into the file.

### Repo layout

```
firmware/arm/arm.ino
laptop/arm.py
laptop/requirements.txt      # pygame-ce, pyserial
laptop/programs/flip.json
laptop/programs/home.json
laptop/recordings/           # created on first save
README.md                    # wiring, power, setup, controller map, tuning steps
```

### Checks

- Python: one small `laptop/test_arm.py` with plain `assert`s covering keyframe
  interpolation, clamping, and parsing of the `LIMITS` line. It runs without
  hardware.
- Firmware: compiles with `arduino-cli` if it's installed. On hardware, the bring-up
  checklist in the README covers the rest: home pose at power-on, limits, and the
  watchdog (unplug USB mid-move and the arm must freeze).

## Decided by the user

- Hardware: 6× MG996R direct drive, Arduino Uno, Xbox controller.
- The laptop is present at the demo, and the controller plugs into it.
- Approach A.
- Features: teleop, record and replay, pre-programmed one-button moves with the flip
  as the main one.

## Left to the builder (Claude's calls, open to correction)

- Exact controller mapping, speeds, deadzone, and watchdog timeout (defaults above).
- Smoothing inside the firmware (rate limit) as well as easing in the player.
- `pygame-ce` rather than `pygame`, because this machine runs Python 3.14 and
  `pygame-ce` publishes current wheels.
- Joint limits and home values start at a conservative 20–160°, home at 90°, until
  they're tuned on the real arm. **Exception:** claw rotate starts at 0–180°, since
  the flip needs a full half-turn.
- Known risk: MG996R units often travel less than a true 180°. If claw rotate falls
  short, the wrist tilt makes up the difference in `flip.json`. The servo pulse range
  (`pulseMin`/`pulseMax`) is a per-joint calibration value, so travel can be
  widened without changing code.
- A per-joint `invert` flag on the laptop side, so a stick pushed up moves the joint
  the way you'd expect regardless of how the servo is mounted.
