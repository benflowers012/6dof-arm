<img src="assets/icon.svg" width="64" height="64" alt="">

# robotics arm type splish

A 6-servo desktop robot arm for a class project. Drive it live with an Xbox controller, record moves and play them back, or press one button to run the **flip**: grab an object in front, swing it backwards over the top, and set it down behind the arm, still upright.

The laptop does the thinking (`laptop/arm.py`). The Arduino Uno (`firmware/arm/arm.ino`) keeps every joint inside its safe range, moves the servos smoothly, and freezes the arm if the laptop stops talking.

## Parts

- Arduino Uno R3 and a USB A–B cable
- Adafruit 16-channel PWM servo driver (PCA9685)
- 6× MG996R servos and servo extension cables
- 5 V 10 A power supply, plus a female barrel-jack-to-screw-terminal adapter
- 1000 µF 16 V electrolytic capacitor
- 4 male-to-female jumper wires, and about 1 ft of 18–20 AWG wire for servo power
- Xbox controller and its USB cable, plus a laptop

**Wiring:** open [`docs/wiring.html`](docs/wiring.html) in a browser.

## Setup

1. **Arduino:** in the Arduino IDE, open Library Manager and install **Adafruit PWM Servo Driver Library**. Open `firmware/arm/arm.ino`, choose board **Arduino Uno**, and upload. Keep the 5 V servo supply unplugged while you upload.
2. **Laptop:** install Python 3, then run:
   ```
   cd laptop
   python -m pip install -r requirements.txt
   python test_arm.py        # should print: all checks passed
   ```

## Running

```
cd laptop
python arm.py               # auto-finds the Uno
python arm.py --port COM5   # if it picks the wrong port
python arm.py --dry-run     # no arm needed: prints the angles it would send
```

Close the Arduino Serial Monitor first, because only one program can use the port at a time. Restarting `arm.py` doesn't move the arm: it asks the Uno where the arm is and carries on from there.

## Controller map

| Input | Action |
|-------|--------|
| Left stick ↔ | Base |
| Left stick ↕ | Shoulder |
| Right stick ↕ | Elbow |
| Right stick ↔ | Wrist |
| LB / RB | Twist the claw |
| LT / RT | Close / open the claw |
| **A** | Run the flip |
| **B** | **Stop**: abort what's playing and hold still |
| **X** | Start / stop recording |
| **Y** | Play the most recent recording |
| Start | Go home |
| Back | Print the current angles (for tuning) |

The sticks control speed: the further you push, the faster the joint moves. Touching a stick while something is playing also stops it. Recordings are saved in `laptop/recordings/`.

## Tuning

| What's wrong | Where to fix it |
|-------------|-----------------|
| A joint hits something, or you want it to stop sooner | `lo` / `hi` in the `J[]` table at the top of `arm.ino`, then re-upload |
| The home pose is wrong | `home` in the same table |
| A servo can't reach the ends of its travel | `pulseMin` / `pulseMax` for that joint (try 600/2400, or 450/2550) |
| Every joint is off by about the same amount | `OSC_HZ` (try 26–28 million) |
| A stick moves a joint the wrong way | `INVERT` at the top of `arm.py` |
| A button or stick does the wrong thing | the `AXIS_*` / `BTN_*` numbers at the top of `arm.py` |
| The arm moves too fast or too slow overall | `MAX_DEG_PER_S` in `arm.ino` |

**Tuning the flip:** the angles in `laptop/programs/flip.json` are placeholders.
1. Run `python arm.py`.
2. Drive the arm to each position of the flip: open over the object, closed on it, lifted, swinging over the top with the claw twisted, lowered behind, released.
3. At each position, press **Back** and paste the printed angles into that step's `"pose"` in `flip.json`.
4. Adjust the `"t"` times, in seconds from the start. They must keep increasing.
5. Press **A** to try it. There's no need to restart `arm.py`, because the file is re-read on every press.

If the shoulder sags or buzzes partway through the flip, it can't hold the arm stretched out. Tune the flip to keep the elbow bent as it passes over the top.

## First power-on checklist

1. Upload the firmware with the 5 V supply **unplugged**. In Serial Monitor at 115200 baud you should see `READY`, a `LIMITS …` line and a `POSE …` line. Typing `?` prints the last two again.
2. Type `hello` and press Enter, then `P 90 90`. Nothing should happen, which shows that bad commands are ignored.
3. With the 5 V supply still unplugged, **move the arm by hand to roughly its home pose** (every joint near the middle of its travel). Then plug in the 5 V supply. The servos have no way to know where they are, so on power-up every joint jumps straight to home at full speed. Starting near home keeps that jump small. Do this every time you power up.
4. Close Serial Monitor and run `python arm.py`. Move each joint to its ends, and tighten `lo`/`hi` wherever a joint hits something.
5. Pull the USB cable while the arm is moving. It should freeze within half a second.
6. Tune the flip (above) and test it with the real object.
