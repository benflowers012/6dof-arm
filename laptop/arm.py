"""Drive the arm from an Xbox controller. Run: python arm.py [--port COM5] [--dry-run] [--no-browser]"""
import argparse
import json
import os
import sys
import time
import webbrowser
from collections import deque
from pathlib import Path

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # no window needed
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402
import serial  # noqa: E402
import serial.tools.list_ports  # noqa: E402

from arm_core import (Player, Recorder, any_active, clamp_pose, format_pose, home_pose,  # noqa: E402
                      latest_recording, load_program, parse_limits, parse_pose, save_recording,
                      step_teleop)
from dashboard import serve  # noqa: E402

# ===== Controller mapping (SDL2 Xbox layout on Windows; change if a control does the wrong thing) =====
AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, AXIS_LT, AXIS_RT = 0, 1, 2, 3, 4, 5
BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, BTN_BACK, BTN_START = 0, 1, 2, 3, 4, 5, 6, 7
INVERT = [False, True, True, False, False, False]  # flip a joint if its stick moves it backwards
# Same as the firmware's starting values; used only with --dry-run.
DRY_LIMITS = [(20, 160, 90)] * 4 + [(0, 180, 90), (20, 160, 90)]
# =====================================================================================================

HERE = Path(__file__).parent
FLIP = HERE / "programs" / "flip.json"
RECORDINGS = HERE / "recordings"
JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist", "Claw rotate", "Claw grip"]
AXIS_NAMES = {"LX": AXIS_LX, "LY": AXIS_LY, "RX": AXIS_RX, "RY": AXIS_RY, "LT": AXIS_LT, "RT": AXIS_RT}
BUTTON_NAMES = {"A": BTN_A, "B": BTN_B, "X": BTN_X, "Y": BTN_Y, "LB": BTN_LB, "RB": BTN_RB,
                "View": BTN_BACK, "Menu": BTN_START}
LOG = deque(maxlen=200)  # shown on the dashboard


def log(msg: str):
    print(msg)
    LOG.append(f"{time.strftime('%H:%M:%S')}  {msg}")


def find_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in (p.description or "") or "CH340" in (p.description or "") or p.vid == 0x2341:
            return p.device
    if ports:
        return ports[0].device
    sys.exit("No serial port found. Is the Uno plugged in?")


class Link:
    def __init__(self, port: str):
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, 115200, 0.2
        # DTR off: don't reset the Uno, so it keeps holding its pose instead of snapping to home.
        self.ser.dtr = False
        self.ser.open()
        self.limits = self.start_pose = None
        deadline = time.monotonic() + 5
        next_ask = 0.0
        while time.monotonic() < deadline and not (self.limits and self.start_pose):
            if time.monotonic() >= next_ask:  # re-ask in case the Uno was still booting
                self.ser.write(b"?\n")
                next_ask = time.monotonic() + 0.5
            line = self.ser.readline().decode(errors="ignore").strip()
            try:
                if line.startswith("LIMITS"):
                    self.limits = parse_limits(line)
                elif line.startswith("POSE"):
                    self.start_pose = parse_pose(line)
            except ValueError:
                pass
        if not (self.limits and self.start_pose):
            sys.exit("Uno didn't answer with LIMITS/POSE — is arm.ino uploaded? Wrong port? Try --port.")
        print(f"Connected on {port}. Limits: {self.limits}")

    def send(self, pose):
        try:
            self.ser.write(format_pose(pose).encode())
        except serial.SerialException:
            sys.exit("Lost the USB connection to the Uno — arm will freeze.")


class DryLink:
    limits = DRY_LIMITS
    start_pose = home_pose(DRY_LIMITS)

    def __init__(self):
        self.n = 0

    def send(self, pose):
        if self.n % 25 == 0:
            print(format_pose(pose), end="")
        self.n += 1


def trig(v: float) -> float:
    return (v + 1) / 2  # triggers rest at -1, fully pressed at +1


def open_controller():
    pygame.init()
    deadline = time.monotonic() + 1
    while pygame.joystick.get_count() == 0 and time.monotonic() < deadline:
        pygame.event.pump()
        time.sleep(0.05)
    if pygame.joystick.get_count() == 0:
        sys.exit("No controller found.")
    js = pygame.joystick.Joystick(0)
    print(f"Controller: {js.get_name()}")
    return js


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="serial port, e.g. COM5 (default: auto-detect)")
    ap.add_argument("--dry-run", action="store_true", help="no arm; print the poses instead")
    ap.add_argument("--no-browser", action="store_true", help="don't open the dashboard automatically")
    args = ap.parse_args()

    js = open_controller()
    port = "dry-run" if args.dry_run else (args.port or find_port())
    link = DryLink() if args.dry_run else Link(port)
    limits = link.limits
    pose = clamp_pose(link.start_pose, limits)  # start where the arm already is: no jump
    player = play_start = recorder = None
    label = ""
    clock = pygame.time.Clock()
    t0 = last = time.monotonic()
    hz = 50.0
    latest = [{}]  # the dashboard thread reads latest[0]; swapped whole each tick
    _, url = serve(lambda: latest[0])
    log(f"Dashboard: {url}")
    if not args.no_browser:
        webbrowser.open(url)
    log("A=flip  B=stop  X=record  Y=replay  Menu=home  View=print pose  Ctrl+C=quit")

    def play(frames, name):
        nonlocal player, play_start, label
        player, play_start, label = Player(frames, start_pose=pose), time.monotonic(), name
        log(f"playing {name}")

    while True:
        now = time.monotonic()
        dt, last = now - last, now
        if dt > 0:
            hz = 0.9 * hz + 0.1 / dt
        for ev in pygame.event.get():
            if ev.type == pygame.JOYDEVICEREMOVED:
                sys.exit("Controller unplugged — stopping. Arm will freeze.")
            if ev.type != pygame.JOYBUTTONDOWN:
                continue
            if ev.button == BTN_A:
                try:
                    play(load_program(FLIP), "flip")
                except (OSError, ValueError) as e:
                    log(f"can't load flip: {e}")
            elif ev.button == BTN_B:
                player = None
                log("stopped")
            elif ev.button == BTN_X:
                if recorder is None:
                    recorder = Recorder()
                    recorder.start(now)
                    log("recording... (X to stop)")
                else:
                    path = save_recording(recorder.frames, RECORDINGS)
                    log(f"saved {path}" if path else "recording too short, discarded")
                    recorder = None
            elif ev.button == BTN_Y:
                rec = latest_recording(RECORDINGS)
                if rec is None:
                    log("no recordings yet")
                else:
                    try:
                        play(load_program(rec), rec.name)
                    except ValueError as e:
                        log(f"can't load recording: {e}")
            elif ev.button == BTN_START:
                play([{"t": 0, "pose": home_pose(limits)}], "home")
            elif ev.button == BTN_BACK:
                log("pose: " + json.dumps([round(a) for a in pose]))

        speeds = [
            js.get_axis(AXIS_LX),
            js.get_axis(AXIS_LY),
            js.get_axis(AXIS_RY),
            js.get_axis(AXIS_RX),
            js.get_button(BTN_RB) - js.get_button(BTN_LB),
            trig(js.get_axis(AXIS_RT)) - trig(js.get_axis(AXIS_LT)),
        ]
        if player is not None:
            if any_active(speeds):
                player = None
                log("stopped (stick moved)")
            else:
                pose = player.pose_at(now - play_start)
                if player.done(now - play_start):
                    player = None
                    log(f"done: {label}")
        if player is None:
            pose = step_teleop(pose, speeds, dt, limits, INVERT)
        pose = clamp_pose(pose, limits)
        if recorder is not None:
            recorder.sample(now, pose)
        link.send(pose)

        latest[0] = {
            "port": port,
            "controller": js.get_name(),
            "hz": round(hz),
            "uptime": f"{int(now - t0) // 60}:{int(now - t0) % 60:02d}",
            "mode": "playing" if player else ("recording" if recorder else "teleop"),
            "program": label if player else None,
            "progress": [now - play_start, player.frames[-1]["t"]] if player else None,
            "recording": len(recorder.frames) if recorder else None,
            "sent": format_pose(pose).strip(),
            "joints": [
                {"name": n, "angle": a, "min": lo, "max": hi, "home": h, "speed": s}
                for n, a, (lo, hi, h), s in zip(JOINT_NAMES, pose, limits, speeds)
            ],
            "axes": [{"name": n, "value": trig(js.get_axis(i)) if n in ("LT", "RT") else js.get_axis(i)}
                     for n, i in AXIS_NAMES.items()],  # triggers shown 0 (released) .. 1 (pressed)
            "buttons": [{"name": n, "pressed": bool(js.get_button(i))} for n, i in BUTTON_NAMES.items()],
            "log": list(LOG),
        }
        clock.tick(50)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")
