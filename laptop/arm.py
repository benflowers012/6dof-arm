"""Drive the arm from an Xbox controller. Run: python arm.py [--port COM5] [--dry-run]"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # no window needed
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402
import serial  # noqa: E402
import serial.tools.list_ports  # noqa: E402

from arm_core import (Player, Recorder, any_active, clamp_pose, format_pose, home_pose,  # noqa: E402
                      latest_recording, load_program, parse_limits, save_recording, step_teleop)

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
        self.ser = serial.Serial(port, 115200, timeout=0.2)  # opening resets the Uno
        self.limits = None
        ready = False
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not (ready and self.limits):
            line = self.ser.readline().decode(errors="ignore").strip()
            if line == "READY":
                ready = True
            elif line.startswith("LIMITS"):
                try:
                    self.limits = parse_limits(line)
                except ValueError:
                    pass
        if not (ready and self.limits):
            sys.exit("Uno didn't report READY/LIMITS — is arm.ino uploaded?")
        print(f"Connected on {port}. Limits: {self.limits}")

    def send(self, pose):
        self.ser.write(format_pose(pose).encode())


class DryLink:
    limits = DRY_LIMITS

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
    args = ap.parse_args()

    js = open_controller()
    link = DryLink() if args.dry_run else Link(args.port or find_port())
    limits = link.limits
    pose = home_pose(limits)
    player = play_start = recorder = None
    clock = pygame.time.Clock()
    last = time.monotonic()
    print("A=flip  B=stop  X=record  Y=replay  Start=home  Back=print pose  Ctrl+C=quit")

    def play(frames, label):
        nonlocal player, play_start
        player, play_start = Player(frames, start_pose=pose), time.monotonic()
        print(f"playing {label}")

    while True:
        now = time.monotonic()
        dt, last = now - last, now
        for ev in pygame.event.get():
            if ev.type == pygame.JOYDEVICEREMOVED:
                sys.exit("Controller unplugged — stopping. Arm will freeze.")
            if ev.type != pygame.JOYBUTTONDOWN:
                continue
            if ev.button == BTN_A:
                try:
                    play(load_program(FLIP), "flip")
                except (OSError, ValueError) as e:
                    print(f"can't load flip: {e}")
            elif ev.button == BTN_B:
                player = None
                print("stopped")
            elif ev.button == BTN_X:
                if recorder is None:
                    recorder = Recorder()
                    recorder.start(now)
                    print("recording... (X to stop)")
                else:
                    path = save_recording(recorder.frames, RECORDINGS)
                    print(f"saved {path}" if path else "recording too short, discarded")
                    recorder = None
            elif ev.button == BTN_Y:
                rec = latest_recording(RECORDINGS)
                if rec is None:
                    print("no recordings yet")
                else:
                    try:
                        play(load_program(rec), rec.name)
                    except ValueError as e:
                        print(f"can't load recording: {e}")
            elif ev.button == BTN_START:
                play([{"t": 0, "pose": home_pose(limits)}], "home")
            elif ev.button == BTN_BACK:
                print(json.dumps([round(a) for a in pose]))

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
                print("stopped (stick moved)")
            else:
                pose = player.pose_at(now - play_start)
                if player.done(now - play_start):
                    player = None
                    print("done")
        if player is None:
            pose = step_teleop(pose, speeds, dt, limits, INVERT)
        pose = clamp_pose(pose, limits)
        if recorder is not None:
            recorder.sample(now, pose)
        link.send(pose)
        clock.tick(50)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")
