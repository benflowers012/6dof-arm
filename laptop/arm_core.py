"""Hardware-free arm logic: limits, teleop, keyframe playback, recording."""
import json
from datetime import datetime
from pathlib import Path

Limits = list[tuple[int, int, int]]  # six (min, max, home)


def parse_limits(line: str) -> Limits:
    parts = line.split()
    if len(parts) != 19 or parts[0] != "LIMITS":
        raise ValueError(f"not a LIMITS line: {line!r}")
    try:
        nums = [int(p) for p in parts[1:]]
    except ValueError:
        raise ValueError(f"bad number in LIMITS line: {line!r}") from None
    return [tuple(nums[i:i + 3]) for i in range(0, 18, 3)]


def parse_pose(line: str) -> list[float]:
    parts = line.split()
    if len(parts) != 7 or parts[0] != "POSE":
        raise ValueError(f"not a POSE line: {line!r}")
    try:
        return [int(p) for p in parts[1:]]
    except ValueError:
        raise ValueError(f"bad number in POSE line: {line!r}") from None


def clamp_pose(pose: list[float], limits: Limits) -> list[float]:
    return [min(max(a, lo), hi) for a, (lo, hi, _) in zip(pose, limits)]


def format_pose(pose: list[float]) -> str:
    return "P " + " ".join(str(round(a)) for a in pose) + "\n"


def home_pose(limits: Limits) -> list[float]:
    return [home for _, _, home in limits]


def step_teleop(pose, speeds, dt, limits, invert, max_speed=60.0, deadzone=0.15):
    out = [
        a + (0 if abs(s) < deadzone else (-s if inv else s)) * max_speed * dt
        for a, s, inv in zip(pose, speeds, invert)
    ]
    return clamp_pose(out, limits)


def any_active(speeds: list[float], deadzone=0.15) -> bool:
    return any(abs(s) >= deadzone for s in speeds)


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return 3 * x * x - 2 * x * x * x


class Player:
    """Eases between keyframes; with start_pose, first eases from there over lead_in seconds."""

    def __init__(self, frames, start_pose=None, lead_in=1.5):
        if start_pose is not None:
            frames = [{"t": 0, "pose": list(start_pose)}] + [
                {"t": f["t"] + lead_in, "pose": f["pose"]} for f in frames
            ]
        self.frames = frames

    def pose_at(self, t: float) -> list[float]:
        f = self.frames
        if t <= f[0]["t"]:
            return list(f[0]["pose"])
        for a, b in zip(f, f[1:]):
            if t <= b["t"]:
                k = smoothstep((t - a["t"]) / (b["t"] - a["t"]))
                return [x + (y - x) * k for x, y in zip(a["pose"], b["pose"])]
        return list(f[-1]["pose"])

    def done(self, t: float) -> bool:
        return t >= self.frames[-1]["t"]


class Recorder:
    def __init__(self, interval=0.05):
        self.interval = interval
        self.frames: list[dict] = []

    def start(self, t: float):
        self.t0 = t
        self.frames = []

    def sample(self, t: float, pose: list[float]):
        rel = t - self.t0
        # ponytail: 1e-9 slack so float sums like 0.1-0.05 don't drop a sample
        if not self.frames or rel - self.frames[-1]["t"] >= self.interval - 1e-9:
            self.frames.append({"t": rel, "pose": list(pose)})


def load_program(path: Path) -> list[dict]:
    path = Path(path)
    try:
        frames = json.loads(path.read_text())["frames"]
        ok = bool(frames) and all(
            len(f["pose"]) == 6 and all(isinstance(a, (int, float)) for a in f["pose"])
            and isinstance(f["t"], (int, float))
            for f in frames
        ) and all(a["t"] < b["t"] for a, b in zip(frames, frames[1:]))
    except (KeyError, TypeError, json.JSONDecodeError) as e:
        raise ValueError(f"{path.name}: {e}") from None
    if not ok:
        raise ValueError(f"{path.name}: needs a non-empty 'frames' list of {{t, pose[6]}} with t increasing")
    return frames


def save_recording(frames: list[dict], folder: Path, now: datetime | None = None) -> Path | None:
    if len(frames) < 2:
        return None
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"rec-{(now or datetime.now()):%Y%m%d-%H%M%S}.json"
    path.write_text(json.dumps({"name": path.stem, "frames": frames}))
    return path


def latest_recording(folder: Path) -> Path | None:
    recs = sorted(Path(folder).glob("rec-*.json"))
    return recs[-1] if recs else None
