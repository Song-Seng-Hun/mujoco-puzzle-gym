"""Staged training over checkpoint segments (R21) and their episode starts.

Segments: a room with checkpoint flags f1 < f2 < ... < fK (by `order`) and a
goal splits into K + 1 segments: start->f1, f1->f2, ..., fK->goal. Segment 0
starts like any episode (sampled layout); segment i > 0 starts from the
snapshot `mpg check` saved when the scripted solver reached flag i
(`runs/check/<room>/checkpoints/<flag>.pkl`, R20), so its layout is that
episode's. A segment episode succeeds when the body reaches the segment's end
flag (or the goal for the last one); with `to_goal` it runs on to the goal.

task.yaml (CLI `--curriculum`, `--advance-at`, `--replay` override it):

    curriculum: {mode: forward, advance_at: 0.8, replay: 0.2, window: 50,
                 snapshots: null}     # null: runs/check/<room>/checkpoints (cwd)

- `forward`: only segment 0 is open. When the newest open segment's success rate
  over its last `window` episodes reaches `advance_at`, the next one opens.
  Episodes start at the newest open segment, or with probability `replay` at an
  earlier open one (uniform), and end at that segment's end.
- `reverse`: the last segment opens first and episodes run to the goal; each
  advance opens the segment before it (the start moves back toward the spawn);
  replay draws from the later starts.
- `none`: M1 episodes (start to goal).

Levels: the controller (`Curriculum`) opens an ordered list of levels, each a
dict; today a level is `{"segment": i}`. Difficulty steps (R21 `difficulty`,
plan step 12) extend a level with `"difficulty": k` and order the list by
`order: segment_first | difficulty_first`; the open / replay / advance rules
stay the same.

Training (`train.CurriculumVec`): the env workers draw their own episode start from
the probabilities the main process sends them (`PuzzleEnv.set_segment_draw`);
the main process records each finished episode's segment result, opens levels
and writes `runs/<id>/curriculum.json` (time, open segments, success rate per
segment).
"""

from __future__ import annotations

import pickle
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

MODES = ("forward", "reverse", "none")
DEFAULTS: dict[str, Any] = {"mode": "none", "advance_at": 0.8, "replay": 0.2, "window": 50,
                            "snapshots": None}
LOG_NAME = "curriculum.json"
SNAP_DIR = Path("runs/check")       # same default as `mpg check`


class CurriculumError(ValueError):
    pass


def validate(cfg: dict[str, Any]) -> None:
    if not isinstance(cfg, dict):
        raise CurriculumError("`curriculum` must be a mapping, e.g. {mode: forward}")
    unknown = set(cfg) - set(DEFAULTS)
    if unknown:
        raise CurriculumError(f"unknown curriculum keys {sorted(unknown)}; known: {sorted(DEFAULTS)}")
    if cfg["mode"] not in MODES:
        raise CurriculumError(f"curriculum.mode must be one of {MODES}, got {cfg['mode']!r}")
    a, r, w = cfg["advance_at"], cfg["replay"], cfg["window"]
    if isinstance(a, bool) or not isinstance(a, (int, float)) or not 0 < a <= 1:
        raise CurriculumError(f"curriculum.advance_at must be in (0, 1], got {a!r}")
    if isinstance(r, bool) or not isinstance(r, (int, float)) or not 0 <= r < 1:
        raise CurriculumError(f"curriculum.replay must be in [0, 1), got {r!r}")
    if isinstance(w, bool) or not isinstance(w, int) or w < 1:
        raise CurriculumError(f"curriculum.window must be a whole number >= 1, got {w!r}")


# ---- segments -----------------------------------------------------------------------
def flags(parts: dict[str, Any]) -> list:
    """Checkpoint flags in `order`."""
    return sorted((p for p in parts.values() if p.role == "checkpoint"), key=lambda p: p.order)


def segments(parts: dict[str, Any]) -> list[dict[str, Any]]:
    """[{"name", "start": flag name | None, "end": flag name | "goal"}], start to goal."""
    names = [p.name for p in flags(parts)]
    starts, ends = [None, *names], [*names, "goal"]
    return [{"name": f"{s or 'start'}->{e}", "start": s, "end": e} for s, e in zip(starts, ends)]


def with_flags(env, targets: list, start_xy) -> list:
    """`targets` (the solution order) with the checkpoint flags put in, in order:
    each flag goes at the earliest place (after the previous flag) where it is
    reachable from `start_xy` with the targets before it done, and before the
    final target (the goal); a flag never reachable goes just before the goal.
    So `progress`, `target_pos` and the scripted solver lead through the flags."""
    from .logic import free_mask

    out = list(targets)
    lo = 0
    for f in flags(env.parts):
        last = max(lo, len(out) - 1)
        pos = last
        for p in range(lo, last):
            on = {t.name for t in out[:p]}
            reach = env.grid.flood(free_mask(env.grid, env.walls, env.parts, on), start_xy)
            if (f.target_mask(env.grid) & reach).any():
                pos = p
                break
        out.insert(pos, f)
        lo = pos + 1
    return out


def snapshot_dir(env) -> Path:
    d = (env.task.get("curriculum") or {}).get("snapshots")
    return Path(d) if d else SNAP_DIR / env.room.name / "checkpoints"


def load_snapshots(env) -> dict[str, Any]:
    """The flag snapshots segment starts need (flag name -> Snapshot), checked
    against the env's model. Raises CurriculumError naming what is missing."""
    import mujoco

    from .snapshot import SIG

    d = snapshot_dir(env)
    need = [s["start"] for s in segments(env.parts) if s["start"] is not None]
    missing = [f for f in need if not (d / f"{f}.pkl").exists()]
    if missing:
        raise CurriculumError(
            f"no snapshots for flags {missing} in {d}; run `mpg check {env.task['room']}` "
            "(its solver saves them) or set curriculum.snapshots")
    size = mujoco.mj_stateSize(env.model, SIG)
    out = {}
    for f in need:
        snap = pickle.loads((d / f"{f}.pkl").read_bytes())
        if snap.state.size != size or set(snap.parts) != set(env.parts):
            raise CurriculumError(f"snapshot {d / f'{f}.pkl'} is from another model; "
                                  f"run `mpg check {env.task['room']}` again")
        out[f] = snap
    return out


# ---- the controller -------------------------------------------------------------------
class Curriculum:
    """Opens levels in order from per-level success rates (pure; no env)."""

    def __init__(self, mode: str, n_segments: int, advance_at: float = 0.8, replay: float = 0.2,
                 window: int = 50, names: list[str] | None = None):
        if mode not in MODES:
            raise CurriculumError(f"curriculum mode must be one of {MODES}, got {mode!r}")
        self.mode = mode
        self.advance_at, self.replay, self.window = float(advance_at), float(replay), int(window)
        self.n_segments = n_segments
        self.names = names or [str(i) for i in range(n_segments)]
        order = range(n_segments) if mode == "forward" else reversed(range(n_segments))
        self.levels = [{"segment": i} for i in order] if mode != "none" else [{"segment": 0}]
        self.open = 1
        self.results = [deque(maxlen=self.window) for _ in self.levels]
        self.counts = [0 for _ in self.levels]

    @property
    def to_goal(self) -> bool:
        """Episodes run on to the goal (reverse, none) or stop at the segment end (forward)."""
        return self.mode != "forward"

    def level_of(self, segment: int) -> int | None:
        for i, lv in enumerate(self.levels):
            if lv["segment"] == segment:
                return i
        return None

    def probs(self) -> list[float]:
        """Start probability per segment: the newest open level 1 - replay, the
        earlier open ones share `replay` (all of it to the newest when alone)."""
        p = [0.0] * self.n_segments
        new = self.levels[self.open - 1]["segment"]
        old = [lv["segment"] for lv in self.levels[:self.open - 1]]
        p[new] = 1.0 - (self.replay if old else 0.0)
        for s in old:
            p[s] += self.replay / len(old)
        return p

    def draw(self, rng: np.random.Generator) -> int:
        return int(rng.choice(self.n_segments, p=self.probs()))

    def record(self, segment: int, success: bool) -> bool:
        """Count one finished episode; True when it opened the next level."""
        lv = self.level_of(segment)
        if lv is None or lv >= self.open:
            return False
        self.results[lv].append(bool(success))
        self.counts[lv] += 1
        frontier = self.results[self.open - 1]
        if (lv == self.open - 1 and self.open < len(self.levels)
                and len(frontier) == self.window and np.mean(frontier) >= self.advance_at):
            self.open += 1
            return True
        return False

    def rates(self) -> dict[str, float | None]:
        out = {}
        for lv, res in zip(self.levels, self.results):
            out[self.names[lv["segment"]]] = round(float(np.mean(res)), 3) if res else None
        return out

    def state(self) -> dict[str, Any]:
        return {"mode": self.mode,
                "open": [self.names[lv["segment"]] for lv in self.levels[:self.open]],
                "rates": self.rates(),
                "episodes": {self.names[lv["segment"]]: n for lv, n in zip(self.levels, self.counts)}}


# ---- in the env ---------------------------------------------------------------------
class SegmentDraw:
    """What an env worker needs to start its episodes: start probabilities per segment."""

    def __init__(self, probs: list[float], to_goal: bool):
        self.probs = list(probs)
        self.to_goal = bool(to_goal)

    def draw(self, rng: np.random.Generator) -> int:
        return int(rng.choice(len(self.probs), p=self.probs))


def begin_segment(env, segment: int, to_goal: bool, layout: dict | None = None) -> None:
    """Reset `env` to the start of segment `segment`: a sampled (or given) layout
    for segment 0, else its start flag's snapshot (layout and all)."""
    segs = env.segments
    if not 0 <= segment < len(segs):
        raise CurriculumError(f"segment {segment} out of range: the room has {len(segs)} "
                              f"({[s['name'] for s in segs]})")
    start = segs[segment]["start"]
    if start is None:
        env._reset_sim(env.np_random, layout)
    else:
        if env._start_snaps is None:
            env._start_snaps = load_snapshots(env)
        snap = env._start_snaps[start]
        env._reset_sim(None, {"spawn": env.room.sample_spawn(None),
                              "parts": {n: xy.copy() for n, xy in snap.part_xy.items()}})
        env.restore_snapshot(snap)
        env.checkpoint_snap = snap
    env.segment = {"index": segment, "end": "goal" if to_goal else segs[segment]["end"]}


def segment_done(env, goal_success: bool) -> bool:
    end = env.segment["end"]
    return goal_success if end == "goal" else bool(env.parts[end].reached)
