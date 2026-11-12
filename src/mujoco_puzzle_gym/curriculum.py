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
dict `{"segment": i}`, or `{"segment": i, "difficulty": k}` when task.yaml has
difficulty steps. `curriculum.order` sets the list: `difficulty_first`
(default; every step of a segment before the next segment: A->B plain, A->B
with a weak cannon, a strong one, then B->C) or `segment_first` (every
segment at step 0, then every segment at step 1, ...). The open / replay /
advance rules are the same for both. With steps and `mode: none` the levels
are the steps of the whole run (start to goal).

Difficulty steps (R21 `difficulty`, task.yaml):

    difficulty:
      - {param: cannon_a.power, from: 0.5, to: 2.0, steps: 4}   # 0.5, 1.0, 1.5, 2.0
      - {param: cannon_a.enabled, from: 0, to: 1, at_step: 1}    # off at step 0

`steps: n` spreads n values evenly from `from` to `to` (step n-1 and later:
`to`); `at_step: k` is `from` before step k and `to` from it on. The number of
steps is the largest n or k + 1. Only values the schema marks run-time or
episode-start may be stepped, and not one a `bind` drives (it would overwrite
the step value). The env applies step k at every episode start (`apply_difficulty`,
also over a segment's start snapshot); without a step given (`reset(options=
{"difficulty": k})` or the draw) it uses the last, hardest one, so `mpg eval`
and `--full` run there unless `--difficulty k` says otherwise.

Training (`train.CurriculumVec`): the env workers draw their own episode start from
the probabilities the main process sends them (`PuzzleEnv.set_segment_draw`);
the main process records each finished episode's segment result, opens levels
and writes `runs/<id>/curriculum.json` (time, open segments, success rate per
segment).
"""

from __future__ import annotations

import copy
import pickle
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

MODES = ("forward", "reverse", "none")
ORDERS = ("difficulty_first", "segment_first")
DEFAULTS: dict[str, Any] = {"mode": "none", "advance_at": 0.8, "replay": 0.2, "window": 50,
                            "snapshots": None, "order": "difficulty_first"}
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
    if cfg["order"] not in ORDERS:
        raise CurriculumError(f"curriculum.order must be one of {ORDERS}, got {cfg['order']!r}")
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
    from .trust import require_trusted

    d = snapshot_dir(env)
    require_trusted(d, "snapshot folder")
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


# ---- difficulty steps (R21) -----------------------------------------------------------
@dataclass
class Step:
    """One stepped value: `values[k]` at difficulty step k."""

    part: str
    key: str
    values: list


def parse_difficulty(raw: Any, parts: dict[str, Any]) -> list[Step]:
    """task.yaml `difficulty:` checked against the room's parts (CurriculumError)."""
    if not raw:
        return []
    if not isinstance(raw, list):
        raise CurriculumError("`difficulty` must be a list of {param: <part>.<value>, from, to, "
                              "steps: n} or {param, from, to, at_step: k}")
    rows, n_steps = [], 1
    for i, e in enumerate(raw):
        where = f"difficulty[{i}]"
        if (not isinstance(e, dict) or not isinstance(e.get("param"), str) or "from" not in e
                or "to" not in e or ("steps" in e) == ("at_step" in e)
                or set(e) - {"param", "from", "to", "steps", "at_step"}):
            raise CurriculumError(f"{where}: write {{param: <part>.<value>, from, to, steps: n}} "
                                  f"or {{param, from, to, at_step: k}}, got {e!r}")
        name, _, key = e["param"].rpartition(".")
        p = parts.get(name)
        if p is None:
            raise CurriculumError(f"{where}: no part {name!r} (parts: {sorted(parts)})")
        f = p.full_schema().get(key)
        if f is None or not (f.runtime or f.episode):
            ok = sorted(k for k, g in p.full_schema().items() if g.runtime or g.episode)
            raise CurriculumError(f"{where}: {e['param']} cannot be stepped; only values marked run-time "
                                  f"or episode-start can ({name}: {ok})")
        if key in p.binds:
            raise CurriculumError(f"{where}: {e['param']} is driven by a `bind`, which would overwrite "
                                  f"the step value every body step")
        why = p.why_not_stepped(key)
        if why:
            raise CurriculumError(f"{where}: {e['param']} cannot be stepped: {why}")
        k = e.get("steps", e.get("at_step"))
        lo = 1 if "steps" in e else 0
        if isinstance(k, bool) or not isinstance(k, int) or k < lo:
            what = "steps" if "steps" in e else "at_step"
            raise CurriculumError(f"{where}: {what} must be a whole number >= {lo}, got {k!r}")
        n_steps = max(n_steps, k if "steps" in e else k + 1)
        rows.append((p, key, f, e, where))
    out = []
    for p, key, f, e, where in rows:
        try:
            a, b = np.asarray(e["from"], float), np.asarray(e["to"], float)
        except (TypeError, ValueError):
            raise CurriculumError(f"{where}: from and to must be numbers or lists of numbers") from None
        vals = []
        for k in range(n_steps):
            if "steps" in e:
                n = e["steps"]
                v = b if n == 1 else a + (b - a) * min(k, n - 1) / (n - 1)
            else:
                v = b if k >= e["at_step"] else a
            v = v.tolist()
            if f.kind == "int":
                v = int(round(v))
            try:
                vals.append(f.check(f"{where}: {e['param']} at step {k}", v))
            except ValueError as exc:
                raise CurriculumError(str(exc)) from None
        out.append(Step(p.name, key, vals))
    return out


def difficulty_step(env, k: int | None) -> int | None:
    """Difficulty step `k` checked against the task (None: the last; None without a table)."""
    if not env.difficulty:
        return None
    n = env.difficulty_steps
    k = n - 1 if k is None else int(k)
    if not 0 <= k < n:
        raise CurriculumError(f"difficulty step {k} out of range: the task has {n} (0..{n - 1})")
    return k


def apply_difficulty(env, k: int | None) -> None:
    """Set difficulty step `k` (None: the last) on `env`'s run-time values. `_reset_sim`
    calls it after the room values come back and before parts reset and the plan, so
    parts start at the step value and the start snapshot holds it."""
    k = difficulty_step(env, k)
    env.difficulty_level = k
    for st in env.difficulty:
        env.parts[st.part].set_value(env.model, env.data, st.key, copy.deepcopy(st.values[k]), episode=True)


# ---- the controller -------------------------------------------------------------------
class Curriculum:
    """Opens levels in order from per-level success rates (pure; no env)."""

    def __init__(self, mode: str, n_segments: int, advance_at: float = 0.8, replay: float = 0.2,
                 window: int = 50, names: list[str] | None = None, difficulties: int = 1,
                 order: str = "difficulty_first"):
        if mode not in MODES:
            raise CurriculumError(f"curriculum mode must be one of {MODES}, got {mode!r}")
        if order not in ORDERS:
            raise CurriculumError(f"curriculum order must be one of {ORDERS}, got {order!r}")
        self.mode = mode
        self.advance_at, self.replay, self.window = float(advance_at), float(replay), int(window)
        self.n_segments = n_segments
        self.names = names or [str(i) for i in range(n_segments)]
        self.difficulties = int(difficulties)
        segs = (list(range(n_segments)) if mode == "forward" else list(reversed(range(n_segments)))
                if mode == "reverse" else [0])
        if self.difficulties == 1:
            self.levels = [{"segment": i} for i in segs]
        elif order == "difficulty_first":
            self.levels = [{"segment": i, "difficulty": k} for i in segs for k in range(self.difficulties)]
        else:
            self.levels = [{"segment": i, "difficulty": k} for k in range(self.difficulties) for i in segs]
        self.open = 1
        self.results = [deque(maxlen=self.window) for _ in self.levels]
        self.counts = [0 for _ in self.levels]

    @property
    def to_goal(self) -> bool:
        """Episodes run on to the goal (reverse, none) or stop at the segment end (forward)."""
        return self.mode != "forward"

    def level_name(self, lv: dict[str, int]) -> str:
        name = self.names[lv["segment"]]
        return name if "difficulty" not in lv else f"{name}@d{lv['difficulty']}"

    def level_of(self, segment: int, difficulty: int | None = None) -> int | None:
        for i, lv in enumerate(self.levels):
            if lv["segment"] == segment and lv.get("difficulty", difficulty) == difficulty:
                return i
        return None

    def level_probs(self) -> list[float]:
        """Start probability per level: the newest open level 1 - replay, the
        earlier open ones share `replay` (all of it to the newest when alone)."""
        p = [0.0] * len(self.levels)
        old = self.open - 1
        p[old] = 1.0 - (self.replay if old else 0.0)
        for i in range(old):
            p[i] = self.replay / old
        return p

    def probs(self) -> list[float]:
        """Start probability per segment (the levels' summed)."""
        p = [0.0] * self.n_segments
        for lv, q in zip(self.levels, self.level_probs()):
            p[lv["segment"]] += q
        return p

    def draw(self, rng: np.random.Generator) -> int:
        return int(rng.choice(self.n_segments, p=self.probs()))

    def record(self, segment: int, success: bool, difficulty: int | None = None) -> bool:
        """Count one finished episode; True when it opened the next level."""
        lv = self.level_of(segment, difficulty)
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
        return {self.level_name(lv): round(float(np.mean(res)), 3) if res else None
                for lv, res in zip(self.levels, self.results)}

    def state(self) -> dict[str, Any]:
        return {"mode": self.mode,
                "open": [self.level_name(lv) for lv in self.levels[:self.open]],
                "rates": self.rates(),
                "episodes": {self.level_name(lv): n for lv, n in zip(self.levels, self.counts)}}


# ---- in the env ---------------------------------------------------------------------
class SegmentDraw:
    """What an env worker needs to start its episodes: start probabilities per
    segment, or per level when `levels` (dicts with "segment", "difficulty") is given."""

    def __init__(self, probs: list[float], to_goal: bool, levels: list[dict] | None = None):
        self.probs = list(probs)
        self.to_goal = bool(to_goal)
        self.levels = (list(levels) if levels is not None
                       else [{"segment": i} for i in range(len(self.probs))])

    def draw_level(self, rng: np.random.Generator) -> dict[str, int]:
        return self.levels[int(rng.choice(len(self.probs), p=self.probs))]

    def draw(self, rng: np.random.Generator) -> int:
        return self.draw_level(rng)["segment"]


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
        if env.difficulty:            # the flag snapshot holds the check run's values: step them again
            apply_difficulty(env, env._difficulty_k)
            snap = copy.copy(snap)
            snap.values = {**snap.values, **{st.part: copy.deepcopy(env.parts[st.part].live)
                                             for st in env.difficulty}}
        env.checkpoint_snap = snap
    env.segment = {"index": segment, "end": "goal" if to_goal else segs[segment]["end"]}


def segment_done(env, goal_success: bool) -> bool:
    end = env.segment["end"]
    return goal_success if end == "goal" else bool(env.parts[end].reached)
