"""`mpg eval runs/<id> --episodes 100` (R10) and `mpg render runs/<id>` (R11).

Eval runs the saved policy on held-out layouts: episode k uses env seed
EVAL_SEED_BASE + k, a range training never touches (see train.py). Episodes
run in parallel worker processes, each loading the checkpoint itself.

`--full` and `--segments` (R22) work on rooms with checkpoint flags
(curriculum.py segments):
- full: episodes from the start (sampled layout, no snapshot) to the goal; the
  same episodes plain `mpg eval` runs, reported as `full_success_rate`.
- With task difficulty steps (R21) every episode runs at the last (hardest)
  step unless `--difficulty k` picks one; the report names it (`difficulty`).
- segments: for every eval seed, one episode per segment from its start (segment
  0 from the sampled layout, the others from their flag snapshot) to its end;
  `segment_success_rates` per segment. With both, `chain_gap_rate` is the
  fraction of seeds whose segments all succeeded while the full run failed.

Render replays eval seeds until one succeeds (or replays `--seed`) and writes
an MP4 with MuJoCo's offscreen renderer; the backend follows MUJOCO_GL
(e.g. `MUJOCO_GL=egl` on headless Linux).
"""

from __future__ import annotations

import multiprocessing as mp
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

from .train import EVAL_SEED_BASE, load_run, load_trainer, window_stats

DEFAULT_WORKERS = 12


class RandomPolicy:
    """Uniform random actions: the baseline R8 compares a trained policy against."""

    def __init__(self, seed: int = 0):
        self.rng = np.random.default_rng(seed)

    def predict(self, obs, deterministic: bool = True):
        return self.rng.uniform(-1.0, 1.0, 3).astype(np.float32)


def make_policy(run: dict[str, Any], policy: str = "trained"):
    if policy == "random":
        return RandomPolicy()
    if policy != "trained":
        raise ValueError(f"policy must be 'trained' or 'random', got {policy!r}")
    return load_trainer(run)


def run_episode(env, policy, seed: int, deterministic: bool = True, frames: list | None = None,
                options: dict | None = None):
    obs, info = env.reset(seed=seed, options=options)
    ep = {"seed": seed, "return": 0.0, "length": 0, "switches_on": 0, "nan": False,
          "contradiction": False, "terms": {}}
    if frames is not None:
        frames.append(env.render())
    while True:
        action = np.asarray(policy.predict(obs, deterministic=deterministic), dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        ep["return"] += float(reward)
        ep["length"] += 1
        ep["switches_on"] = max(ep["switches_on"], int(info["switches_on"]))
        ep["nan"] |= bool(info["nan"])
        ep["contradiction"] |= bool(info["contradiction"])
        for k, v in info["reward_terms"].items():
            ep["terms"][k] = ep["terms"].get(k, 0.0) + float(v)
        if frames is not None:
            frames.append(env.render())
        if terminated or truncated:
            break
    ep.update(success=bool(info["success"]), fallen=bool(info["fallen"]), truncated=bool(truncated),
              stage=info["stage"], target=info["target"], seconds=ep["length"] / env.hz)
    if "segment" in info:
        ep.update(segment=int(info["segment"]), segment_success=bool(info["segment_success"]))
    if info.get("difficulty") is not None:
        ep["difficulty"] = int(info["difficulty"])
    return ep


Job = tuple[int, "dict | None"]      # (env seed, reset options)


def _worker(run_dir: str | None, jobs: list[Job], policy: str, deterministic: bool,
            onnx: str | None = None, task: str | None = None) -> list[dict]:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    from .env import PuzzleEnv

    if onnx:                        # no run folder, no pickle (community spec R5)
        from .onnx_policy import OnnxPolicy

        pol = OnnxPolicy(onnx)
        env = PuzzleEnv(task=task)
    else:
        run = load_run(run_dir)
        pol = make_policy(run, policy)
        env = PuzzleEnv(task=run["task"])
    try:
        return [run_episode(env, pol, s, deterministic, options=o) for s, o in jobs]
    finally:
        env.close()


def run_jobs(run_dir: str | Path | None, jobs: list[Job], policy: str = "trained",
             deterministic: bool = True, workers: int = DEFAULT_WORKERS,
             onnx: str | Path | None = None, task: str | Path | None = None) -> list[dict]:
    """Episodes for (seed, reset options) jobs, in job order, over worker processes.
    With `onnx` (and `task`), the policy is that ONNX file instead of the run's checkpoint."""
    jobs = list(jobs)
    workers = max(1, min(workers, len(jobs)))
    rd = str(run_dir) if run_dir is not None else None
    onnx, task = (str(onnx) if onnx else None), (str(task) if task else None)
    if workers == 1:
        return _worker(rd, jobs, policy, deterministic, onnx, task)
    idx = [list(range(i, len(jobs), workers)) for i in range(workers)]
    with ProcessPoolExecutor(workers, mp_context=mp.get_context("spawn")) as pool:
        parts = list(pool.map(_worker, [rd] * workers, [[jobs[j] for j in ix] for ix in idx],
                              [policy] * workers, [deterministic] * workers,
                              [onnx] * workers, [task] * workers))
    out: list[dict] = [{}] * len(jobs)
    for ix, part in zip(idx, parts):
        for j, ep in zip(ix, part):
            out[j] = ep
    return out


def run_episodes(run_dir: str | Path, seeds: list[int], policy: str = "trained",
                 deterministic: bool = True, workers: int = DEFAULT_WORKERS) -> list[dict]:
    eps = run_jobs(run_dir, [(s, None) for s in seeds], policy, deterministic, workers)
    return sorted(eps, key=lambda e: e["seed"])


def evaluate(run_dir: str | Path, episodes: int = 100, policy: str = "trained",
             deterministic: bool = True, workers: int = DEFAULT_WORKERS,
             full: bool = False, segments: bool = False,
             difficulty: int | None = None) -> dict[str, Any]:
    run = load_run(run_dir)
    seeds = list(range(EVAL_SEED_BASE, EVAL_SEED_BASE + episodes))
    segs = _segments(run) if segments else []
    opts = {} if difficulty is None else {"difficulty": int(difficulty)}
    jobs: list[Job] = [(s, dict(opts) or None) for s in seeds]
    jobs += [(s, {"segment": i, **opts}) for i in range(len(segs)) for s in seeds]
    done = run_jobs(run_dir, jobs, policy, deterministic, workers)
    eps = done[:episodes]
    hz = float(run["task"]["env"]["hz"])
    stats = window_stats(eps, hz)
    out = {"run_id": run["summary"]["run_id"], "policy": policy,
           **_report(eps, seeds, hz, deterministic)}
    if eps and "difficulty" in eps[0]:
        out["difficulty"] = eps[0]["difficulty"]
    if full:
        out["full_success_rate"] = stats["success_rate"]
    if segs:
        by_seg = [done[episodes * (i + 1):episodes * (i + 2)] for i in range(len(segs))]
        out["segment_success_rates"] = {
            s["name"]: round(sum(e["segment_success"] for e in seg_eps) / episodes, 3)
            for s, seg_eps in zip(segs, by_seg)}
        if full:
            all_ok = [all(seg_eps[k]["segment_success"] for seg_eps in by_seg) for k in range(episodes)]
            gap = sum(ok and not e["success"] for ok, e in zip(all_ok, eps))
            out["chain_gap_rate"] = round(gap / episodes, 3)
    return out


def _report(eps: list[dict], seeds: list[int], hz: float, deterministic: bool) -> dict[str, Any]:
    stats = window_stats(eps, hz)
    wins = [e["seconds"] for e in eps if e["success"]]
    return {
        "deterministic": deterministic,
        "episodes": len(eps),
        "seeds": [seeds[0], seeds[-1]],
        "success_rate": stats["success_rate"],
        "mean_time_s": round(float(np.mean(wins)), 2) if wins else None,
        "mean_return": stats["mean_return"],
        "fall_rate": stats["fall_rate"],
        "switch_rate": stats["switch_rate"],
        "nan_episodes": stats["nan_episodes"],
        "failed_seeds": [e["seed"] for e in eps if not e["success"]][:20],
    }


def evaluate_onnx(onnx: str | Path, task: str | Path, episodes: int = 100,
                  workers: int = DEFAULT_WORKERS) -> dict[str, Any]:
    """`mpg eval --onnx policy.onnx --task task.yaml`: held-out layouts, no pickle (R5)."""
    from .env import PuzzleEnv
    from .onnx_policy import OnnxPolicy, fit_report

    pol = OnnxPolicy(onnx)
    env = PuzzleEnv(task=task)
    try:
        warnings = fit_report(pol.meta, env)
        hz = float(env.hz)
    finally:
        env.close()
    seeds = list(range(EVAL_SEED_BASE, EVAL_SEED_BASE + episodes))
    eps = run_jobs(None, [(s, None) for s in seeds], workers=workers, onnx=onnx, task=task)
    out = {"onnx": str(onnx), "policy_room": pol.meta["room"], "algo": pol.meta["algo"],
           **_report(eps, seeds, hz, True)}
    if warnings:
        out["warnings"] = warnings
    return out


def _segments(run: dict[str, Any]) -> list[dict[str, Any]]:
    """The run room's segments; checks the flag snapshots load (fails before the workers)."""
    from .curriculum import CurriculumError, load_snapshots
    from .env import PuzzleEnv

    env = PuzzleEnv(task=run["task"])
    try:
        if len(env.segments) < 2:
            raise ValueError(f"--segments needs checkpoint flags in the room "
                             f"({run['task']['room']} has none)")
        try:
            load_snapshots(env)
        except CurriculumError as exc:
            raise ValueError(str(exc)) from None
        return env.segments
    finally:
        env.close()


# ---- video ----------------------------------------------------------------------
def write_video(frames: list[np.ndarray], path: Path, fps: float) -> Path:
    import imageio.v2 as imageio

    path.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(path, fps=fps, codec="libx264", macro_block_size=16) as w:
        for f in frames:
            w.append_data(f)
    return path


def record(run: dict[str, Any], policy, seed: int, path: Path, deterministic: bool = True):
    """Replay one episode with rendering; returns (episode record, mp4 path)."""
    from .env import PuzzleEnv

    env = PuzzleEnv(task=run["task"], render_mode="rgb_array")
    try:
        frames: list[np.ndarray] = []
        ep = run_episode(env, policy, seed, deterministic, frames)
        write_video(frames, path, fps=env.hz)
    finally:
        env.close()
    return ep, path


def render(run_dir: str | Path, out: str | Path | None = None, seed: int | None = None,
           tries: int = 20) -> dict[str, Any]:
    """MP4 of a success episode (first success among eval seeds), or of `seed`."""
    from .env import PuzzleEnv

    run = load_run(run_dir)
    policy = make_policy(run)
    path = Path(out) if out else Path(run_dir) / "success.mp4"
    if seed is None:
        env = PuzzleEnv(task=run["task"])
        try:
            for s in range(EVAL_SEED_BASE, EVAL_SEED_BASE + tries):
                if run_episode(env, policy, s)["success"]:
                    seed = s
                    break
        finally:
            env.close()
        if seed is None:
            return {"ok": False, "error": f"no success in {tries} eval episodes "
                                          f"(seeds {EVAL_SEED_BASE}..{EVAL_SEED_BASE + tries - 1}); "
                                          "run `mpg diagnose` or pass --seed to render one anyway"}
    ep, path = record(run, policy, seed, path)
    return {"run_id": run["summary"]["run_id"], "mp4": str(path.resolve()),
            "bytes": path.stat().st_size, "seed": seed, "success": ep["success"],
            "seconds": ep["seconds"], "frames": ep["length"] + 1}
