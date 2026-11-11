"""`mpg eval runs/<id> --episodes 100` (R10) and `mpg render runs/<id>` (R11).

Eval runs the saved policy on held-out layouts: episode k uses env seed
EVAL_SEED_BASE + k, a range training never touches (see train.py). Episodes
run in parallel worker processes, each loading the checkpoint itself.

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


def run_episode(env, policy, seed: int, deterministic: bool = True, frames: list | None = None):
    obs, info = env.reset(seed=seed)
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
    return ep


def _worker(run_dir: str, seeds: list[int], policy: str, deterministic: bool) -> list[dict]:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    from .env import PuzzleEnv

    run = load_run(run_dir)
    pol = make_policy(run, policy)
    env = PuzzleEnv(task=run["task"])
    try:
        return [run_episode(env, pol, s, deterministic) for s in seeds]
    finally:
        env.close()


def run_episodes(run_dir: str | Path, seeds: list[int], policy: str = "trained",
                 deterministic: bool = True, workers: int = DEFAULT_WORKERS) -> list[dict]:
    workers = max(1, min(workers, len(seeds)))
    if workers == 1:
        return _worker(str(run_dir), list(seeds), policy, deterministic)
    chunks = [list(seeds[i::workers]) for i in range(workers)]
    with ProcessPoolExecutor(workers, mp_context=mp.get_context("spawn")) as pool:
        parts = pool.map(_worker, [str(run_dir)] * workers, chunks, [policy] * workers,
                         [deterministic] * workers)
        eps = [e for part in parts for e in part]
    return sorted(eps, key=lambda e: e["seed"])


def evaluate(run_dir: str | Path, episodes: int = 100, policy: str = "trained",
             deterministic: bool = True, workers: int = DEFAULT_WORKERS) -> dict[str, Any]:
    run = load_run(run_dir)
    seeds = list(range(EVAL_SEED_BASE, EVAL_SEED_BASE + episodes))
    eps = run_episodes(run_dir, seeds, policy, deterministic, workers)
    hz = float(run["task"]["env"]["hz"])
    stats = window_stats(eps, hz)
    wins = [e["seconds"] for e in eps if e["success"]]
    return {
        "run_id": run["summary"]["run_id"],
        "policy": policy,
        "deterministic": deterministic,
        "episodes": episodes,
        "seeds": [seeds[0], seeds[-1]],
        "success_rate": stats["success_rate"],
        "mean_time_s": round(float(np.mean(wins)), 2) if wins else None,
        "mean_return": stats["mean_return"],
        "fall_rate": stats["fall_rate"],
        "switch_rate": stats["switch_rate"],
        "nan_episodes": stats["nan_episodes"],
        "failed_seeds": [e["seed"] for e in eps if not e["success"]][:20],
    }


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
