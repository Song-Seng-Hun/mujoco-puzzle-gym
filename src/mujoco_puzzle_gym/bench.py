"""Speed benchmark (R13).

Two measurements:
- walker: the default body alone on a flat floor with a constant walk command.
  One high-level env step (10 Hz) = 5 body steps (50 Hz) = 20 physics steps
  (5 ms), so high-level steps/s is estimated as body steps/s / 5.
- env: the real `PuzzleEnv` on a room (default: examples/switch_door.yaml) with
  uniform random actions, auto-reset on episode end; single process and N
  worker processes through gymnasium's AsyncVectorEnv. This includes walls,
  doors, part logic, obs and rewards and inter-process transfer, but not the
  learner's own compute (PPO updates share the same CPU).
- camera (R39, OQ11): the same env runs with a 64x64 head `camera` item added
  to the default obs, once per mode (`rgb`, `depth`), single and parallel; the
  camera renders once per env step. `--no-camera` skips it.
"""

from __future__ import annotations

import functools
import multiprocessing as mp
import os
import platform
import subprocess
import time
from pathlib import Path
from typing import Any

import mujoco

from .bodies import flat_floor_spec, make_body

HIGH_LEVEL_HZ = 10.0
BENCH_CMD = (0.3, 0.0, 0.0)
DEFAULT_ROOM = Path(__file__).resolve().parents[2] / "examples" / "switch_door.yaml"
TWO_HOURS = 7200
CAMERA_MODES = ("rgb", "depth")
CAMERA_SIZE = [64, 64]


def _make():
    body = make_body()
    model = body.spawn(flat_floor_spec()).compile()
    data = mujoco.MjData(model)
    body.reset(data)
    return body, model, data


def _run_body(body, data, seconds: float) -> tuple[int, float, int]:
    """Body steps done in `seconds` of wall time; resets on a fall."""
    steps = falls = 0
    t0 = time.perf_counter()
    end = t0 + seconds
    while True:
        for _ in range(25):
            body.step(data, BENCH_CMD)
            steps += 1
        if body.state(data).fallen:
            falls += 1
            body.reset(data)
        now = time.perf_counter()
        if now >= end:
            return steps, now - t0, falls


def _worker(seconds: float, ready, start, out) -> None:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    body, _, data = _make()
    _run_body(body, data, 0.2)          # warm-up
    body.reset(data)
    ready.put(1)
    start.wait()
    out.put(_run_body(body, data, seconds))


def run_parallel(n: int, seconds: float) -> dict[str, Any]:
    ctx = mp.get_context("spawn")
    ready, out, start = ctx.Queue(), ctx.Queue(), ctx.Event()
    procs = [ctx.Process(target=_worker, args=(seconds, ready, start, out)) for _ in range(n)]
    for p in procs:
        p.start()
    for _ in procs:
        ready.get(timeout=300)
    start.set()
    results = [out.get(timeout=seconds + 300) for _ in procs]
    for p in procs:
        p.join()
    steps = sum(r[0] for r in results)
    wall = max(r[1] for r in results)
    body_sps = steps / wall
    return {
        "procs": n,
        "body_steps_per_s": round(body_sps, 1),
        "physics_steps_per_s": round(body_sps * 4, 1),
        "high_level_steps_per_s": round(body_sps / 5, 1),
        "falls": sum(r[2] for r in results),
    }


def machine() -> dict[str, Any]:
    info: dict[str, Any] = {"platform": platform.platform(), "cpu_count": os.cpu_count()}
    if platform.system() == "Darwin":
        for key, name in (("cpu", "machdep.cpu.brand_string"), ("perf_cores", "hw.perflevel0.physicalcpu"),
                          ("eff_cores", "hw.perflevel1.physicalcpu"), ("model", "hw.model")):
            try:
                info[key] = subprocess.run(["sysctl", "-n", name], capture_output=True,
                                           text=True, check=True).stdout.strip()
            except (OSError, subprocess.CalledProcessError):
                pass
    return info


def _make_env(room: str, obs: list | None = None):
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    from .env import PuzzleEnv
    return PuzzleEnv(task={"obs": obs} if obs is not None else None, room=room)


def camera_obs(mode: str) -> list:
    from .task import DEFAULTS
    return [*DEFAULTS["obs"], {"item": "camera", "mode": mode, "size": list(CAMERA_SIZE)}]


def env_single(room: str, seconds: float, obs: list | None = None) -> dict[str, Any]:
    import numpy as np

    env = _make_env(room, obs)
    rng = np.random.default_rng(0)
    env.reset(seed=0)
    steps = episodes = 0
    t0 = time.perf_counter()
    end = t0 + seconds
    while True:
        _, _, term, trunc, _ = env.step(rng.uniform(-1, 1, 3).astype(np.float32))
        steps += 1
        if term or trunc:
            episodes += 1
            env.reset()
        if steps % 10 == 0 and time.perf_counter() >= end:
            break
    wall = time.perf_counter() - t0
    env.close()
    return {"steps_per_s": round(steps / wall, 1), "episodes_ended": episodes}


def env_parallel(room: str, n: int, seconds: float, obs: list | None = None) -> dict[str, Any]:
    import numpy as np
    from gymnasium.vector import AsyncVectorEnv

    venv = AsyncVectorEnv([functools.partial(_make_env, room, obs) for _ in range(n)], context="spawn")
    try:
        venv.reset(seed=0)
        rng = np.random.default_rng(0)
        for _ in range(5):        # warm-up
            venv.step(rng.uniform(-1, 1, (n, 3)).astype(np.float32))
        vec_steps = 0
        t0 = time.perf_counter()
        end = t0 + seconds
        while time.perf_counter() < end:
            venv.step(rng.uniform(-1, 1, (n, 3)).astype(np.float32))
            vec_steps += 1
        wall = time.perf_counter() - t0
    finally:
        venv.close()
    return {"procs": n, "steps_per_s": round(vec_steps * n / wall, 1)}


def run_env(room: str | Path, seconds: float, procs: list[int], camera: bool = True) -> dict[str, Any]:
    room = str(Path(room).resolve())
    single = env_single(room, seconds)
    parallel = [env_parallel(room, n, seconds) for n in procs]
    best = max([single["steps_per_s"]] + [p["steps_per_s"] for p in parallel])
    out = {"room": room, "actions": "uniform random", "single": single, "parallel": parallel,
           "best_steps_per_s": best, "steps_in_2h": int(best * TWO_HOURS)}
    if camera:
        out["camera"] = {"size": list(CAMERA_SIZE), "renders": "once per env step"}
        for mode in CAMERA_MODES:
            obs = camera_obs(mode)
            cs = env_single(room, seconds, obs)
            cp = [env_parallel(room, n, seconds, obs) for n in procs]
            cbest = max([cs["steps_per_s"]] + [p["steps_per_s"] for p in cp])
            out["camera"][mode] = {"single": cs, "parallel": cp, "best_steps_per_s": cbest,
                                   "vs_privileged": round(cbest / best, 3)}
    return out


def run(seconds: float = 5.0, procs: list[int] | None = None,
        room: str | Path | None = DEFAULT_ROOM, camera: bool = True) -> dict[str, Any]:
    body, model, data = _make()
    control_dt = body.control_dt
    physics_per_body_step = round(control_dt / model.opt.timestep)
    body_per_high = round(1.0 / HIGH_LEVEL_HZ / control_dt)
    steps, wall, falls = _run_body(body, data, seconds)
    single = steps / wall
    result: dict[str, Any] = {
        "machine": machine(),
        "seconds": seconds,
        "command": list(BENCH_CMD),
        "physics_steps_per_body_step": physics_per_body_step,
        "body_steps_per_high_level_step": body_per_high,
        "single": {
            "body_steps_per_s": round(single, 1),
            "physics_steps_per_s": round(single * physics_per_body_step, 1),
            "high_level_steps_per_s": round(single / body_per_high, 1),
            "falls": falls,
        },
        "parallel": [run_parallel(n, seconds) for n in (procs or [])],
    }
    best = max([result["single"]["high_level_steps_per_s"]]
               + [p["high_level_steps_per_s"] for p in result["parallel"]])
    result["best_high_level_steps_per_s"] = best
    result["high_level_steps_in_2h"] = int(best * TWO_HOURS)
    if room is not None:
        result["env"] = run_env(room, seconds, procs or [], camera=camera)
    return result
