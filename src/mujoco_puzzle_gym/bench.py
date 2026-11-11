"""Speed benchmark (R13, walker-only for now).

Runs the default body on a flat floor with a constant walk command and counts
steps. One high-level env step (10 Hz) = 5 body steps (50 Hz) = 20 physics
steps (5 ms), so high-level steps/s is estimated as body steps/s / 5; puzzle
logic and the training algorithm are not included yet.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import platform
import subprocess
import time
from typing import Any

import mujoco

from .bodies import flat_floor_spec, make_body

HIGH_LEVEL_HZ = 10.0
BENCH_CMD = (0.3, 0.0, 0.0)


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


def run(seconds: float = 5.0, procs: list[int] | None = None) -> dict[str, Any]:
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
    result["high_level_steps_in_2h"] = int(best * 7200)
    return result
