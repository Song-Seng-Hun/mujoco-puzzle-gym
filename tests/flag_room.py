"""Test helpers for R21/R22: a room with two checkpoint flags on a straight line,
their snapshots, and fake policies.

    spawn (-1.0, -1.0) facing +x  (walking straight ahead never passes a flag)
    f1 (-0.5, 0.0) -> f2 (-0.1, 0.0) -> goal (0.3, 0.0), all on y = 0

From f1's snapshot (body on f1 facing +x) walking straight ahead reaches f2,
and from f2's the goal, so a "walk forward" policy succeeds on segments
f1->f2 and f2->goal but not on start->f1 or the full run.
"""

from __future__ import annotations

import pickle

import numpy as np
import yaml

from mujoco_puzzle_gym.env import PuzzleEnv

STAND = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
FORWARD = np.array([1.0, 0.0, 0.0], dtype=np.float32)

ROOM = {
    "name": "two_flags", "size": [3.0, 3.0],
    "spawn": {"pos": [-1.0, -1.0], "yaw": 0.0},
    "parts": [
        {"name": "f1", "type": "checkpoint", "order": 1, "pos": [-0.5, 0.0]},
        {"name": "f2", "type": "checkpoint", "order": 2, "pos": [-0.1, 0.0]},
        {"name": "goal", "type": "goal", "pos": [0.3, 0.0], "radius": 0.1},
    ],
}
FLAGS = ("f1", "f2")


def write_room(tmp_path):
    path = tmp_path / "two_flags.yaml"
    path.write_text(yaml.safe_dump(ROOM))
    return path


def task_dict(room, snap_dir, **extra):
    return {"room": str(room), "env": {"hz": 10, "episode_seconds": 6.0},
            "obs": ["gravity", "ang_vel", "lin_vel", "part_pos", "part_state", "checkpoints"],
            "curriculum": {"snapshots": str(snap_dir)}, **extra}


def save_snapshots(env, snap_dir):
    """Teleport onto each flag facing +x, step once, pickle the snapshot as
    `mpg check` does; returns {flag: snapshot}."""
    snap_dir.mkdir(parents=True, exist_ok=True)
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    out = {}
    for f in FLAGS:
        env.body.teleport(env.data, (*env.parts[f].xy, 0.0))
        env.body_state = env.body.state(env.data)
        env.step(STAND)
        snap = env.checkpoint_snap
        assert snap.meta["checkpoint"] == f
        (snap_dir / f"{f}.pkl").write_bytes(pickle.dumps(snap))
        out[f] = snap
    return out


def flag_env(tmp_path, **extra):
    """(env, snapshots) for the two-flag room with its snapshots in tmp_path/snaps."""
    room = write_room(tmp_path)
    snap_dir = tmp_path / "snaps"
    env = PuzzleEnv(task=task_dict(room, snap_dir, **extra))
    snaps = save_snapshots(env, snap_dir)
    return env, snaps
