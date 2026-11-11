"""`Trainer`: the small interface every training algorithm implements (R8).

`mpg train` builds the parallel envs, hands them to a Trainer, and saves the
checkpoint; `mpg eval`, `diagnose` and `render` load it back and call
`predict`. Episode logging (curves, per-term rewards) happens in a wrapper
around the envs, so a Trainer only has to step them.

The env given to `__init__` is a Stable-Baselines3 `VecEnv` (n parallel
PuzzleEnvs; may be None when loading only to predict):

    obs = env.reset()                                  # (n, obs_dim)
    obs, rewards, dones, infos = env.step(actions)     # actions (n, 3) in [-1, 1]
    env.num_envs, env.observation_space, env.action_space

Finished episodes reset automatically. A user algorithm subclasses Trainer and
registers through the entry point group `mujoco_puzzle_gym.algos` (or
`registry.register_algo` from a task.yaml `plugins:` module).
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, ClassVar

import numpy as np


class Trainer(ABC):
    name: ClassVar[str] = ""

    def __init__(self, env: Any, task: dict[str, Any], seed: int = 0):
        self.env = env
        self.task = task
        self.seed = seed

    @abstractmethod
    def train(self, seconds: float, max_steps: int | None = None) -> int:
        """Train for at most `seconds` of wall time (and `max_steps` env steps if
        given); return the env steps taken (summed over parallel envs)."""

    @abstractmethod
    def save(self, path: Path) -> Path:
        """Save to `path` (a file stem inside the run directory); return the file written."""

    @classmethod
    @abstractmethod
    def load(cls, path: Path, task: dict[str, Any], env: Any = None) -> "Trainer":
        """Load what `save` wrote (the returned path)."""

    @abstractmethod
    def predict(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        """Action(s) in [-1, 1] for one observation (obs_dim,) or a batch (n, obs_dim)."""


class Deadline:
    """Wall-clock and step budget shared by trainers."""

    def __init__(self, seconds: float, max_steps: int | None = None):
        self.end = time.monotonic() + seconds
        self.max_steps = max_steps

    def done(self, steps: int = 0) -> bool:
        return time.monotonic() >= self.end or (self.max_steps is not None and steps >= self.max_steps)
