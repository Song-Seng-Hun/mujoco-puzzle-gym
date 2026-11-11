"""Stable-Baselines3 algorithms behind the `Trainer` interface (R8): ppo (default), sac, a2c.

task.yaml `algo_kwargs` go to the SB3 constructor as they are; `net` sets the
hidden layers of the policy (and value / Q networks).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, ClassVar

import numpy as np
import torch
from stable_baselines3 import A2C, PPO, SAC
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import BaseCallback

from ..registry import register_algo
from .base import Deadline, Trainer

UNBOUNDED_STEPS = 10**12
MAX_TORCH_THREADS = 4


class _StopAtDeadline(BaseCallback):
    def __init__(self, deadline: Deadline):
        super().__init__()
        self.deadline = deadline

    def _on_step(self) -> bool:
        return not self.deadline.done(self.num_timesteps)


class SB3Trainer(Trainer):
    sb3_class: ClassVar[type[BaseAlgorithm]]
    shared_net: ClassVar[bool] = False     # on-policy: separate pi / vf layers of size `net`

    def __init__(self, env, task: dict[str, Any], seed: int = 0, model: BaseAlgorithm | None = None):
        super().__init__(env, task, seed)
        if model is not None:
            self.model = model
            return
        n_envs = getattr(env, "num_envs", 1)
        torch.set_num_threads(max(1, min(MAX_TORCH_THREADS, (os.cpu_count() or 4) - n_envs)))
        kwargs = dict(task.get("algo_kwargs") or {})
        net = list(task.get("net") or [64, 64])
        policy_kwargs = dict(kwargs.pop("policy_kwargs", {}) or {})
        policy_kwargs.setdefault("net_arch", net if self.shared_net else {"pi": net, "vf": net})
        self.model = self.sb3_class("MlpPolicy", env, seed=seed, device="cpu", verbose=0,
                                    policy_kwargs=policy_kwargs, **kwargs)

    def train(self, seconds: float, max_steps: int | None = None) -> int:
        deadline = Deadline(seconds, max_steps)
        self.model.learn(total_timesteps=max_steps or UNBOUNDED_STEPS,
                         callback=_StopAtDeadline(deadline), progress_bar=False)
        return int(self.model.num_timesteps)

    def save(self, path: Path) -> Path:
        path = Path(path).with_suffix(".zip")
        self.model.save(path)
        return path

    @classmethod
    def load(cls, path: Path, task: dict[str, Any], env=None) -> "SB3Trainer":
        torch.set_num_threads(1)
        model = cls.sb3_class.load(Path(path), env=env, device="cpu")
        return cls(env, task, model=model)

    def predict(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        action, _ = self.model.predict(obs, deterministic=deterministic)
        return action


@register_algo("ppo", replace=True)
class PPOTrainer(SB3Trainer):
    sb3_class = PPO


@register_algo("a2c", replace=True)
class A2CTrainer(SB3Trainer):
    sb3_class = A2C


@register_algo("sac", replace=True)
class SACTrainer(SB3Trainer):
    sb3_class = SAC
    shared_net = True      # SAC's net_arch is one list for actor and critics
