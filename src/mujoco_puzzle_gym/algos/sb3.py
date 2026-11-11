"""Stable-Baselines3 algorithms behind the `Trainer` interface (R8): ppo (default), sac, a2c.

task.yaml `algo_kwargs` go to the SB3 constructor as they are; `net` sets the
hidden layers of the policy (and value / Q networks).

Policy class (R39): a Box observation gets `MlpPolicy`; a Dict observation
(camera `image`) gets `MultiInputPolicy` (SB3's CNN feature extractor on the
image); a Dict with a `critic` key (task `critic_obs`) gets `AsymmetricPolicy`
(ppo, a2c only): the actor reads every key but `critic`, the value network
reads only `critic`.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, ClassVar

import numpy as np
import torch
from gymnasium import spaces
from stable_baselines3 import A2C, PPO, SAC
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.policies import MultiInputActorCriticPolicy
from stable_baselines3.common.preprocessing import get_flattened_obs_dim
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor, CombinedExtractor, MlpExtractor

from ..registry import register_algo
from ..task import TaskError
from .base import Deadline, Trainer

UNBOUNDED_STEPS = 10**12
MAX_TORCH_THREADS = 4
CRITIC_KEY = "critic"


# ---- asymmetric actor-critic (R39 `critic_obs`) -----------------------------------
class SplitExtractor(BaseFeaturesExtractor):
    """Features = [actor features (every key but `critic`, CNN on images) | critic vector]."""

    def __init__(self, observation_space: spaces.Dict, cnn_output_dim: int = 256,
                 normalized_image: bool = False):
        actor_space = spaces.Dict({k: v for k, v in observation_space.spaces.items() if k != CRITIC_KEY})
        actor = CombinedExtractor(actor_space, cnn_output_dim=cnn_output_dim,
                                  normalized_image=normalized_image)
        critic_dim = get_flattened_obs_dim(observation_space[CRITIC_KEY])
        super().__init__(observation_space, actor.features_dim + critic_dim)
        self.actor = actor
        self.actor_dim = actor.features_dim

    def forward(self, observations):
        actor = self.actor({k: v for k, v in observations.items() if k != CRITIC_KEY})
        return torch.cat([actor, torch.flatten(observations[CRITIC_KEY], start_dim=1)], dim=1)


class SplitMlpExtractor(torch.nn.Module):
    """Policy MLP on the actor slice of the features, value MLP on the critic slice."""

    def __init__(self, actor_dim: int, critic_dim: int, net_arch, activation_fn, device):
        super().__init__()
        self.actor_dim = actor_dim
        pi = MlpExtractor(actor_dim, net_arch, activation_fn, device)
        vf = MlpExtractor(critic_dim, net_arch, activation_fn, device)
        self.policy_net, self.latent_dim_pi = pi.policy_net, pi.latent_dim_pi
        self.value_net, self.latent_dim_vf = vf.value_net, vf.latent_dim_vf

    def forward(self, features):
        return self.forward_actor(features), self.forward_critic(features)

    def forward_actor(self, features):
        return self.policy_net(features[:, :self.actor_dim])

    def forward_critic(self, features):
        return self.value_net(features[:, self.actor_dim:])


class AsymmetricPolicy(MultiInputActorCriticPolicy):
    """Actor sees the task's `obs`, the value function sees only `critic_obs` (R39)."""

    def __init__(self, *args, **kwargs):
        kwargs["features_extractor_class"] = SplitExtractor
        kwargs["share_features_extractor"] = True
        super().__init__(*args, **kwargs)

    def _build_mlp_extractor(self) -> None:
        actor_dim = self.features_extractor.actor_dim
        self.mlp_extractor = SplitMlpExtractor(actor_dim, self.features_dim - actor_dim,
                                               self.net_arch, self.activation_fn, self.device)


def policy_for(observation_space, algo: str):
    """SB3 policy for an observation space (see the module docstring)."""
    if not isinstance(observation_space, spaces.Dict):
        return "MlpPolicy"
    if CRITIC_KEY in observation_space.spaces:
        if algo not in ("ppo", "a2c"):
            raise TaskError(f"`critic_obs` needs an actor-critic algorithm (ppo, a2c), not {algo!r}")
        return AsymmetricPolicy
    return "MultiInputPolicy"


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
        policy = policy_for(env.observation_space, self.name)
        self.model = self.sb3_class(policy, env, seed=seed, device="cpu", verbose=0,
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
