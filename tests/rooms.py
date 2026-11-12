"""Test helpers: an open room built from a part list, and a test-only free box.

The box stands in for the cube (R17) in sensor tests: a free body of kind
"object" with a given size and mass and no handles.
"""

from __future__ import annotations

import mujoco
import numpy as np
import yaml

from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.registry import register_part
from mujoco_puzzle_gym.schema import Field


class FreeBox(Part):
    kind = "object"
    object_kind = "box"
    stage = "device"
    fields = (
        Field("half", "float", 0.03, unit="m", min=0.005, max=0.2),
        Field("mass", "float", 0.05, unit="kg", min=0.001, max=10.0),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = float(self.cfg["half"])
        self.mass = float(self.cfg["mass"])

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, self.half])
        body.add_freejoint(name=self.body_name + "/free")
        body.add_geom(name=self.body_name + "/geom", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[self.half] * 3, mass=self.mass, rgba=(0.9, 0.6, 0.1, 1.0))
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.body_jntadr[self._bid]
        self._q = int(model.jnt_qposadr[j])
        self._v = int(model.jnt_dofadr[j])

    def place(self, model, xy):
        self.xy = np.asarray(xy, dtype=float)

    def reset(self, model, data):
        self.put(data, [*self.xy, self.half])

    def put(self, data, xyz, yaw: float = 0.0) -> None:
        """Move the box (call mj_forward afterwards)."""
        data.qpos[self._q:self._q + 3] = xyz
        data.qpos[self._q + 3:self._q + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        data.qvel[self._v:self._v + 6] = 0.0

    def pos(self, data) -> np.ndarray:
        return data.xpos[self._bid].copy()


register_part("t_box", FreeBox, replace=True)


def open_room(tmp_path, parts, spawn=(-1.0, -1.0), size=(3.0, 3.0), task=None, name="room"):
    """An env in a wall-less room with `parts` plus a far goal."""
    raw = {"name": name, "size": list(size), "spawn": {"pos": list(spawn), "yaw": 0.0},
           "parts": list(parts) + [{"name": "goal", "type": "goal", "pos": [1.3, 1.3], "radius": 0.1}]}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(task=task, room=path)
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    return env


def stand_at(env, x, y, yaw=0.0):
    """Teleport the body and run one bus step (no physics)."""
    env.body.teleport(env.data, (x, y, yaw))
    tick(env)


def tick(env):
    """One bus step at the current state."""
    mujoco.mj_forward(env.model, env.data)
    env.body_state = env.body.state(env.data)
    env.signals.step(env.model, env.data, env.body_state)


def body_steps(env, seconds, cmd=None, every=None):
    """Body steps for `seconds` as `env.advance` runs them (requests included);
    `every(env)` after each. Returns True if the body fell at some step."""
    cmd = np.zeros(3) if cmd is None else cmd
    fell = False
    for _ in range(int(round(seconds / env.body.control_dt))):
        env.body.step(env.data, cmd, on_substep=env._on_substep)
        env.signals.step(env.model, env.data, env.read_body())
        if env.world.requests:
            env._run_requests()
        env.body_state = env.read_body()
        fell |= bool(env.body_state.fallen)
        if every is not None:
            every(env)
    return fell
