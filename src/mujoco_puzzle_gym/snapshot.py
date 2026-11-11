"""Checkpoint snapshots (R6): save and restore everything an episode depends on.

A snapshot holds
- the MuJoCo state from `mj_getState(..., mjSTATE_INTEGRATION)`: time, qpos,
  qvel, act, warm start, ctrl, applied forces, `eq_active` (beak welds and rope
  connections stay as they were) and mocap poses. `PHYSICS` / `FULLPHYSICS`
  leave `eq_active` out, so restoring them would drop a held object;
- model values that `mj_getState` does not hold but parts or setters change
  while running (`MODEL_FIELDS`: part placement, colours, friction, collision
  flags, constraint data);
- every part's `save_state()` and position, the signal values, and the body
  adapter's controller history (`history_state`).

`restore` writes them back in that order (state, model values, signals, parts,
body history) and runs `mj_forward`. Parts restore after the signals, so a part
that keeps its value across restores (counter `persist`) can write it again.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import mujoco
import numpy as np

if TYPE_CHECKING:
    from .bodies import BodyAdapter
    from .signals import SignalBus

SIG = mujoco.mjtState.mjSTATE_INTEGRATION
#: Model arrays that parts, setters (R13) and difficulty steps (R21) may change while running.
MODEL_FIELDS = ("body_pos", "body_quat", "geom_rgba", "geom_friction", "geom_contype",
                "geom_conaffinity", "eq_data")


@dataclass
class Snapshot:
    state: np.ndarray
    model: dict[str, np.ndarray]
    parts: dict[str, dict[str, Any]]
    part_xy: dict[str, np.ndarray]
    signals: dict[str, float]
    body: dict[str, Any]
    meta: dict[str, Any] = field(default_factory=dict)


def model_values(model: mujoco.MjModel) -> dict[str, np.ndarray]:
    return {f: getattr(model, f).copy() for f in MODEL_FIELDS}


def load_model_values(model: mujoco.MjModel, values: dict[str, np.ndarray]) -> None:
    for f, v in values.items():
        getattr(model, f)[...] = v


def save(model: mujoco.MjModel, data: mujoco.MjData, parts: dict[str, Any], bus: "SignalBus",
         body: "BodyAdapter", **meta: Any) -> Snapshot:
    state = np.empty(mujoco.mj_stateSize(model, SIG))
    mujoco.mj_getState(model, data, state, SIG)
    return Snapshot(
        state=state,
        model=model_values(model),
        parts={n: copy.deepcopy(p.save_state()) for n, p in parts.items()},
        part_xy={n: np.array(p.xy, dtype=float) for n, p in parts.items()},
        signals=bus.snapshot(),
        body=copy.deepcopy(body.history_state()),
        meta=copy.deepcopy(meta),
    )


def restore(snap: Snapshot, model: mujoco.MjModel, data: mujoco.MjData, parts: dict[str, Any],
            bus: "SignalBus", body: "BodyAdapter") -> None:
    mujoco.mj_setState(model, data, snap.state, SIG)
    load_model_values(model, snap.model)
    bus.load(snap.signals)
    for n, p in parts.items():
        p.xy = snap.part_xy[n].copy()
        p.load_state(model, data, copy.deepcopy(snap.parts[n]))
    body.load_history(data, copy.deepcopy(snap.body))
    mujoco.mj_forward(model, data)
