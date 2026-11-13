"""R30 roller puzzles: the zone's `min_speed` (R5), the brake strip, the static
warning for steps reachable on wheels, and the `puzzles/roller/` presets.

- a zone with `min_speed` turns on only for a target moving at least that fast
  (averaged speed), and turns off when it slows down inside; without it the
  zone behaves as before;
- a brake strip raises the wheel joints' friction while the body is over it,
  restores it when the body leaves, on reset and on a snapshot restore;
- `mpg check` stage `parts` warns (does not fail) when steps are reachable on
  wheels without a detach pad in the way;
- the six roller presets load and pass the static stages;
- slow: walking only (no `addon` sub-goal), the timed corridor's door closes
  before the body gets there. The presets' and examples' 20-seed `mpg check`
  runs in test_examples.py and test_check2.py.
"""

from pathlib import Path

import numpy as np
import pytest
import yaml
from rooms import open_room, stand_at, tick

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.check import check, check_parts
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

ROOT = Path(__file__).resolve().parents[1]
ROLLER = catalog.PRESET_ROOT / "puzzles/roller"
NAMES = ["speed_gate", "timed_corridor", "mode_switch_route", "brake_strip", "one_way_station",
         "crosswind"]
STATION = {"name": "st", "preset": "device/addon_roller", "pos": [1.0, -1.0]}


@pytest.fixture
def make(asset_dir, tmp_path):
    envs = []

    def _make(parts, **kw):
        env = open_room(tmp_path, parts, **kw)
        envs.append(env)
        return env

    yield _make
    for e in envs:
        e.close()


# ---- zone min_speed -----------------------------------------------------------------
def glide(env, x0, x1, speed, dt=0.02, y=0.0):
    """Teleport the body from x0 to x1 at `speed` [m/s], one bus step per `dt`;
    the zone's value at each step."""
    out = []
    for x in np.arange(x0, x1, speed * dt):
        env.data.time += dt
        stand_at(env, float(x), y)
        out.append(env.signals.get("z"))
    return out


ZONE = {"name": "z", "type": "zone", "pos": [0.0, 0.0], "size": [0.4, 0.4]}


def test_zone_min_speed_counts_fast_targets_only(make):
    env = make([{**ZONE, "min_speed": 0.28}])
    assert max(glide(env, -0.8, 0.6, 0.2)) == 0          # a walking pace never counts
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    vals = glide(env, -0.8, 0.0, 0.45)                   # a rolling pace does
    assert vals[-1] == 1 and vals[0] == 0
    for _ in range(40):                                  # stops inside: off again
        env.data.time += 0.02
        stand_at(env, 0.0, 0.0)
    assert env.signals.get("z") == 0


def test_zone_min_speed_leave_holds_once_on(make):
    env = make([{**ZONE, "min_speed": 0.28, "rearm": "leave"}])
    assert glide(env, -0.8, 0.0, 0.45)[-1] == 1
    for _ in range(40):
        env.data.time += 0.02
        stand_at(env, 0.0, 0.0)
    assert env.signals.get("z") == 1                     # held until it leaves


def test_zone_without_min_speed_unchanged(make):
    env = make([ZONE])
    assert env.parts["z"].min_speed == 0 and not env.parts["z"].stateful
    stand_at(env, 0.0, 0.0)                               # standing still counts
    assert env.signals.get("z") == 1


# ---- brake strip --------------------------------------------------------------------
BRAKE = {"name": "brake", "preset": "trap/brake_strip", "pos": [0.0, 0.0]}


def wheel_friction(env):
    return env.model.dof_frictionloss[env.body.wheel_dofs(env.model)]


def test_brake_strip_on_and_off(make):
    env = make([STATION, BRAKE])
    base = wheel_friction(env).copy()
    assert len(base) == 4 and np.allclose(base, 0.0015)
    stand_at(env, 0.0, 0.1)
    assert np.allclose(wheel_friction(env), 0.01) and env.parts["brake"].state() == 1
    stand_at(env, 0.6, 0.0)                               # left: built values again
    assert np.allclose(wheel_friction(env), base) and env.parts["brake"].state() == 0
    stand_at(env, 0.0, 0.0)
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    assert np.allclose(wheel_friction(env), base)


def test_brake_strip_snapshot_restores_the_brake(make):
    env = make([STATION, BRAKE])
    stand_at(env, 0.0, 0.0)
    snap = env.save_snapshot()
    stand_at(env, 0.6, 0.0)
    assert np.allclose(wheel_friction(env), 0.0015)
    env.restore_snapshot(snap)
    assert np.allclose(wheel_friction(env), 0.01)
    stand_at(env, 0.6, 0.0)
    assert np.allclose(wheel_friction(env), 0.0015)


def test_brake_strip_off_signal_and_two_strips(make):
    env = make([STATION, {**BRAKE, "in": ["sw"]}, {"name": "sw", "type": "switch", "pos": [-1.2, 1.2]},
                {"name": "brake2", "preset": "trap/brake_strip", "pos": [0.25, 0.0], "frictionloss": 0.005}])
    stand_at(env, 0.0, 0.0)                               # `in` off: no brake from it
    assert np.allclose(wheel_friction(env), 0.0015)
    stand_at(env, 0.2, 0.0)                               # on brake2 only
    assert np.allclose(wheel_friction(env), 0.005)


def test_brake_strip_warns_without_wheels(make):
    env = make([BRAKE])
    assert env.body.wheel_dofs(env.model) == []
    res = check_parts(env)
    assert res["ok"] and "no wheels" in res["warnings"][0]


# ---- steps on wheels (static warning) ---------------------------------------------------
def step_room(tmp_path, detach: bool):
    row = "#" + "." * 20 + "1122" + "#"
    raw = {"name": "steps_wheels", "cell": 0.1, "step_height": 0.01,
           "map": "\n".join(["#" * 26] + [row] * 6 + ["#" * 26]),
           "spawn": {"pos": [-1.0, 0.0], "yaw": 0.0},
           "parts": [{"name": "on", "preset": "device/addon_roller", "pos": [-0.6, 0.0], "size": [0.3, 0.58]},
                     {"name": "goal", "type": "goal", "pos": [1.0, 0.0], "radius": 0.08}]
           + ([{"name": "off", "preset": "device/addon_roller", "mode": "detach", "pos": [0.4, 0.0],
                "size": [0.3, 0.58]}] if detach else [])}
    path = tmp_path / f"steps_wheels_{detach}.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


@pytest.mark.parametrize("detach", [False, True])
def test_steps_reachable_on_wheels_warn(asset_dir, tmp_path, detach):
    res = check(step_room(tmp_path, detach), static_only=True)
    assert res["static"]["ok"], res                       # a warning never fails the stage
    warns = res["parts"].get("warnings", [])
    if detach:
        assert not warns
    else:
        assert len(warns) == 1 and "'on'" in warns[0] and "steps" in warns[0]


def test_flat_roller_room_has_no_step_warning(asset_dir):
    res = check(ROOT / "examples/rooms/roller_rink.yaml", static_only=True)
    assert "warnings" not in res["parts"]


# ---- presets ---------------------------------------------------------------------------
def test_roller_presets_listed():
    assert {f"puzzles/roller/{n}" for n in NAMES} <= set(catalog.puzzle_preset_names())
    assert "trap/brake_strip" in catalog.part_preset_names()


@pytest.mark.parametrize("name", NAMES)
def test_roller_presets_pass_static_stages(asset_dir, name):
    res = check(ROLLER / f"{name}.yaml", static_only=True)
    assert res["static"]["ok"] and "warnings" not in res["parts"], res
    raw = yaml.safe_load((ROLLER / f"{name}.yaml").read_text())
    assert any(d.startswith("addon ") for d in raw["demo"])


# ---- slow -----------------------------------------------------------------------------
def walking_only(path: Path, tmp_path: Path) -> PuzzleEnv:
    raw = yaml.safe_load(path.read_text())
    raw["demo"] = [d for d in raw["demo"] if not d.startswith("addon ")]
    out = tmp_path / path.name
    out.write_text(yaml.safe_dump(raw))
    return PuzzleEnv(room=out)


@pytest.mark.slow
@pytest.mark.parametrize("room", [ROLLER / "timed_corridor.yaml", ROLLER / "speed_gate.yaml",
                                  ROOT / "examples/rooms/roller_rink.yaml"], ids=lambda p: p.stem)
def test_walking_only_does_not_solve(asset_dir, tmp_path, room):
    env = walking_only(room, tmp_path)
    solver = ScriptedSolver(env)
    wins = [run_episode(env, s, solver)["success"] for s in range(3)]
    env.close()
    assert not any(wins)
