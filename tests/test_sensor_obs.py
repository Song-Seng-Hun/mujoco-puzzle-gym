"""R39: real-sensor observation items (rangefinder, camera), Box/Dict switch,
`critic_obs`, sensor noise (R35 body layer), item parameters, sensors-only runs.

Camera tests render offscreen (MUJOCO_GL picks the backend; 64x64 frames are
cheap under OSMesa too), so they stay in the fast suite.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.obs import BALLS_K, COIN_OBS_K, obs_kind
from mujoco_puzzle_gym.task import TaskError, load_task

ROOT = Path(__file__).resolve().parents[1]
ROOM = ROOT / "examples" / "switch_door.yaml"
STAND = np.array([-1.0, 0.0, 0.0], dtype=np.float32)


def _box_room(tmp_path, half=1.0, height=0.5):
    """Square room with tall walls (inner faces at +-half) and a goal in a corner."""
    raw = {"name": "box", "size": [2 * half, 2 * half], "wall": {"thickness": 0.06, "height": height},
           "spawn": {"pos": [0.0, 0.0], "yaw": 0.0},
           "parts": [{"name": "goal", "type": "goal", "pos": [-0.7, 0.7], "radius": 0.1}]}
    path = tmp_path / "box.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def _env(task, room=ROOM):
    env = PuzzleEnv(task=task, room=room)
    return env


# ---- spaces ------------------------------------------------------------------------
def test_plain_obs_stays_a_box(asset_dir):
    env = _env(None)
    assert env.observation_space.shape == (18,)
    obs, _ = env.reset(seed=0)
    assert isinstance(obs, np.ndarray) and env.observation_space.contains(obs)


@pytest.mark.parametrize("mode,channels", [("rgb", 3), ("depth", 1), ("rgbd", 4)])
def test_camera_makes_a_dict(asset_dir, mode, channels):
    env = _env({"obs": ["gravity", {"item": "camera", "mode": mode, "size": [40, 48]}]})
    try:
        space = env.observation_space
        assert set(space.spaces) == {"image", "vector"}
        assert space["image"].shape == (channels, 40, 48) and space["image"].dtype == np.uint8
        assert space["vector"].shape == (3,)
        obs, _ = env.reset(seed=0)
        assert space.contains(obs)
        obs, *_ = env.step(STAND)
        assert space.contains(obs)
        assert obs["image"].std() > 0           # not a blank frame (the eye is outside the head)
    finally:
        env.close()


def test_camera_only_obs_has_no_vector_key(asset_dir):
    env = _env({"obs": [{"item": "camera", "mode": "depth"}]})
    try:
        assert set(env.observation_space.spaces) == {"image"}
    finally:
        env.close()


def test_rangefinder_shape_and_range(asset_dir):
    env = _env({"obs": [{"item": "rangefinder", "rays": 7, "fov": 90, "max_dist": 1.5}]})
    assert env.observation_space.shape == (7,)
    obs, _ = env.reset(seed=0)
    for _ in range(3):
        assert obs.shape == (7,) and np.all((obs >= 0) & (obs <= 1))
        obs, *_ = env.step(STAND)


def test_critic_obs_adds_a_critic_key(asset_dir):
    env = _env({"obs": ["gravity", {"item": "rangefinder", "rays": 3}],
                "critic_obs": ["part_pos", "part_state"]})
    space = env.observation_space
    assert set(space.spaces) == {"vector", "critic"}
    assert space["vector"].shape == (6,) and space["critic"].shape == (9,)
    obs, _ = env.reset(seed=0)
    assert space.contains(obs)


def test_item_parameters_and_plain_names(asset_dir, tmp_path):
    """Carry-over 5b/5d: obs items take parameters; plain names keep working."""
    room = ROOT / "examples" / "rooms" / "coin_room.yaml"
    env = _env({"obs": [{"item": "coins", "k": 5}, "gravity"]}, room=room)
    assert env.observation_space.shape == (1 + 3 * 5 + 3,)
    env = _env({"obs": ["coins"]}, room=room)
    assert env.observation_space.shape == (1 + 3 * COIN_OBS_K,)
    env = _env({"obs": [{"item": "balls", "k": 1}]})
    assert env.observation_space.shape == (5,)
    assert BALLS_K != 1


@pytest.mark.parametrize("task,msg", [
    ({"obs": [{"item": "camera", "mode": "infra"}]}, "camera mode"),
    ({"obs": [{"item": "camera", "size": [8, 8]}]}, "camera size"),
    ({"obs": [{"item": "rangefinder", "rays": 0}]}, "rays"),
    ({"obs": [{"item": "rangefinder", "range": 3}]}, "bad parameters"),
    ({"obs": [{"item": "gravity", "k": 3}]}, "bad parameters"),
    ({"obs": [{"item": "gravity", "noise": -1}]}, "noise"),
    ({"obs": [{"nope": 1}]}, "term name"),
    ({"obs": [{"item": "camera"}, {"item": "camera"}]}, "one camera"),
    ({"critic_obs": [{"item": "camera"}]}, "critic_obs"),
    ({"critic_obs": []}, "critic_obs"),
    ({"randomize": {"body": -0.5}}, "randomize.body"),
])
def test_bad_items_are_rejected(task, msg):
    with pytest.raises(TaskError, match=msg):
        load_task({**task, "room": str(ROOM)})


def test_function_term_checks_its_own_values(asset_dir):
    with pytest.raises(ValueError, match="k must be"):
        _env({"obs": [{"item": "coins", "k": 0}]})


def test_obs_kind():
    assert obs_kind({"obs": ["gravity", "ang_vel", {"item": "rangefinder"}, {"item": "camera"}]}) \
        == "sensors_only"
    assert obs_kind({"obs": ["gravity", "part_pos"]}) == "privileged"


# ---- rangefinder accuracy ------------------------------------------------------------
def test_rangefinder_reads_the_wall_in_front(asset_dir, tmp_path):
    env = _env({"obs": [{"item": "rangefinder", "rays": 5, "fov": 60, "max_dist": 2.0}]},
               room=_box_room(tmp_path))
    env.reset(seed=0, options={"layout": {**env.sample_layout(None), "spawn": (0.2, 0.0, 0.0)}})
    for _ in range(10):                      # settle standing (the spawn pose nods ~12 deg)
        obs, *_ = env.step(STAND)
    mid = env.model.site("obs0_rf2").id
    x0 = env.data.site_xpos[mid][0]
    ray = env.data.site_xmat[mid].reshape(3, 3)[:, 2]
    assert abs(np.degrees(np.arcsin(ray[2]))) < 2.0          # level when standing
    assert abs(obs[2] * 2.0 - (1.0 - x0)) < 0.01             # wall face at x = 1.0


def test_rangefinder_reads_max_dist_when_nothing_is_near(asset_dir, tmp_path):
    env = _env({"obs": [{"item": "rangefinder", "rays": 9, "fov": 120, "max_dist": 0.5}]},
               room=_box_room(tmp_path))
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    obs, *_ = env.step(STAND)
    assert np.all(obs == 1.0)


# ---- camera determinism and noise ------------------------------------------------------
def _frames(task, seed=3, steps=3):
    env = _env(task)
    try:
        obs, _ = env.reset(seed=seed)
        out = [obs["image"].copy()]
        for _ in range(steps):
            obs, *_ = env.step(np.array([0.5, 0.0, 0.3], dtype=np.float32))
            out.append(obs["image"].copy())
        return out
    finally:
        env.close()


def test_camera_is_deterministic(asset_dir):
    task = {"obs": [{"item": "camera", "mode": "rgbd", "noise": 0.05}]}
    a, b = _frames(task), _frames(task)
    assert all(np.array_equal(x, y) for x, y in zip(a, b))


def test_camera_brightness_noise_follows_randomize_body(asset_dir):
    noisy = _frames({"obs": [{"item": "camera", "mode": "rgb", "noise": 0.1}]})
    off = _frames({"obs": [{"item": "camera", "mode": "rgb", "noise": 0.1}], "randomize": {"body": 0.0}})
    clean = _frames({"obs": [{"item": "camera", "mode": "rgb"}]})
    assert all(np.array_equal(x, y) for x, y in zip(off, clean))
    assert not np.array_equal(noisy[0], clean[0])


def test_rangefinder_noise_is_seeded_and_scaled(asset_dir, tmp_path):
    room = _box_room(tmp_path)
    task = {"obs": [{"item": "rangefinder", "rays": 3, "fov": 20, "noise": 0.05}]}

    def read(t, seed):
        env = _env(t, room=room)
        obs, _ = env.reset(seed=seed, options={"layout": env.sample_layout(None)})
        return obs
    a, b = read(task, 1), read(task, 1)
    assert np.array_equal(a, b)
    clean = read({"obs": [{"item": "rangefinder", "rays": 3, "fov": 20}]}, 1)
    assert not np.array_equal(a, clean)
    assert np.array_equal(read({**task, "randomize": {"body": 0.0}}, 1), clean)


def test_generic_item_noise_skips_the_critic(asset_dir):
    env = _env({"obs": [{"item": "part_pos", "noise": 0.1}], "critic_obs": ["part_pos"]})
    obs, _ = env.reset(seed=0)
    assert not np.allclose(obs["vector"], obs["critic"])
    env2 = _env({"obs": ["part_pos"]})
    obs2, _ = env2.reset(seed=0)
    assert np.allclose(obs["critic"], obs2)


# ---- training and eval through the Dict / critic_obs paths -----------------------------
@pytest.mark.parametrize("name,extra", [
    ("sensors_only", {"obs": ["gravity", "ang_vel", {"item": "rangefinder"},
                              {"item": "camera", "mode": "rgbd"}]}),
    ("critic_obs", {"obs": ["gravity", {"item": "rangefinder"}, {"item": "camera"}],
                    "critic_obs": ["part_pos", "part_state", "target_pos"]}),
])
def test_sensor_tasks_train_save_and_eval(asset_dir, tmp_path, name, extra):
    from mujoco_puzzle_gym.evaluate import evaluate
    from mujoco_puzzle_gym.train import train

    task = {"room": str(ROOM), "n_envs": 2, "algo_kwargs": {"n_steps": 64, "batch_size": 128},
            **extra}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(task))
    summary = train(path, minutes=1.0, runs_dir=tmp_path / "runs", run_id=name, max_steps=128)
    assert summary["obs_kind"] == "sensors_only"
    assert summary["critic_obs"] is ("critic_obs" in extra)
    assert (Path(summary["run_dir"]) / summary["checkpoint"]).exists()
    saved = json.loads((Path(summary["run_dir"]) / "summary.json").read_text())
    assert saved["obs_kind"] == "sensors_only"
    result = evaluate(summary["run_dir"], episodes=1, workers=1)
    assert result["episodes"] == 1


def test_critic_obs_rejects_sac(tmp_path):
    from mujoco_puzzle_gym.train import train

    path = tmp_path / "t.yaml"
    path.write_text(yaml.safe_dump({"room": str(ROOM), "critic_obs": ["part_pos"]}))
    with pytest.raises(TaskError, match="actor-critic"):
        train(path, minutes=1.0, algo="sac", runs_dir=tmp_path / "runs", run_id="x")


# ---- R112: which observation terms a robot could not produce -------------------------

def test_the_privileged_terms_are_declared_where_a_program_can_read_them():
    """`PRIVILEGED_OBS` and the "Privileged:" labels in the term docstrings are the same
    fact written twice, so this fails if either side drifts.

    Why it matters: a task meant to train a policy for hardware is disqualified by any of
    these - they read the simulator's own state (a part's exact position, a part's internal
    state, which part the solution wants next), and nothing on the robot produces them. The
    labels were already in the docstrings; the tuple is the version `privileged_terms()`
    can check a task against.
    """
    import inspect

    from mujoco_puzzle_gym.obs import PRIVILEGED_OBS, privileged_terms
    from mujoco_puzzle_gym.registry import OBS

    labelled = {n for n, o in OBS.items()
                if (inspect.getdoc(o) or "").lstrip().startswith("Privileged:")}
    assert labelled == set(PRIVILEGED_OBS), (labelled, PRIVILEGED_OBS)
    assert privileged_terms(["lin_vel", "part_pos", {"item": "target_pos"}, "camera"]) == \
        ["part_pos", "target_pos"]
    assert privileged_terms(["lin_vel", "gravity", "camera", "rangefinder", "beak"]) == []
    assert privileged_terms(None) == [] and privileged_terms([]) == []


def test_exactly_one_shipped_task_could_be_observed_by_the_robot(repo_root):
    """The audit run on the real files, and the count it returns.

    Most shipped tasks train with privileged observations on purpose, and that is fine.
    What was not fine was that **every** one of them did: when this test was first written
    it asserted zero clean tasks, because the arm answered `None` for `view_camera` and had
    no rangefinder mount, so it had no exteroception to be clean with. R115 wires its own
    wrist camera and `task_pick_and_place_onboard.yaml` is the first task with no privileged
    term in `obs:`. Read its header for how narrow that claim is - no policy has been
    trained on it.

    If this count changes, change it here and say so in the docs; the number is the point.
    """
    tasks = sorted((repo_root / "examples").glob("task_*.yaml"))
    assert tasks, "no example tasks found"
    from mujoco_puzzle_gym.obs import privileged_terms

    clean, dirty = [], []
    for path in tasks:
        spec = yaml.safe_load(path.read_text()) or {}
        (dirty if privileged_terms(spec.get("obs")) else clean).append(path.name)
    assert dirty, "the audit found nothing privileged in any task, which cannot be right"
    assert clean == ["task_pick_and_place_onboard.yaml"], clean


def test_the_clean_task_builds_and_runs(so101_asset_dir, repo_root):
    """What "clean" is actually checked to mean: the task loads, the observation space holds
    an image beside the vector, the image is not blank, and an episode steps. Measured:
    Dict(image (3, 48, 64) uint8, vector (14,)), image std 31.2, 30 random steps.

    It does **not** check that a policy can learn the room from it. Nothing here does, and
    the task's own header says so.
    """
    from mujoco_puzzle_gym.task import load_task

    env = PuzzleEnv(task=load_task(repo_root / "examples/task_pick_and_place_onboard.yaml"))
    try:
        obs, _ = env.reset(seed=0)
        assert set(obs) == {"image", "vector"}, sorted(obs)
        assert obs["image"].shape == (3, 48, 64) and obs["image"].dtype == np.uint8
        assert float(obs["image"].std()) > 5.0, "the eye sees nothing"
        for _ in range(30):
            env.step(env.action_space.sample())
    finally:
        env.close()


def test_the_arm_has_an_eye_at_all(so101_asset_dir):
    """R115, the reason the count above could not have been 1 before: a body with no camera
    and no rangefinder cannot be observed without privileged terms. Measured at the home
    pose, the wrist camera sits 0.04 m behind and above the jaw and looks down across the
    working area."""
    import mujoco

    from mujoco_puzzle_gym.bodies import flat_floor_spec, make_body

    body = make_body("so101")
    model = body.spawn(flat_floor_spec(), (0.0, 0.0, 0.0)).compile()
    cam = body.view_camera(model)
    assert cam is not None and model.camera(cam).name.endswith("wrist_cam")
    data = mujoco.MjData(model)
    body.reset(data)
    mujoco.mj_forward(model, data)
    view = -np.asarray(data.cam_xmat[cam], float).reshape(3, 3)[:, 2]
    assert view[2] < -0.3, ("the eye should look down over the table", view)
