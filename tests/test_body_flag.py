"""R49: `--body` picks the body, on every command, and never lies about which one ran.

Three contracts live here:

- On a room (`build`, `check`, `demo <room>`) and on a task (`train`, `eval --onnx`) the
  flag chooses the body, and the chosen name comes back in the result.
- On a finished run (`eval`, `render`, `diagnose`) it is *checked*, not applied: a policy
  belongs to the body it was trained with, and replaying it on another body would be a
  silently wrong number rather than a comparison.
- Absent, the flag changes nothing: the task's own `body:` stands. This is the direction
  that bites - a flag defaulting to the package default body would quietly override every
  task written for the other body.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from mujoco_puzzle_gym.bodies import BODIES, DEFAULT_BODY, make_body, resolve
from mujoco_puzzle_gym.task import TaskError, load_task

ROOT = Path(__file__).resolve().parents[1]
ROOM = str(ROOT / "examples" / "switch_door.yaml")
TASK = str(ROOT / "examples" / "task_ppo.yaml")
OTHER = next(b for b in BODIES if b != DEFAULT_BODY)


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", *args],
                          capture_output=True, text=True)


def body_of(result: dict) -> str:
    """The body name a command reports. `check` says `body_name`, because its own
    capability-gate stage already owns the key `body`."""
    return result.get("body_name") or result["body"]


# --- the task layer -------------------------------------------------------------------

def test_resolve_names_one_default():
    assert resolve(None) == resolve("") == resolve("default") == DEFAULT_BODY
    assert resolve(OTHER) == OTHER


def test_flag_overrides_the_task_file():
    assert load_task(TASK, body=OTHER)["body"] == OTHER


def test_no_flag_leaves_the_task_alone():
    """The regression this ordering exists to prevent: with the flag absent a task that
    asks for the other body keeps it."""
    assert load_task({"room": ROOM, "body": OTHER})["body"] == OTHER
    assert resolve(load_task({"room": ROOM})["body"]) == DEFAULT_BODY


def test_unknown_body_in_a_task_names_the_usable_bodies():
    with pytest.raises(TaskError) as exc:
        load_task({"room": ROOM, "body": "spot"})
    assert "spot" in str(exc.value) and all(b in str(exc.value) for b in BODIES)


# --- the command line -----------------------------------------------------------------

def test_unknown_body_exits_nonzero_with_the_usable_names():
    out = run("check", ROOM, "--body", "spot")
    assert out.returncode != 0
    assert "spot" in out.stderr and all(b in out.stderr for b in BODIES)


@pytest.mark.parametrize("body", list(BODIES))
def test_build_compiles_the_body_it_was_asked_for(body, asset_dir, go2_asset_dir, tmp_path):
    """`mpg build` used to compile the package default body whatever the room was for,
    so the MJCF and PNG of a room written for the other body showed the wrong robot."""
    out = run("build", "--json", "--body", body, ROOM, "--out", str(tmp_path / body))
    assert out.returncode == 0, out.stderr
    result = json.loads(out.stdout)
    assert body_of(result) == body
    assert Path(result["mjcf"]).is_file() and Path(result["png"]).is_file()


def test_build_differs_between_bodies(asset_dir, go2_asset_dir, tmp_path):
    sizes = {}
    for body in BODIES:
        out = run("build", "--json", "--body", body, ROOM, "--out", str(tmp_path / body))
        sizes[body] = json.loads(out.stdout)["model"]
    assert len({tuple(sorted(m.items())) for m in sizes.values()}) == len(BODIES), sizes


@pytest.mark.parametrize("body", list(BODIES))
def test_check_answers_for_that_body(body, asset_dir, go2_asset_dir, so101_asset_dir):
    """The flag decides which body every stage answers for, and the answer is allowed to
    be "this body cannot". The example room has a floor switch, which needs walking, so a
    body with no legs is refused by name - that is the capability gate working, not a
    failure of `--body`."""
    out = run("check", "--json", "--body", body, ROOM, "--static")
    result = json.loads(out.stdout)
    assert body_of(result) == body
    walks = "walk" in make_body(body).capabilities.names
    assert result["body"]["ok"] is walks, result["body"]
    assert out.returncode == (0 if walks else 1)
    if not walks:
        assert "walk" in result["body"]["reason"]


def test_the_gate_tells_the_bodies_apart(asset_dir, go2_asset_dir, so101_asset_dir,
                                          g1_asset_dir):
    """The gate is not decoration, and the claim is exactly as strong as the facts.

    Two walking bodies may declare the *same* capability names - the quadruped and the
    humanoid both declare `walk` and nothing else - and differ only in their numbers. So
    what is asserted is that the names are not all alike, that only bodies with legs
    declare `walk`, and that bodies sharing a name set still differ in the limits parts
    compare against. An earlier version of this test asserted every body's names were
    unique, which was simply untrue once a second walker arrived.
    """
    caps, limits = {}, {}
    for body in BODIES:
        out = run("check", "--json", "--body", body, ROOM, "--static")
        caps[body] = frozenset(json.loads(out.stdout)["body"]["capabilities"])
        adapter = make_body(body)
        limits[body] = (adapter.footprint_radius,
                        adapter.capabilities.limit("walk_speed"))
    assert len(set(caps.values())) > 1, caps
    walkers = {b for b, c in caps.items() if "walk" in c}
    assert walkers == {b for b in BODIES if "walk" in make_body(b).capabilities.names}
    assert walkers and walkers != set(BODIES)
    for a, b in ((x, y) for x in BODIES for y in BODIES if x < y):
        if caps[a] == caps[b]:
            assert limits[a] != limits[b], (a, b, limits[a])


# --- a finished run: checked, not applied ---------------------------------------------

RUN_COMMANDS = {
    "eval": ["--episodes", "1", "--workers", "1"],
    "render": ["--seed", "1000000"],
    "diagnose": ["--episodes", "1", "--workers", "1", "--no-video"],
}


@pytest.mark.parametrize("command", list(RUN_COMMANDS))
def test_the_runs_own_body_is_accepted(command, quick_run, tmp_path):
    extra = RUN_COMMANDS[command] + (["--out", str(tmp_path / "r.mp4")] if command == "render" else [])
    out = run(command, "--json", "--body", DEFAULT_BODY, str(quick_run), *extra)
    assert out.returncode == 0, out.stderr
    result = json.loads(out.stdout)
    assert result["ok"] is True and result["body"] == DEFAULT_BODY


@pytest.mark.parametrize("command", list(RUN_COMMANDS))
def test_another_body_is_refused_with_the_recorded_one(command, quick_run, tmp_path):
    extra = RUN_COMMANDS[command] + (["--out", str(tmp_path / "r.mp4")] if command == "render" else [])
    out = run(command, "--json", "--body", OTHER, str(quick_run), *extra)
    assert out.returncode != 0, out.stdout
    error = json.loads(out.stdout)["error"]
    assert DEFAULT_BODY in error and OTHER in error and "trained with" in error


# --- the run record -------------------------------------------------------------------

@pytest.mark.slow
def test_train_records_the_body_it_used(asset_dir, go2_asset_dir, tmp_path):
    """R49's record: the body is in `task.resolved.yaml` and in the run summary, by name,
    never as the word `default` - a run that cannot say which body it used cannot be
    compared with another."""
    out = run("train", "--json", "--body", OTHER, TASK, "--minutes", "1", "--max-steps", "64",
              "--n-envs", "1", "--runs-dir", str(tmp_path))
    assert out.returncode == 0, out.stderr
    summary = json.loads(out.stdout)
    assert summary["body"] == OTHER
    resolved = yaml.safe_load((Path(summary["run_dir"]) / "task.resolved.yaml").read_text())
    assert resolved["body"] == OTHER


# --- a wall plate out of reach (found after the R46 audit) ----------------------------

WALL_ROOM = {
    "name": "wallbtn",
    "size": [2.4, 1.6],
    "wall": {"thickness": 0.06, "height": 1.6},
    "walls": [{"from": [0.3, -0.8], "to": [0.3, -0.3]}, {"from": [0.3, 0.3], "to": [0.3, 0.8]}],
    "spawn": {"pos": [-0.7, 0.0], "yaw": 0.0},
    "parts": [
        {"name": "btn", "type": "button", "mount": "wall", "kind": "oneshot", "trigger": "body",
         "pos": [-1.17, 0.4], "size": [0.2, 0.2], "height": 0.2, "facing": 0.0},
        {"name": "door", "type": "door", "mode": "oneshot", "pos": [0.3, 0.0], "width": 0.5,
         "axis": "y", "in": ["btn"]},
        {"name": "goal", "type": "goal", "pos": [0.9, 0.0], "radius": 0.15},
    ],
}


def wall_room(tmp_path, height: float) -> str:
    room = yaml.safe_load(yaml.safe_dump(WALL_ROOM))
    room["parts"][0]["height"] = height
    path = tmp_path / f"wall{height}.yaml"
    path.write_text(yaml.safe_dump(room, sort_keys=False))
    return str(path)


@pytest.mark.parametrize("height,ok", [(0.15, True), (0.2, True), (0.21, True),
                                       (0.5, False), (1.0, False)])
def test_a_wall_plate_past_the_bodys_reach_is_refused(height, ok, asset_dir, tmp_path):
    """`mount: wall` declares `grab` and nothing more, so it used to say "I need a
    beak" rather than "I need a beak that reaches this high": a plate at the schema's
    ceiling of 1.0 m, nearly five times the declared beak height, passed every static
    stage. The capability gate compares names, so the height is compared in the part."""
    from mujoco_puzzle_gym.check import check

    result = check(wall_room(tmp_path, height), static_only=True)
    assert result["parts"]["ok"] is ok, result["parts"]
    if not ok:
        reason = result["parts"]["reason"]
        assert f"{height:g} m up" in reason and "0.21 m" in reason, reason


def test_the_existing_wall_presets_are_within_reach(asset_dir):
    """The two shipped wall-button presets sit at 0.15 m and 0.2 m, under the 0.21 m the
    body declares, so this check cannot have changed any existing room."""
    from mujoco_puzzle_gym import catalog

    heights = {}
    for name in ("beak/wall_button_beak_low", "beak/wall_button_beak_mid"):
        raw = yaml.safe_load((catalog.PRESET_ROOT / "parts" / f"{name}.yaml").read_text())
        heights[name] = float(raw["height"])
    assert max(heights.values()) <= 0.21, heights
