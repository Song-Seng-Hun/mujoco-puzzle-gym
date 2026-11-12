"""R3 (schema part): M1 parts have value schemas; out-of-range values fail build/check
with the reason and the allowed range."""

import json
import subprocess
import sys

import pytest
import yaml

from mujoco_puzzle_gym.parts import Door, Goal, Switch
from mujoco_puzzle_gym.room import Room, RoomError
from mujoco_puzzle_gym.schema import Field, SchemaError, resolve

ROOM = "examples/switch_door.yaml"


def mpg(*args):
    return subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", *args],
                          capture_output=True, text=True)


def _variant(repo_root, tmp_path, edit):
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    edit(raw)
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_m1_parts_have_schemas():
    for cls, own in ((Switch, {"size", "latch"}),
                     (Door, {"width", "axis", "thickness", "height", "open_dir"}),
                     (Goal, {"radius"})):
        schema = cls.full_schema()
        assert own | {"pos", "random"} <= set(schema)
        for f in schema.values():
            assert isinstance(f, Field)
            if f.kind in ("float", "vec") and f.name != "pos":
                assert f.unit and f.min is not None and f.max is not None, f


def test_defaults_are_filled(repo_root):
    parts = Room.load(repo_root / ROOM).make_parts()
    assert parts["door_b"].cfg["height"] == 0.25
    assert parts["switch_a"].cfg["latch"] is True


@pytest.mark.parametrize("index, key, value, needle", [
    (1, "width", 5.0, "width = 5.0 m is out of range [0.1, 2]"),
    (0, "size", [0.3, 9.0], "size = [0.3, 9.0] m is out of range [0.02, 2]"),
    (2, "radius", -0.1, "radius = -0.1 m is out of range [0.03, 1]"),
    (1, "axis", "z", "one of 'x', 'y'"),
    (0, "random", [-0.1, 0.0], "random = [-0.1, 0.0] m is out of range [0, 5]"),
])
def test_out_of_range_fails_load(repo_root, tmp_path, index, key, value, needle):
    path = _variant(repo_root, tmp_path, lambda r: r["parts"][index].update({key: value}))
    with pytest.raises(RoomError) as err:
        Room.load(path)
    assert needle in str(err.value)
    assert f"part {yaml.safe_load(path.read_text())['parts'][index]['name']!r}" in str(err.value)


@pytest.mark.parametrize("cmd", ["build", "check"])
def test_out_of_range_fails_cli(repo_root, tmp_path, cmd):
    path = _variant(repo_root, tmp_path, lambda r: r["parts"][1].update(width=5.0))
    out = mpg(cmd, str(path), "--json", *(["--out", str(tmp_path / "o")] if cmd == "build" else []))
    assert out.returncode != 0
    res = json.loads(out.stdout)
    assert res["ok"] is False and "door_b" in res["error"] and "[0.1, 2]" in res["error"]


def test_resolve_kinds():
    schema = {f.name: f for f in (Field("n", "int", 1, min=0, max=3), Field("pos", "vec", length=2),
                                  Field("tags", "list", []))}
    assert resolve("x", schema, {"pos": [0, 1], "tags": "a"}) == {"n": 1, "pos": [0.0, 1.0],
                                                                    "tags": ["a"]}
    with pytest.raises(SchemaError, match="`pos` is required"):
        resolve("x", schema, {})
    with pytest.raises(SchemaError, match="whole number"):
        resolve("x", schema, {"pos": [0, 0], "n": 1.5})
    with pytest.raises(SchemaError, match="list of 2 numbers"):
        resolve("x", schema, {"pos": [0]})
