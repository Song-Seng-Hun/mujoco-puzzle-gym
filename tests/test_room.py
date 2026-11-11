"""R5: room.yaml -> combined MJCF + top-view PNG; invalid rooms fail with a reason."""

import json
import subprocess
import sys

import mujoco
import pytest
import yaml

from mujoco_puzzle_gym.room import Room, RoomError

ROOM = "examples/switch_door.yaml"


def mpg(*args):
    return subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", *args],
                          capture_output=True, text=True)


def test_build_writes_mjcf_and_png(asset_dir, repo_root, tmp_path):
    out = mpg("build", str(repo_root / ROOM), "--json", "--out", str(tmp_path))
    assert out.returncode == 0, out.stderr
    res = json.loads(out.stdout)
    assert res["ok"] is True
    png = tmp_path / "switch_door.png"
    assert res["png"] == str(png.resolve()) and png.read_bytes()[:4] == b"\x89PNG"
    model = mujoco.MjModel.from_xml_path(res["mjcf"])     # standalone-loadable
    assert model.body("part/door_b").id > 0
    assert {p["name"] for p in res["parts"]} == {"switch_a", "door_b", "goal"}


def _variant(repo_root, tmp_path, edit):
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    edit(raw)
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_bad_link_fails_with_reason(asset_dir, repo_root, tmp_path):
    path = _variant(repo_root, tmp_path, lambda r: r["links"][0].update(to="door_x"))
    out = mpg("build", str(path), "--json", "--out", str(tmp_path / "out"))
    assert out.returncode != 0
    res = json.loads(out.stdout)
    assert res["ok"] is False and "door_x" in res["error"]


@pytest.mark.parametrize("edit, needle", [
    (lambda r: r["parts"][0].update(type="lever"), "unknown part type 'lever'"),
    (lambda r: r["parts"].pop(), "goal"),
    (lambda r: r["links"][0].update(**{"from": "goal"}), "must be a switch"),
    (lambda r: r["parts"][1].update(name="switch_a"), "duplicate"),
    (lambda r: r.pop("spawn"), "spawn"),
])
def test_invalid_rooms(repo_root, tmp_path, edit, needle):
    with pytest.raises(RoomError, match=needle):
        Room.load(_variant(repo_root, tmp_path, edit))


def test_outer_walls_close_the_room(repo_root):
    room = Room.load(repo_root / ROOM)
    grid = room.grid()
    from mujoco_puzzle_gym.logic import free_mask
    reach = grid.flood(free_mask(grid, room.walls(), {}, set()), room.spawn_pos)
    # Nothing outside the interior is reachable.
    outside = (abs(grid.X) > room.size[0] / 2) | (abs(grid.Y) > room.size[1] / 2)
    assert reach.any() and not (reach & outside).any()
