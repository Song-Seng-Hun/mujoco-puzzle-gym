"""Community spec R2: `mpg check --community` limits for shared rooms."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from mujoco_puzzle_gym import registry
from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.cli import main
from mujoco_puzzle_gym.parts import Switch
from mujoco_puzzle_gym.registry import register_part

ROOT = Path(__file__).resolve().parents[1]
BASE = yaml.safe_load((ROOT / "examples/switch_door.yaml").read_text())


def write(folder: Path, raw: dict, name: str = "room.yaml") -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    p = folder / name
    p.write_text(yaml.safe_dump(raw))
    return p


def community(path: Path) -> dict:
    return check(path, static_only=True, community=True)


def problems(path: Path) -> str:
    r = community(path)
    assert r["ok"] is False and r["community"]["ok"] is False, r
    return " ".join(r["community"]["problems"])


def test_examples_pass(asset_dir):
    rooms = sorted((ROOT / "examples/rooms").glob("*.yaml")) + [ROOT / "examples/switch_door.yaml"]
    for p in rooms:
        r = community(p)
        assert r["community"] == {"ok": True}, (p.name, r["community"])


def test_plugins_key(tmp_path):
    assert "plugins" in problems(write(tmp_path, {**BASE, "plugins": ["os"]}))


@pytest.mark.parametrize("rel", ["../other/room.yaml", "/etc/hosts", "sub/../../x.yaml"])
def test_stages_outside_folder(tmp_path, rel):
    write(tmp_path / "other", BASE)
    p = write(tmp_path / "mine", {**BASE, "stages": [rel]})
    assert "outside the room's folder" in problems(p)


def test_stages_inside_folder_pass(asset_dir, tmp_path):
    second = {"name": "second", "size": [1.2, 1.2], "spawn": {"pos": [0.0, 0.0]},
              "parts": [{"name": "goal2", "type": "goal", "pos": [0.0, 0.0], "radius": 0.15}]}
    write(tmp_path / "mine" / "sub", second, "second.yaml")
    p = write(tmp_path / "mine", {**BASE, "stages": ["sub/second.yaml"]})
    assert community(p)["community"] == {"ok": True}


@pytest.fixture
def plugin_part():
    @register_part("community_test_pad")
    class Pad(Switch):
        pass

    yield "community_test_pad"
    registry.PARTS.pop("community_test_pad", None)


def test_plugin_part(asset_dir, tmp_path, plugin_part):
    raw = copy.deepcopy(BASE)
    raw["parts"][0]["type"] = plugin_part
    assert "plugin module" in problems(write(tmp_path, raw))


def test_too_many_parts(asset_dir, tmp_path):
    raw = {**BASE, "size": [9.0, 9.0], "walls": [], "links": []}
    raw["parts"] = [{"name": f"g{i}", "type": "goal", "pos": [-4 + (i % 13) * 0.6, -4 + (i // 13) * 0.6],
                     "radius": 0.1} for i in range(65)]
    assert "65 parts" in problems(write(tmp_path, raw))


def test_too_large(asset_dir, tmp_path):
    raw = {**BASE, "size": [12.0, 1.8]}
    assert "the limit is 10 m" in problems(write(tmp_path, raw))


def test_map_too_large(asset_dir, tmp_path):
    rows = ["#" * 42] + ["#" + "." * 40 + "#"] * 4 + ["#" * 42]
    raw = {"name": "wide_map", "cell": 0.1, "map": "\n".join(rows) + "\n",
           "spawn": {"pos": [-1.5, 0.0], "yaw": 0.0},
           "parts": [{"name": "goal", "type": "goal", "pos": [1.5, 0.0], "radius": 0.1}]}
    assert "map is 42 x 6 cells" in problems(write(tmp_path, raw))


def test_cli_rejects_task(tmp_path, capsys):
    p = write(tmp_path, BASE)
    assert main(["check", str(p), "--community", "--task", "examples/task_ppo.yaml", "--json"]) == 1
    assert "no --task" in json.loads(capsys.readouterr().out)["error"]


# ---- R1: tools/check_community_files.py ------------------------------------

def _files_tool():
    import importlib.util
    spec = importlib.util.spec_from_file_location("ccf", ROOT / "tools/check_community_files.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_files_accepted(tmp_path):
    tool = _files_tool()
    d = tmp_path / "community/rooms/alice"
    d.mkdir(parents=True)
    (d / "room.yaml").write_text("name: r\n")
    (d / "README.md").write_text("# alice\n")
    assert tool.problems(["community/rooms/alice/room.yaml", "community/rooms/alice/README.md"],
                         root=tmp_path) == []


@pytest.mark.parametrize("rel, why", [
    ("community/rooms/alice/policy.pkl", "only room files"),
    ("community/rooms/alice/plugin.py", "only room files"),
    ("community/rooms/alice/notes.md", "only room files"),
    ("community/rooms/alice/sub/room.yaml", "no subfolders"),
    ("community/room.yaml", "no subfolders"),
])
def test_files_rejected(tmp_path, rel, why):
    assert why in " ".join(_files_tool().problems([rel], root=tmp_path))


def test_files_too_big_and_symlink(tmp_path):
    tool = _files_tool()
    d = tmp_path / "community/rooms/alice"
    d.mkdir(parents=True)
    (d / "big.yaml").write_text("#" * (64 * 1024 + 1))
    (d / "link.yaml").symlink_to(tmp_path / "elsewhere.yaml")
    out = " ".join(tool.problems(["community/rooms/alice/big.yaml",
                                  "community/rooms/alice/link.yaml"], root=tmp_path))
    assert "the limit is 65536" in out and "symbolic links" in out
    assert tool.main(["community/rooms/alice/x.zip"]) == 1


def test_repo_community_folder_is_clean():
    files = [str(p.relative_to(ROOT)) for p in (ROOT / "community").rglob("*") if p.is_file()]
    assert _files_tool().problems(files, root=ROOT) == []


def _git(cwd, *args):
    import subprocess
    subprocess.run(["git", "-c", "user.name=t", "-c", "user.email=t@t", *args], cwd=cwd,
                   check=True, capture_output=True)


def test_room_pr_must_not_change_other_files(tmp_path, monkeypatch, capsys):
    tool = _files_tool()
    _git(tmp_path, "init", "-q", "-b", "main")
    (tmp_path / "tool.py").write_text("x = 1\n")
    _git(tmp_path, "add", ".")
    _git(tmp_path, "commit", "-qm", "base")
    d = tmp_path / "community/rooms/alice"
    d.mkdir(parents=True)
    (d / "r.yaml").write_text("name: r\n")
    _git(tmp_path, "add", ".")
    _git(tmp_path, "commit", "-qm", "room")
    monkeypatch.chdir(tmp_path)
    assert tool.main(["--base", "main~1", "--author", "alice", "--rooms-out", "rooms.txt"]) == 0
    assert (tmp_path / "rooms.txt").read_text() == "community/rooms/alice/r.yaml\n"
    (tmp_path / "tool.py").write_text("x = 2\n")      # the PR also edits code
    _git(tmp_path, "commit", "-qam", "edit tool")
    assert tool.main(["--base", "main~2", "--author", "alice", "--rooms-out", "rooms.txt"]) == 1
    assert "outside it" in capsys.readouterr().out
    assert (tmp_path / "rooms.txt").read_text() == ""


def test_yaml_aliases_rejected(tmp_path):
    p = tmp_path / "room.yaml"
    p.write_text("a: &x [1, 2]\nb: [*x, *x]\n" + yaml.safe_dump(BASE))
    assert "aliases" in problems(p)


def test_non_string_stage_rejected(tmp_path):
    assert "must be a file name" in problems(write(tmp_path, {**BASE, "stages": [["a"]]}))


@pytest.mark.parametrize("name", ["../../x", "/tmp/x", "..", "a\\\\b"])
def test_room_name_with_path_parts_rejected(tmp_path, name):
    r = check(write(tmp_path, {**BASE, "name": name}), static_only=True)
    assert r["ok"] is False and "plain name" in r["values"]["reason"]


def test_files_names_and_author(tmp_path):
    tool = _files_tool()
    d = tmp_path / "community/rooms/alice"
    d.mkdir(parents=True)
    (d / "r.yaml").write_text("name: r\n")
    spaced = "community/rooms/alice/r.yaml community/rooms/alice/README.md"
    assert tool.problems([spaced], root=tmp_path)
    assert "letters, digits" in " ".join(tool.problems(["community/rooms/alice/my room.yaml"],
                                                       root=tmp_path))
    assert "not a file" in " ".join(tool.problems(["community/rooms/alice/gone.yaml"], root=tmp_path))
    ok = ["community/rooms/alice/r.yaml"]
    assert tool.problems(ok, root=tmp_path, author="Alice") == []
    assert "author's folder" in " ".join(tool.problems(ok, root=tmp_path, author="mallory"))
