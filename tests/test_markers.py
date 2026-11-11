"""R26: ArUco markers on parts (ids, faces, textures, static rejection, rendering).

The OpenCV detection test runs only where `cv2` (with `aruco`) is importable; it
is a dev-only check (Open question 6 (a)). Without OpenCV the top-view render is
read back by sampling the marker cells.
"""

from __future__ import annotations

import math

import imageio.v3 as iio
import mujoco
import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym import markers
from mujoco_puzzle_gym.bodies import make_body
from mujoco_puzzle_gym.room import Room, RoomError

GOAL = {"name": "goal", "type": "goal", "pos": [-1.2, -1.2]}


def _room(tmp_path, parts, **extra):
    raw = {"name": "markers", "size": [3.0, 3.0], "spawn": {"pos": [0.0, 0.0], "yaw": 0.0},
           "parts": [*parts, GOAL], **extra}
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def _compile(path):
    room = Room.load(path)
    parts = room.make_parts()
    body = make_body()
    model = room.build_spec(body, parts).compile()
    data = mujoco.MjData(model)
    body.reset(data, room.sample_spawn(None))
    for p in parts.values():
        p.bind(model, data)
        p.reset(model, data)
    mujoco.mj_forward(model, data)
    return parts, model, data


# ---- dictionary and textures ---------------------------------------------------------
def test_every_png_holds_its_own_id():
    for i in range(markers.N_IDS):
        img = iio.imread(markers.png_path(i))
        cell = img.shape[0] // markers.PRINT_CELLS
        cells = img[cell // 2::cell, cell // 2::cell, 0] > 127
        assert not cells[1:-1, 1:-1][[0, -1]].any() and not cells[1:-1, 1:-1][:, [0, -1]].any()
        assert cells[0].all() and cells[-1].all()                     # white margin
        assert markers.decode(cells[2:-2, 2:-2]) == i
        assert np.array_equal(img, markers.marker_image(i))           # PNG == generator


def test_codes_are_unique_under_rotation():
    seen = set()
    for i in range(markers.N_IDS):
        g = markers.bit_grid(i)
        for _ in range(4):
            assert markers.decode(g) == i
            g = np.rot90(g)
        seen.add(markers.BITS[i])
    assert len(seen) == markers.N_IDS


# ---- part values ---------------------------------------------------------------------
ALL_MARKED = [
    {"name": "btn", "type": "button", "pos": [0.6, 0.8], "marker": {"id": 11}},
    {"name": "plate", "type": "pressure", "pos": [0.6, -0.8], "marker": {"id": 12}},
    {"name": "flag", "type": "checkpoint", "pos": [-0.6, 0.8], "marker": {"id": 13}},
    {"name": "door", "type": "door", "pos": [1.0, 0.0], "marker": {"id": 14, "face": "front"}},
    {"name": "swing", "type": "swing_door", "pos": [-1.0, 0.0], "marker": {"id": 15, "face": "back"}},
    {"name": "warp", "type": "warp", "pos": [-0.6, -0.8], "to": "reset", "marker": {"id": 16}},
    {"name": "box", "type": "receptacle", "pos": [0.0, 1.0], "marker": {"id": 17}},
    {"name": "tile", "type": "floor_tile", "pos": [0.0, -1.0], "marker": {"id": 18}},
    {"name": "trap", "type": "trapdoor", "pos": [1.0, -1.1], "marker": {"id": 19}},
]
NORMALS = {"btn": (0, 0, 1), "plate": (0, 0, 1), "flag": (0, 0, 1), "door": (1, 0, 0),
           "swing": (-1, 0, 0), "warp": (0, 0, 1), "box": (0, 0, 1), "tile": (0, 0, 1),
           "trap": (0, 0, 1)}


def test_each_part_gets_its_texture_on_its_face(tmp_path, asset_dir):
    parts, model, data = _compile(_room(tmp_path, ALL_MARKED))
    for cfg in ALL_MARKED:
        p = parts[cfg["name"]]
        mid = cfg["marker"]["id"]
        assert p.marker.id == mid
        [g] = markers.geom_ids(model, p)
        assert model.mat(int(model.geom_matid[g])).name == f"aruco_{mid:02d}"
        assert model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0
        normal = data.geom_xmat[g].reshape(3, 3)[:, 2]
        assert np.allclose(normal, NORMALS[cfg["name"]], atol=1e-6), cfg["name"]
        # on the face: the anchor geom's surface lies just behind the marker box
        anchor = model.geom(p.body_name + p.marker_anchor).id
        d = mujoco.mj_geomDistance(model, data, g, anchor, 0.05, None)
        assert -1e-6 <= d <= 2e-3, (cfg["name"], d)


def test_door_both_faces_and_auto_ids(tmp_path, asset_dir):
    room = [
        {"name": "b1", "type": "button", "pos": [0.6, 0.8], "marker": {"id": 0}},
        {"name": "b2", "type": "button", "pos": [-0.6, 0.8]},                     # auto -> 1
        {"name": "b3", "type": "button", "pos": [0.6, -0.8], "marker": "none"},
        {"name": "door", "type": "door", "pos": [1.0, 0.0]},                      # auto -> 2, both
        {"name": "g", "type": "gate", "op": "not", "in": ["b1"]},                 # logic: none
    ]
    parts, model, data = _compile(_room(tmp_path, room))
    assert parts["b2"].marker.id == 1 and parts["door"].marker.id == 2
    assert parts["b3"].marker is None and parts["g"].marker is None and parts["goal"].marker is None
    assert parts["door"].marker.face == "both"
    normals = [data.geom_xmat[g].reshape(3, 3)[:, 2] for g in markers.geom_ids(model, parts["door"])]
    assert np.allclose(normals, [(1, 0, 0), (-1, 0, 0)], atol=1e-6)
    assert markers.ids(parts) == [0, 1, 2]


def test_default_size_fits_and_is_capped(tmp_path):
    room = Room.load(_room(tmp_path, [
        {"name": "big", "type": "button", "pos": [0.6, 0.8], "size": [1.0, 1.0]},
        {"name": "small", "type": "button", "pos": [-0.6, 0.8], "size": [0.06, 0.06]},
    ]))
    parts = room.make_parts()
    assert parts["big"].marker.size == pytest.approx(markers.DEFAULT_SIZE)
    small = parts["small"].marker.size * markers.PRINT_CELLS / markers.CELLS
    assert small <= 0.06


# ---- static check: rejected at load (stage `values`) ---------------------------------
@pytest.mark.parametrize("parts,needle", [
    ([{"name": "a", "type": "button", "pos": [0.6, 0.8], "marker": {"id": 4}},
      {"name": "b", "type": "door", "pos": [1.0, 0.0], "marker": {"id": 4}}], "both have marker id 4"),
    ([{"name": "a", "type": "button", "pos": [0.6, 0.8], "marker": {"id": 50}}], "outside DICT_4X4_50"),
    ([{"name": "a", "type": "button", "pos": [0.6, 0.8], "marker": {"id": -1}}], "outside DICT_4X4_50"),
    ([{"name": "a", "type": "button", "pos": [0.6, 0.8], "marker": {"size": 0.2}}], "bigger than the top face"),
    ([{"name": "a", "type": "door", "pos": [1.0, 0.0], "height": 0.1, "marker": {"size": 0.1}}],
     "bigger than the both face"),
    ([{"name": "a", "type": "button", "pos": [0.6, 0.8], "marker": {"face": "front"}}], "face 'front'"),
    ([{"name": "a", "type": "button", "pos": [0.6, 0.8], "marker": {"colour": 1}}], "must be `none`"),
    ([{"name": "a", "type": "button", "pos": [0.6, 0.8]},
      {"name": "g", "type": "gate", "op": "not", "in": ["a"], "marker": {"id": 3}}], "cannot carry a marker"),
])
def test_bad_markers_are_rejected_at_load(tmp_path, parts, needle):
    with pytest.raises(RoomError) as exc:
        Room.load(_room(tmp_path, parts))
    assert needle in str(exc.value)
    assert exc.value.stage == "values"


def test_copies_of_one_object_share_an_id():
    class P:
        def __init__(self, name, mid, clone_of=None):
            self.name = name
            self.marker = markers.Marker(mid, 0.05, "top")
            self.marker_clone_of = clone_of

    parts = {"egg": P("egg", 5, "egg"), "egg#1": P("egg#1", 5, "egg"), "egg#2": P("egg#2", markers.AUTO, "egg"),
             "x": P("x", markers.AUTO)}
    markers.assign_ids(parts)
    assert parts["egg#2"].marker.id == 5 and parts["x"].marker.id == 0
    parts["y"] = P("y", 5)
    with pytest.raises(ValueError, match="both have marker id 5"):
        markers.assign_ids(parts)


def test_ids_run_out(tmp_path):
    many = [{"name": f"t{i}", "type": "button", "pos": [-1.3 + 0.1 * (i % 26), -1.3 + 0.3 * (i // 26)],
             "size": [0.06, 0.06]} for i in range(51)]
    with pytest.raises(RoomError, match="all 50 DICT_4X4_50 marker ids are used"):
        Room.load(_room(tmp_path, many))


# ---- rendering ---------------------------------------------------------------------
def _top_view(model, data, centre, half_view, px=256):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = centre
    cam.elevation, cam.azimuth = -90.0, 90.0            # +x right, +y up
    cam.distance = half_view / math.tan(math.radians(model.vis.global_.fovy) / 2)
    with mujoco.Renderer(model, px, px) as r:
        r.update_scene(data, camera=cam)
        return r.render()


def test_render_reads_back_the_id(tmp_path, asset_dir):
    """Offscreen top view of a floor button: the 4 x 4 cells decode to its id (no OpenCV)."""
    parts, model, data = _compile(_room(tmp_path, [
        {"name": "btn", "type": "button", "pos": [0.6, 0.8], "marker": {"id": 23, "size": 0.12}}]))
    [g] = markers.geom_ids(model, parts["btn"])
    centre = data.geom_xpos[g].copy()
    half_view = 0.1
    img = _top_view(model, data, centre, half_view).mean(axis=2)
    px = img.shape[0]
    cell = 0.12 / markers.CELLS
    bits = np.zeros((4, 4), dtype=np.uint8)
    lo, hi = img.min(), img.max()
    for r in range(4):
        for c in range(4):
            x = (c - 1.5) * cell                       # +x right
            y = (1.5 - r) * cell                       # +y up (image row 0 at the top)
            u = int(round(px / 2 + x / half_view * px / 2))
            v = int(round(px / 2 - y / half_view * px / 2))
            bits[r, c] = img[v, u] > (lo + hi) / 2
    assert markers.decode(bits) == 23


def test_opencv_detects_the_ids(tmp_path, asset_dir):
    """Dev-only (OQ6): OpenCV's ArUco detector reads the ids from one offscreen render."""
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "aruco"):
        pytest.skip("cv2 without aruco")
    parts, model, data = _compile(_room(tmp_path, [
        {"name": "b1", "type": "button", "pos": [0.3, 0.3], "marker": {"id": 7, "size": 0.12}},
        {"name": "b2", "type": "button", "pos": [-0.3, 0.3], "marker": {"id": 42, "size": 0.12}}]))
    img = _top_view(model, data, [0.0, 0.3, 0.0], 0.5, px=480)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    _, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
    assert ids is not None and sorted(ids.ravel().tolist()) == [7, 42]
