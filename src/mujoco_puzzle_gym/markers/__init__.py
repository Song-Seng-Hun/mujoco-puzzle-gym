"""ArUco markers on parts (R26).

Parts the agent must tell apart by eye carry an ArUco marker from the
`DICT_4X4_50` dictionary: buttons, pressure plates, checkpoints, doors (sliding,
swing, trapdoor), receptacles (sockets), warps and floor tiles. Signal
converters (gates, counters, transceivers) and the other parts carry none
unless room.yaml gives one.

Part value `marker:`
- omitted: parts that carry markers get the lowest free id (room order);
- `none`: no marker;
- `{id, size, face}` (each optional): `id` in 0..49, `size` [m] the side of the
  black square (the white margin adds one cell per side, so the printed square
  is `size * 8 / 6`), `face` one of the part's faces (`top`, or for doors
  `both` (default), `front`, `back`; front faces +x for a door along y, +y along x).

The textures are the PNGs in `markers/png/` (one per id, made by
`tools/make_aruco_markers.py` from the bit table below; no OpenCV at run time).
Each marker is a thin visual box (no collision) whose local +z is the face
normal; MuJoCo maps the PNG unmirrored onto that face.

Room load (static check, stage `values`) rejects an id outside the dictionary,
a marker bigger than its face, and two parts with the same id, except copies of
one object (parts whose `marker_clone_of` names the same original, e.g. the
copies a dispenser makes) which share the original's id, and a receptacle
showing a marker id its `accept` lists (a key's socket, R28).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import mujoco
import numpy as np

from ..schema import SchemaError

if TYPE_CHECKING:
    from ..parts.base import Part

DICTIONARY = "DICT_4X4_50"
#: DICT_4X4_50 codes (OpenCV 4.12 `getBitsFromByteList`): 16 bits, row-major
#: from the top-left cell, most significant bit first, 1 = white.
BITS = (
    46386, 3994, 13101, 39238, 21662, 31181, 40494, 50418, 65242, 53078,
    63889, 4519, 3767, 10767, 9393, 9790, 18021, 26112, 27742, 30383,
    34443, 45099, 52437, 56706, 65095, 38001, 44260, 42324, 8483, 13423,
    17429, 22450, 40655, 61643, 2222, 2345, 6261, 1279, 3574, 7258,
    5912, 10792, 12940, 14514, 9448, 12011, 11583, 19300, 20526, 20499,
)
N_IDS = len(BITS)
CELLS = 6                      # black border + 4 x 4 bits
PRINT_CELLS = CELLS + 2        # plus a white margin of one cell
PNG_DIR = Path(__file__).parent / "png"
DEFAULT_SIZE = 0.1             # [m] black square, shrunk to fit small faces
FIT = 0.9                      # default size: at most this fraction of the face
LIFT = 0.0006                  # [m] marker box centre above its face
HALF_THICK = 0.0005            # [m]
FACES_FLAT = ("top",)
FACES_PANEL = ("both", "front", "back")


def png_path(marker_id: int) -> Path:
    return PNG_DIR / f"aruco_4x4_50_{marker_id:02d}.png"


def bit_grid(marker_id: int) -> np.ndarray:
    """(4, 4) bits of `marker_id`, 1 = white."""
    code = BITS[marker_id]
    return np.array([(code >> (15 - i)) & 1 for i in range(16)], dtype=np.uint8).reshape(4, 4)


def marker_image(marker_id: int, cell_px: int = 16) -> np.ndarray:
    """RGB uint8 image: white margin, black border, the 4 x 4 bits."""
    cells = np.ones((PRINT_CELLS, PRINT_CELLS), dtype=np.uint8)
    cells[1:-1, 1:-1] = 0
    cells[2:-2, 2:-2] = bit_grid(marker_id)
    img = np.kron(cells, np.ones((cell_px, cell_px), dtype=np.uint8)) * 255
    return np.repeat(img[:, :, None], 3, axis=2)


def decode(cells: np.ndarray) -> int | None:
    """Id of a (4, 4) bit grid read in any of the four rotations (not mirrored)."""
    g = np.asarray(cells, dtype=np.uint8)
    for _ in range(4):
        code = int("".join(str(int(b)) for b in g.ravel()), 2)
        if code in BITS:
            return BITS.index(code)
        g = np.rot90(g)
    return None


# ---- part values -----------------------------------------------------------------
AUTO = "auto"


@dataclass
class Marker:
    id: int | str               # an id, or AUTO until the room assigns one
    size: float                 # [m] black square side
    face: str


def parse(part: "Part", value: Any) -> Marker | None:
    """Check `marker:` of `part` (see the module doc); None = no marker."""
    owner = f"part {part.name!r}"
    faces = part.marker_faces
    if value is None:
        if not part.marked:
            return None
        value = {}
    if value == "none" or value is False:
        return None
    if not faces:
        raise SchemaError(f"{owner}: a {part.type_name} part cannot carry a marker (use `marker: none`)")
    if not isinstance(value, dict) or set(value) - {"id", "size", "face"}:
        raise SchemaError(f"{owner}: marker must be `none` or {{id, size, face}}, got {value!r}")
    face = value.get("face", faces[0])
    if face not in faces:
        raise SchemaError(f"{owner}: marker face {face!r} is not allowed; one of {list(faces)}")
    mid = value.get("id", AUTO)
    if mid != AUTO:
        if isinstance(mid, bool) or not isinstance(mid, int) or not 0 <= mid < N_IDS:
            raise SchemaError(f"{owner}: marker id {mid!r} is outside {DICTIONARY} [0, {N_IDS - 1}]")
    room = float(part.marker_face_size(face))
    if room <= 0:
        raise SchemaError(f"{owner}: the {face} face has no room for a marker (use `marker: none`)")
    size = value.get("size")
    if size is None:
        size = min(DEFAULT_SIZE, FIT * room * CELLS / PRINT_CELLS)
    elif isinstance(size, bool) or not isinstance(size, (int, float)) or size <= 0:
        raise SchemaError(f"{owner}: marker size must be a positive number [m], got {size!r}")
    printed = float(size) * PRINT_CELLS / CELLS
    if printed > room + 1e-9:
        raise SchemaError(f"{owner}: marker size {size:g} m (printed {printed:.3f} m with its "
                          f"white margin) is bigger than the {face} face ({room:.3f} m); "
                          f"largest size {room * CELLS / PRINT_CELLS:.3f} m")
    return Marker(mid, float(size), face)


def assign_ids(parts: dict[str, "Part"]) -> None:
    """Give AUTO markers the lowest free ids (room order) and reject duplicates.

    Parts with the same `marker_clone_of` (copies of one object) share one id."""
    used: dict[int, tuple[str, Any]] = {}
    clones: dict[Any, int] = {}
    for name, p in parts.items():
        m = getattr(p, "marker", None)
        if m is None or m.id == AUTO:
            continue
        group = getattr(p, "marker_clone_of", None)
        if m.id in used:
            other, ogroup = used[m.id]
            shared = m.id in getattr(p, "marker_match", ()) or m.id in getattr(parts[other], "marker_match", ())
            if (group is None or group != ogroup) and not shared:
                raise SchemaError(f"parts {other!r} and {name!r} both have marker id {m.id}; "
                                  f"give one another id or leave it out for a free one")
        else:
            used[m.id] = (name, group)
        if group is not None:
            clones.setdefault(group, m.id)
    free = (i for i in range(N_IDS) if i not in used)
    for name, p in parts.items():
        m = getattr(p, "marker", None)
        if m is None or m.id != AUTO:
            continue
        group = getattr(p, "marker_clone_of", None)
        if group is not None and group in clones:
            m.id = clones[group]
            continue
        try:
            m.id = next(free)
        except StopIteration:
            raise SchemaError(f"part {name!r}: all {N_IDS} {DICTIONARY} marker ids are used; "
                              f"set `marker: none` on parts that need none") from None
        if group is not None:
            clones[group] = m.id


def ids(parts: dict[str, "Part"]) -> list[int]:
    """Marker ids used in the room, ascending."""
    return sorted({p.marker.id for p in parts.values() if getattr(p, "marker", None) is not None})


# ---- scene -----------------------------------------------------------------------
def _frame(normal: np.ndarray) -> list[float]:
    """Quaternion turning local +z to `normal`; the image's up is +y for a top face
    and world +z for a side face."""
    z = normal / np.linalg.norm(normal)
    up = np.array([0.0, 1.0, 0.0]) if abs(z[2]) > 0.5 else np.array([0.0, 0.0, 1.0])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.column_stack([x, y, z]).ravel())
    return q.tolist()


def geom_name(part: "Part", face: str) -> str:
    return f"{part.body_name}/marker/{face}"


def attach(spec: mujoco.MjSpec, part: "Part") -> None:
    """Add the marker texture and its face boxes to the part's body (after `build`)."""
    m = part.marker
    if m is None:
        return
    tex = f"aruco_{m.id:02d}"
    if not any(t.name == tex for t in spec.textures):
        spec.add_texture(name=tex, type=mujoco.mjtTexture.mjTEXTURE_2D, file=str(png_path(m.id)))
        mat = spec.add_material(name=tex)
        mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex
    anchor = spec.geom(part.body_name + part.marker_anchor)
    body = spec.body(part.body_name)
    half = (m.size * PRINT_CELLS / CELLS) / 2
    size = np.asarray(anchor.size, dtype=float)
    centre = np.asarray(anchor.pos, dtype=float)
    faces = ("front", "back") if m.face == "both" else (m.face,)
    for face in faces:
        if face == "top":
            top = size[1] if anchor.type == mujoco.mjtGeom.mjGEOM_CYLINDER else size[2]
            normal, offset = np.array([0.0, 0.0, 1.0]), top
        else:
            thin = 0 if size[0] <= size[1] else 1
            normal = np.zeros(3)
            normal[thin] = 1.0 if face == "front" else -1.0
            offset = size[thin]
        body.add_geom(name=geom_name(part, face), type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[half, half, HALF_THICK], pos=(centre + normal * (offset + LIFT)).tolist(),
                      quat=_frame(normal), material=tex, contype=0, conaffinity=0, mass=0.0)


def geom_ids(model: mujoco.MjModel, part: "Part") -> list[int]:
    m = part.marker
    if m is None:
        return []
    names = ("front", "back") if m.face == "both" else (m.face,)
    return [model.geom(geom_name(part, f)).id for f in names]


def face_poses(data: mujoco.MjData, gids: list[int]) -> list[tuple[int, np.ndarray, np.ndarray]]:
    """(geom id, world centre, outward normal) of marker face boxes `gids` (`geom_ids`)."""
    out = []
    for g in gids:
        rot = data.geom_xmat[g].reshape(3, 3)
        out.append((g, data.geom_xpos[g] + rot[:, 2] * HALF_THICK, rot[:, 2].copy()))
    return out


def disk_square(radius: float) -> float:
    """Side of the square inside a disk of `radius`."""
    return radius * math.sqrt(2.0)
