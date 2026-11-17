"""Stack (R71): 1 while `need` accepted objects are stacked on this patch of floor.

Designed in M4 and built in M5. Stacking itself already worked - `cube` supports one cube
resting on another - and what was missing was a part that *reads* a stack: every receiving
part in the package reads "went in" (a box, a socket, a basket, a scale) and none reads a
height, so nothing could ask for one. `mpg parts` had no part taking a `height` at all.

On when all of these hold:

- at least `need` accepted objects have their centres inside `size` (a floor rectangle);
- the highest of them has its centre at least `height` above the floor, which is what
  separates a stack from a row: three cubes side by side fill the area and reach nothing;
- none of them has moved faster than `REST_SPEED` for `settle` seconds, so a tower being
  built - or collapsing - does not read as built.

`accept` names what counts, the way a receptacle's does: object part names, object kinds or
marker ids, and empty accepts every object. Objects the body holds never count, like every
other receiving part - carrying two cubes past the patch is not a stack.

The field is `accept` and not `accepts` because `accepts(obj)` is already a protocol: the
grab code asks a receiving part whether it takes an object (`grabbable.takes`). The first
draft of this part named its field `accepts` and shadowed that method with a list, which
broke loading any room holding a cube - caught at once, by a room that would not build.

Channels: its own (or `out`) 0/1, and `<name>.count`, how many accepted objects are inside
the area right now, settled or not - a room can light a cue at two and win at three.

No `needs_body`: this part only looks. What puts the objects there is the room's business,
and a body with no grab simply never satisfies it - which the capability gate cannot say
for it, because reading is not a capability.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field
from ..base import Part
from ._sense import candidates, in_rect

MAT_THICK = 0.002     # the mat is a mark, not a surface: objects rest on the floor
OFF_RGBA = (0.45, 0.45, 0.5, 0.7)
ON_RGBA = (0.2, 0.8, 0.35, 0.85)
REST_SPEED = 0.02     # [m/s] an object this slow counts as resting (carry.py uses the same)


class Stack(Part):
    role = "sensor"
    marked = True
    marker_anchor = "/mat"
    marker_faces = markers.FACES_FLAT
    stage = "input"
    emits = True
    suffixes = (".count",)
    state_keys = ("on", "count", "_since")
    fields = (
        Field("size", "vec", [0.2, 0.2], length=2, unit="m", min=0.05, max=2.0,
              doc="floor rectangle the stack is counted on"),
        Field("need", "int", 3, min=2, max=10, runtime=True,
              doc="accepted objects that must be stacked"),
        Field("height", "float", 0.09, unit="m", min=0.02, max=1.0, runtime=True,
              doc="the topmost object's centre must be at least this high: a row is not a stack"),
        Field("accept", "list", None,
              doc="object part names, kinds or marker ids that count (empty: every object)"),
        Field("settle", "float", 0.3, unit="s", min=0.0, max=5.0,
              doc="everything counted must rest this long before it reads on"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"]))

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], dtype=float) / 2
        self.accept = list(self.cfg["accept"] or [])
        self.on = False
        self.count = 0
        self._since = None                      # `data.time` the stack has rested since
        self._data = None                       # last `update`'s data, for the place helpers
        self._on_out = self.outputs[:-1]        # the 0/1 channels (the last is `.count`)

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/mat", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, MAT_THICK / 2], pos=[0, 0, MAT_THICK / 2],
                      rgba=OFF_RGBA, contype=0, conaffinity=0)
        return body

    def reset(self, model, data):
        self.on = False
        self.count = 0
        self._since = None

    def update(self, model, data, body):
        self._data = data        # the place helpers are asked outside `update` (carry.py)
        if not self.enabled():
            self.on = False
            for ch in self._on_out:
                self.write(0.0, ch)
            self.write(0.0, self.outputs[-1])
            return
        here = [(rid, pos) for rid, pos in candidates(self, data, body, "object",
                                                      self.accept or None)
                if in_rect(pos, self.xy, self.half)]
        self.count = len(here)
        resting = all(float(np.linalg.norm(data.cvel[rid][3:])) < REST_SPEED for rid, _ in here)
        top = max((float(pos[2]) for _, pos in here), default=0.0)
        enough = self.count >= int(self.live["need"]) and top >= float(self.live["height"])
        now = float(data.time)
        if enough and resting:
            self._since = now if self._since is None else self._since
        else:
            self._since = None
        self.on = self._since is not None and now - self._since >= float(self.cfg["settle"])
        for ch in self._on_out:
            self.write(float(self.on), ch)
        self.write(float(self.count), self.outputs[-1])
        self._paint(model)

    def _paint(self, model) -> None:
        model.geom_rgba[model.geom(self.body_name + "/mat").id] = ON_RGBA if self.on else OFF_RGBA

    def state(self):
        """One number, like every other input part: the observation reads a scalar per part
        (`obs.part_state`). The count goes out on the `.count` channel, not here."""
        return float(self.on)

    def satisfied(self):
        return self.on

    def target_mask(self, grid):
        return grid.rect(self.xy, self.half)

    def check_static(self, env) -> list[str]:
        """A `height` the room's own objects cannot reach, which makes the room unsolvable.

        The top of a tower of `need` cubes of edge `e` has its centre at `(need - 0.5) * e`,
        so a patch asking for more than that can never read on however well the body
        stacks. Measured against the objects this part accepts, since a room may hold cubes
        of several sizes. Said rather than warned: `mpg check` calls a room unsolvable only
        after the solver has failed twenty times otherwise, and the reason is this number.
        """
        sizes = [float(getattr(p, "size", 0.0)) for p in env.parts.values()
                 if getattr(p, "object_kind", None) is not None and self.accepts(p)]
        best = max(sizes, default=0.0)
        need, height = int(self.live["need"]), float(self.live["height"])
        if best and height > (need - 0.5) * best + 1e-9:
            return [f"part {self.name!r}: height {height:g} m is more than {need} objects of "
                    f"{best:g} m reach when stacked ({(need - 0.5) * best:.3f} m to the top "
                    f"one's centre)"]
        return []

    def accepts(self, obj: Part) -> bool:
        """The receiving-part protocol (`grabbable.takes`): does this object count here."""
        if not self.accept:
            return True
        mid = obj.marker.id if getattr(obj, "marker", None) is not None else None
        return any(e == obj.name or e == getattr(obj, "object_kind", None)
                   or (type(e) is int and e == mid) for e in self.accept)

    def _top_of(self, obj):
        """((x, y), centre z) of the highest thing already counted here, or None."""
        best = None
        for rid, pos in candidates(self, self._data, None, "object", self.accept or None):
            if in_rect(pos, self.xy, self.half) and (best is None or pos[2] > best[1]):
                best = (np.asarray(pos[:2], dtype=float), float(pos[2]))
        return best

    def place_xy(self, obj):
        """Where a place aims: the top of the stack if there is one, else the patch centre.
        The centre is the wrong target for the second object - a stack is built on what is
        already there, and aiming at the middle of a wide patch put the two side by side,
        0.083 m apart, which this part reads as a row and refuses."""
        top = self._top_of(obj) if self._data is not None else None
        return top[0] if top is not None else np.asarray(self.xy, dtype=float)

    def place_height(self, obj) -> float:
        """The height to let go at [m]: on top of what is already here, else the floor."""
        size = float(getattr(obj, "size", 0.03))
        top = self._top_of(obj) if self._data is not None else None
        return (top[1] + size) if top is not None else size / 2

    def place_tolerance(self, obj) -> float:
        """Where the scripted place lets go: anywhere inside the patch, less a margin for
        the object sliding after. A stack is built by dropping each one on the same spot,
        so this is the patch's own half-width rather than a fixed tolerance."""
        return float(max(0.01, min(self.half) - 0.02))
