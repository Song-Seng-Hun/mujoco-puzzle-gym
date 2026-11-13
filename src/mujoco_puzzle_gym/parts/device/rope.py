"""Tow rope (R31): a rope between two knots that the walking body drags things with.

Structure. A `rope` part adds two knot parts `<rope>/a` and `<rope>/b` (kind "knot":
no sensor ever counts them, R4) and, once the body is in the scene, a limited
spatial tendon between the knots' sites (range 0..`length`: slack inside, a pull
beyond) and one inactive `connect` equality per knot and hitch, site to site.
MuJoCo cannot add bodies while running, so tying switches one of these on
(`eq_active`); `mj_getState` keeps it in snapshots. Tying moves the knot onto the
hitch site with zero velocity and then switches the equality on; untying switches it
off and lays the knot on the floor where it is.

Knots (`Knot`): a 20 g (`knot_mass`, no less: OQ9 stretch 0.35 cm at 20 g, 0.7 cm at
10 g, 2.5-2.8 cm at 2 g with a 0.3 kg sled) flat disc with a rod and a ring on top at
`handle_height` (the beak grabs the ring, R18; Open question 8 leaves the height open,
0.16 m is within `grab_radius` of the lowered beak tip at 0.135 m). Knots collide with
the floor only (explicit contact pairs with the floor plane, floor tiles, map ramps and
floor parts), not with legs, walls or objects.

Hitches (`Hitch`): the walking body's (`BodyAdapter.tow_site`, `a: body`), objects
with `hitch: true` (R17, R28; their colliding geoms get contact priority 2 so their own
`friction` is what drags), NPC robots (R29) and `anchor` posts (fixed). A room has at
most `max_hitches` (default 16; hitches x 2 knots x ropes equalities). Relay boxes (R34)
do not exist yet.

Tying and untying:
- reset: ends listed in `tied` (default: the ends whose `a` / `b` hitch is set) tie to
  that hitch; other ends lie at `rest` (default: `pos` -+ 0.4 `length` along x);
- channels `attach_a`, `detach_a`, `attach_b`, `detach_b` (rising edge): tie the end to
  its `a` / `b` hitch, or when unset to the nearest hitch within `hitch_radius` of the
  knot; untie it;
- hitch stations (`hitch_station`, hitch_station.py) tie the body to the nearest hitch
  within `hitch_radius` of the body's hitch, or untie the body's end;
- beak: a loose knot let go within `clip_radius` of a hitch ties to it; a tied knot held
  `untie_time` seconds comes loose (it stays in the beak).
A tie that would put the hitch farther than `length` (the run-time value now) from the
other end (its hitch, or the loose knot) does not happen and pulses
`<rope>.attach_failed` (also when no hitch is near enough).

Snapping: the tension is the tendon limit force (efc rows `mjCNSTR_LIMIT_TENDON`),
read every physics step. Above `max_tension` for `snap_time` seconds in a row, both
ends come loose and `<rope>.snapped` turns on (until reset); `on_snap: fail` ends the
episode. Defaults 0.9 N and 0.2 s (OQ9: dragging a 0.2 kg sled at friction 0.2 the
tension is 0.42-0.43 N median, 0.68 N at the 95th percentile; the start spikes to
1.2-1.7 N but stays above 1.5x for at most 0.07 s).

Length: `length` is a run-time value (R13 `bind` makes a winch). The tendon range moves
to it at no more than `reel_speed` m/s (a lever cell jump 0.4 -> 0.1 m at once gave a
0.3 kg sled over 200 N). Snapshots keep the current and the target length.

Teleports: warps and `next_stage` (R14) move everything tied to the body through ropes
along with it (ropes whose far end is an anchor come loose there first); a warp of an
object, a spring kick of an object, a dispenser recycle (park), a receptacle eject and
`set_addon` (R30, the body's end) untie that thing first (`World.untie`).

NPC leash (R29): while a rope is tied to an NPC robot its velocity actuators get zero
gains (it is dragged only; `ctrl = 0` would brake) and its `leash` limits are off. Both
follow the rope state on every reset, restore and tie change; they are not saved.
Untied outside its leash, the robot walks back in first.

Signals: `<rope>.attached_a`, `.attached_b`, `.tension` (0..1 of `max_tension`, mean of
the body step), `.snapped`, `.attach_failed` (pulse), and `body.towing` (1 while any
rope is tied to the body; written by the room's first rope). Observation item `tow`.

Static check (`mpg check` stage `parts`): `hitch_radius` and `clip_radius` at most
`length`; the default ends (with their random ranges) within `length`; a box the
`demo:` drags into a pit (`drag <rope> <pit>`) as tall as the pit is deep (rim to the
catch plane, `floor.CATCH_DEPTH`) within 1 cm, so its top is flush with the floor. Warnings: a hitch
object whose sliding friction x weight is above DRAG_WARN (OQ9: walking keeps 0.8 of its
speed up to about 0.4 N), and on the `demo:` way while towing (from `tow <rope> ...` to
`untow <rope>`, from the start for a rope tying the body to an object or NPC at reset,
and on `drag <rope> <part>` legs): a door narrower than the towed thing, or a bend
sharper than a right angle within `length` (the tendon is straight through walls; a
towed thing catches on corners).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from ...schema import Field, SchemaError
from ..base import Part

BODY_HITCH = "body"
BODY_TOWING = "body.towing"
ENDS = ("a", "b")
KNOT_R = 0.012            # [m] knot disc radius
KNOT_HZ = 0.006           # [m] knot disc half height (rests with its centre this high)
RING_R = 0.012            # [m] handle ring radius
KNOT_RGBA = (0.95, 0.8, 0.15, 1.0)
ROPE_RGBA = (0.35, 0.25, 0.15, 1.0)
PAIR_FRICTION = [1.0, 1.0, 0.005, 0.0001, 0.0001]
HITCH_PRIORITY = 2        # hitch objects' own friction wins over the floor's (priority 1)
DRAG_WARN = 0.4           # [N] OQ9: walking keeps >= 0.8 of its speed up to a steady ~0.4 N
G = 9.81
FLUSH_TOL = 0.01          # [m] a box dragged into a pit: its top this close to the floor
LOOSE_BOX = 1.0e3         # [m] leash limits while a rope holds an NPC (none in effect)


@dataclass
class Hitch:
    """A place a knot can tie to: `name` is a part name or "body"."""
    name: str
    kind: str              # body | object | npc | anchor
    site: str
    part: Any = None       # the owning part (None for the body)
    sid: int = -1
    rid: int = -1          # root body id (world 0 for anchors)

    @property
    def fixed(self) -> bool:
        return self.kind == "anchor"


def hitch_parts(parts: dict[str, Part]) -> list[Part]:
    """Parts a rope can tie to, room order: anchors, NPC robots, objects with `hitch: true`."""
    return [p for p in parts.values()
            if getattr(p, "is_anchor", False) or getattr(p, "kind", None) == "npc"
            or (getattr(p, "kind", None) == "object" and p.cfg.get("hitch"))]


class Knot(Part):
    """One end of a rope (added by the rope, not in room.yaml)."""
    type_name = "knot"
    kind = "knot"
    object_kind = "knot"
    stage = "device"
    #: R18 grab values (the beak holds the ring).
    grab_radius = 0.04
    max_hold_force = 3.0

    def __init__(self, name: str, rope: "Rope", end: int, xy):
        super().__init__(name, {"pos": [float(xy[0]), float(xy[1])]})
        self.rope = rope
        self.end = end
        self.mass = float(rope.cfg["knot_mass"])
        self.ring_z = float(rope.cfg["handle_height"]) - KNOT_HZ    # ring centre above the knot centre

    @property
    def site_name(self) -> str:
        return self.body_name + "/site"

    @property
    def geom_name(self) -> str:
        return self.body_name + "/disc"

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, KNOT_HZ])
        body.add_freejoint(name=self.body_name + "/free")
        body.add_geom(name=self.geom_name, type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[KNOT_R, KNOT_HZ, 0],
                      mass=self.mass, rgba=KNOT_RGBA, contype=0, conaffinity=0)
        body.add_site(name=self.site_name, size=[0.004, 0, 0])
        top = self.ring_z - RING_R
        vis = {"contype": 0, "conaffinity": 0, "mass": 0.0, "rgba": KNOT_RGBA}
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, KNOT_HZ, 0, 0, top],
                      size=[0.002, 0, 0], **vis)
        n = 8
        for k in range(n):                       # the ring, upright in the x-z plane
            a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
            p0 = [RING_R * math.sin(a0), 0, self.ring_z - RING_R * math.cos(a0)]
            p1 = [RING_R * math.sin(a1), 0, self.ring_z - RING_R * math.cos(a1)]
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[*p0, *p1], size=[0.002, 0, 0], **vis)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.body_jntadr[self._bid]
        self._q = int(model.jnt_qposadr[j])
        self._v = int(model.jnt_dofadr[j])
        self.sid = model.site(self.site_name).id

    def place(self, model, xy):
        self.xy = np.asarray(xy, dtype=float)

    # the beak's view (grab.py)
    @property
    def handle_body(self) -> str:
        return self.body_name

    @property
    def handle_bid(self) -> int:
        return self._bid

    def handle_points(self, data) -> np.ndarray:
        rot = data.xmat[self._bid].reshape(3, 3)
        return (data.xpos[self._bid] + rot[:, 2] * self.ring_z)[None, :]

    def put(self, data, xyz, quat=(1.0, 0.0, 0.0, 0.0)) -> None:
        data.qpos[self._q:self._q + 3] = xyz
        data.qpos[self._q + 3:self._q + 7] = quat
        data.qvel[self._v:self._v + 6] = 0.0

    def pos(self, data) -> np.ndarray:
        return data.xpos[self._bid].copy()


class Rope(Part):
    type_name = "rope"
    stage = "device"
    stateful = True               # its channels come from the physics, not this step's input
    has_body = False
    is_rope = True
    suffixes = (".attached_a", ".attached_b", ".tension", ".snapped", ".attach_failed")
    state_keys = ("tied", "cur", "over", "snapped", "held_t", "was_held", "hold_untied", "edges", "pending")
    fields = (
        Field("length", "float", 0.4, unit="m", min=0.05, max=5.0, runtime=True,
              doc="most distance between the knots (tendon range); bind it for a winch"),
        Field("reel_speed", "float", 0.05, unit="m/s", min=0.001, max=2.0,
              doc="the length moves to a new `length` at most this fast"),
        Field("a", "str", None, doc="hitch of end a: a part name or `body` (the walking body)"),
        Field("b", "str", None, doc="hitch of end b"),
        Field("tied", "list", None, doc="ends tied at reset, e.g. [a]; default: the ends with a hitch"),
        Field("rest", "any", None,
              doc="[[xa, ya], [xb, yb]]: where loose ends lie at reset; default: pos -+ 0.4 length along x"),
        Field("max_tension", "float", 0.9, unit="N", min=0.01, max=100.0,
              doc="above this for `snap_time` the rope snaps (both ends come loose)"),
        Field("snap_time", "float", 0.2, unit="s", min=0.0, max=10.0),
        Field("on_snap", "enum", "none", choices=("none", "fail"), doc="fail: snapping fails the episode"),
        Field("hitch_radius", "float", 0.3, unit="m", min=0.01, max=5.0,
              doc="channels and hitch stations tie to the nearest hitch this close (no `a` / `b`)"),
        Field("clip_radius", "float", 0.04, unit="m", min=0.005, max=1.0,
              doc="a knot the beak lets go this close to a hitch ties to it"),
        Field("untie_time", "float", 0.5, unit="s", min=0.05, max=10.0,
              doc="holding a tied knot this long unties it"),
        Field("knot_mass", "float", 0.02, unit="kg", min=0.02, max=0.5),
        Field("handle_height", "float", 0.16, unit="m", min=0.03, max=0.3,
              doc="knot ring height above the floor (the beak grabs it)"),
        Field("attach_a", "str", None, doc="channel: on its rising edge end a ties"),
        Field("detach_a", "str", None, doc="channel: on its rising edge end a comes loose"),
        Field("attach_b", "str", None, doc="channel: on its rising edge end b ties"),
        Field("detach_b", "str", None, doc="channel: on its rising edge end b comes loose"),
        Field("max_hitches", "int", 16, min=1, max=64, doc="most hitches a room with ropes may have"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        owner = f"part {name!r}"
        self.length0 = float(c["length"])
        self.ends_cfg = [c["a"], c["b"]]
        tied = c["tied"]
        if tied is None:
            tied = [e for e, h in zip(ENDS, self.ends_cfg) if h]
        if any(e not in ENDS for e in tied):
            raise SchemaError(f"{owner}: tied must list ends a / b, got {tied!r}")
        for e in tied:
            if not self.ends_cfg[ENDS.index(e)]:
                raise SchemaError(f"{owner}: end {e} is tied at reset but has no hitch (`{e}: <part or body>`)")
        self.tied0 = [e in tied for e in ENDS]
        rest = c["rest"]
        if rest is None:
            off = 0.4 * self.length0
            rest = [[self.base_xy[0] - off, self.base_xy[1]], [self.base_xy[0] + off, self.base_xy[1]]]
        ok = (isinstance(rest, (list, tuple)) and len(rest) == 2
              and all(isinstance(p, (list, tuple)) and len(p) == 2
                      and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in p) for p in rest))
        if not ok:
            raise SchemaError(f"{owner}: rest must be [[xa, ya], [xb, yb]], got {rest!r}")
        self.rest = np.asarray(rest, dtype=float)
        self.channels = {k: c[k] for k in ("attach_a", "detach_a", "attach_b", "detach_b") if c[k]}
        self.watch += [ch for ch in self.channels.values() if ch not in self.watch]
        self.knots = [Knot(f"{name}/{e}", self, i, self.rest[i]) for i, e in enumerate(ENDS)]
        self.writes_body = False
        self.hitches: list[Hitch] = []
        self._eq: list[list[int]] = [[], []]
        self.reset(None, None)

    @classmethod
    def shift_cfg(cls, cfg, dx, dy):
        out = super().shift_cfg(cfg, dx, dy)
        if isinstance(out.get("rest"), (list, tuple)):
            out["rest"] = [[float(p[0]) + dx, float(p[1]) + dy] if isinstance(p, (list, tuple)) and len(p) == 2
                           else p for p in out["rest"]]
        return out

    @classmethod
    def add_room_parts(cls, parts):
        """Add each rope's two knots after it, check hitch names and the hitch cap, and
        let the first rope write `body.towing`."""
        ropes = [p for p in parts.values() if isinstance(p, Rope)]
        if not ropes or any(k.name in parts for r in ropes for k in r.knots):
            return
        hitchable = {p.name for p in hitch_parts(parts)}
        for r in ropes:
            for e, h in zip(ENDS, r.ends_cfg):
                if h and h != BODY_HITCH and h not in hitchable:
                    raise SchemaError(f"part {r.name!r}: {e}: {h!r} is not a hitch (use `body`, an anchor, "
                                      f"an npc or an object with `hitch: true`; hitches: {sorted(hitchable)})")
        n = len(hitchable) + 1
        cap = min(r.cfg["max_hitches"] for r in ropes)
        if n > cap:
            raise SchemaError(f"room has {n} rope hitches (body, anchors, npcs, objects with hitch: true), "
                              f"more than max_hitches {cap}")
        rest = list(parts.items())
        parts.clear()
        for k, v in rest:
            parts[k] = v
            if isinstance(v, Rope):
                for kn in v.knots:
                    parts[kn.name] = kn
        ropes[0].writes_body = True
        ropes[0].outputs.append(BODY_TOWING)
        from ...grab import add_beak

        add_beak(parts)                    # knots are grabbable (their rings)

    # ---- scene ------------------------------------------------------------------
    def attach_body(self, spec, body):
        """Hitch sites, the tendon, the knots' floor contacts and one connect equality per
        knot and hitch (all inactive)."""
        parts = self.bus_parts
        hitches: list[Hitch] = []
        site = body.tow_site(spec)
        if site is not None:
            hitches.append(Hitch(BODY_HITCH, "body", site))
        names = {s.name for s in spec.sites}
        for p in hitch_parts(parts):
            sname = p.body_name + "/hitch"
            if sname not in names:
                off = p.hitch_offset() if hasattr(p, "hitch_offset") else [0.0, 0.0, 0.0]
                spec.body(p.body_name).add_site(name=sname, pos=list(off), size=[0.005, 0, 0], group=4)
                names.add(sname)
                if p.kind == "object":
                    for g in spec.body(p.body_name).geoms:
                        if g.contype or g.conaffinity:
                            g.priority = max(int(g.priority), HITCH_PRIORITY)
            kind = "anchor" if getattr(p, "is_anchor", False) else p.kind
            hitches.append(Hitch(p.name, kind, sname, part=p))
        self.hitches = hitches
        for e, h in zip(ENDS, self.ends_cfg):
            if h and h not in {x.name for x in hitches}:
                raise SchemaError(f"part {self.name!r}: {e}: {h!r}: this body has no tow hitch")
        floors = [g.name for g in spec.worldbody.geoms if g.name == "floor" or g.name.startswith("floor/")]
        for p in parts.values():
            if getattr(p, "floor_part", False) and p.has_body:
                floors += [g.name for g in spec.body(p.body_name).geoms if g.name]
        for kn in self.knots:
            for i, f in enumerate(floors):
                spec.add_pair(name=f"{kn.body_name}/floor{i}", geomname1=f, geomname2=kn.geom_name,
                              condim=3, friction=PAIR_FRICTION)
        t = spec.add_tendon(name=self.tendon_name, limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                            range=[0.0, self.length0], width=0.003, rgba=ROPE_RGBA)
        for kn in self.knots:
            t.wrap_site(kn.site_name)
        for e, kn in enumerate(self.knots):
            for h in hitches:
                spec.add_equality(name=self._eq_name(e, h), type=mujoco.mjtEq.mjEQ_CONNECT,
                                  objtype=mujoco.mjtObj.mjOBJ_SITE, name1=kn.site_name, name2=h.site,
                                  active=False)

    @property
    def tendon_name(self) -> str:
        return f"part/{self.name}/rope"

    def _eq_name(self, end: int, h: Hitch) -> str:
        return f"part/{self.name}/{ENDS[end]}@{h.name}"

    @property
    def bus_parts(self) -> dict[str, Part]:
        return {p.name: p for p in self.bus.order}

    def bind(self, model, data):
        super().bind(model, data)
        for h in self.hitches:
            h.sid = model.site(h.site).id
            h.rid = int(model.body_rootid[model.site_bodyid[h.sid]])
        self._tendon = model.tendon(self.tendon_name).id
        self._eq = [[model.equality(self._eq_name(e, h)).id for h in self.hitches] for e in range(2)]
        self._eq_all = np.array([i for row in self._eq for i in row], dtype=int)
        self._dt = float(model.opt.timestep)
        self._index = {h.name: i for i, h in enumerate(self.hitches)}
        self._body_h = self._index.get(BODY_HITCH)

    # ---- state ------------------------------------------------------------------
    def reset(self, model, data):
        self.tied: list[int | None] = [None, None]
        self.cur = float(self.live["length"]) if hasattr(self, "live") else 0.0
        self.over = 0.0
        self.snapped = False
        self.held_t = [0.0, 0.0]
        self.was_held = [False, False]
        self.hold_untied = [False, False]   # untied by this hold: letting go does not clip it again
        self.edges = {k: False for k in self.channels}
        self.pending = 0.0           # attach_failed pulse to write this step
        self._acc, self._n = 0.0, 0
        self.tension = 0.0
        if model is None:
            return
        data.eq_active[self._eq_all] = 0
        model.tendon_range[self._tendon] = [0.0, self.cur]
        for kn in self.knots:
            kn.xy = self.rest[kn.end].copy()
            self._lay(data, kn, self.rest[kn.end])

    def after_reset(self, model, data):
        """Tie the default ends (all parts are in place now), then the NPC leash rules."""
        mujoco.mj_kinematics(model, data)
        for e in range(2):
            if self.tied0[e]:
                self.tie(model, data, e, self._index[self.ends_cfg[e]], check=False)
        sync_npcs(self.world, model, data)
        self._write()

    def load_state(self, model, data, st):
        """Snapshots keep the current length here and the target in `live` (R13)."""
        super().load_state(model, data, st)
        self._acc, self._n = 0.0, 0
        model.tendon_range[self._tendon] = [0.0, self.cur]
        sync_npcs(self.world, model, data)

    # ---- tying ------------------------------------------------------------------
    def tied_hitch(self, end: int) -> Hitch | None:
        i = self.tied[end]
        return None if i is None else self.hitches[i]

    def _end_point(self, data, end: int) -> np.ndarray:
        """World point of end `end`: its hitch site when tied, else its knot."""
        h = self.tied_hitch(end)
        return data.site_xpos[h.sid].copy() if h is not None else data.site_xpos[self.knots[end].sid].copy()

    def _lay(self, data, kn: Knot, xy) -> None:
        """Knot `kn` loose on the floor at `xy`, upright, at rest."""
        fh = self.world.floor_height if self.world is not None else None
        top = (fh(xy)[0] if fh is not None else 0.0) or 0.0
        kn.put(data, [float(xy[0]), float(xy[1]), top + KNOT_HZ])

    def tie(self, model, data, end: int, h: int, check: bool = True) -> bool:
        """Tie end `end` to hitch index `h`: knot onto the hitch site, velocity 0, then the
        equality on. With `check`, a hitch farther than the length now from the other end
        fails (`attach_failed`)."""
        p = data.site_xpos[self.hitches[h].sid].copy()
        if check:
            other = self._end_point(data, 1 - end)
            if float(np.linalg.norm(p - other)) > self.cur + 1e-6:
                self.pending = 1.0
                return False
        old = self.tied[end]
        if old is not None:
            data.eq_active[self._eq[end][old]] = 0
        self.knots[end].put(data, p)
        data.eq_active[self._eq[end][h]] = 1
        self.tied[end] = h
        if self.hitches[h].kind == "npc" or (old is not None and self.hitches[old].kind == "npc"):
            sync_npcs(self.world, model, data)
        return True

    def untie(self, model, data, end: int, lay: bool = True) -> None:
        """End `end` loose; with `lay`, its knot lies on the floor where it is."""
        old = self.tied[end]
        if old is None:
            return
        data.eq_active[self._eq[end][old]] = 0
        self.tied[end] = None
        if lay:
            kn = self.knots[end]
            self._lay(data, kn, data.xpos[kn._bid][:2].copy())
        if self.hitches[old].kind == "npc":
            sync_npcs(self.world, model, data)

    def untie_body(self, model, data, rid: int) -> None:
        for e in range(2):
            h = self.tied_hitch(e)
            if h is not None and h.rid == rid:
                self.untie(model, data, e)

    def nearest(self, data, xy3, radius: float, skip: set[int]) -> int | None:
        best, bd = None, radius
        for i, h in enumerate(self.hitches):
            if i in skip:
                continue
            d = float(np.linalg.norm(data.site_xpos[h.sid] - xy3))
            if d <= bd:
                best, bd = i, d
        return best

    def attach_end(self, model, data, end: int) -> bool:
        """Channel `attach_<end>`: tie to the `a` / `b` hitch, else the nearest within
        `hitch_radius` of the knot (not the other end's hitch)."""
        name = self.ends_cfg[end]
        other = self.tied[1 - end]
        if name:
            h = self._index[name]
        else:
            h = self.nearest(data, data.site_xpos[self.knots[end].sid], float(self.cfg["hitch_radius"]),
                             {other} if other is not None else set())
        if h is None or h == other:
            self.pending = 1.0
            return False
        if self.tied[end] == h:
            return True
        return self.tie(model, data, end, h)

    def station_attach(self, model, data) -> bool:
        """Hitch station: tie the body's hitch and the nearest other hitch within
        `hitch_radius` of it; the end already on that hitch stays."""
        b = self._body_h
        if b is None:
            return False
        dp = data.site_xpos[self.hitches[b].sid]
        t = self.nearest(data, dp, float(self.cfg["hitch_radius"]), {b})
        if t is None or float(np.linalg.norm(data.site_xpos[self.hitches[t].sid] - dp)) > self.cur + 1e-6:
            self.pending = 1.0
            return False
        te = self.tied.index(t) if t in self.tied else 1
        be = 1 - te
        if self.tied[te] != t:
            self.tie(model, data, te, t, check=False)
        if self.tied[be] != b:
            self.tie(model, data, be, b, check=False)
        return True

    def station_detach(self, model, data) -> bool:
        if self._body_h is None or self._body_h not in self.tied:
            return False
        self.untie(model, data, self.tied.index(self._body_h))
        return True

    @property
    def on_body(self) -> bool:
        return self._body_h is not None and self._body_h in self.tied

    # ---- running ------------------------------------------------------------------
    def tension_now(self, data) -> float:
        n = data.nefc
        if n == 0:
            return 0.0
        sel = (data.efc_type[:n] == mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON) & (data.efc_id[:n] == self._tendon)
        return float(np.abs(data.efc_force[:n][sel]).sum())

    def substep(self, model, data):
        t = self.tension_now(data)
        self._acc += t
        self._n += 1
        self.over = self.over + self._dt if t > float(self.cfg["max_tension"]) else 0.0

    def update(self, model, data, body):
        dt = self.bus.dt
        target = float(self.live["length"])          # reel towards it (R13 bind: a winch)
        step = float(self.cfg["reel_speed"]) * dt
        self.cur = target if abs(target - self.cur) <= step else self.cur + math.copysign(step, target - self.cur)
        model.tendon_range[self._tendon] = [0.0, self.cur]
        for key, ch in self.channels.items():
            on = self.bus.on(ch)
            if on and not self.edges[key]:
                end = ENDS.index(key[-1])
                if key.startswith("attach"):
                    self.attach_end(model, data, end)
                else:
                    self.untie(model, data, end)
            self.edges[key] = on
        self._beak(model, data)
        self.tension = self._acc / self._n if self._n else 0.0
        self._acc, self._n = 0.0, 0
        if self.over >= float(self.cfg["snap_time"]) and (self.tied[0] is not None or self.tied[1] is not None):
            self.untie(model, data, 0)
            self.untie(model, data, 1)
            self.snapped = True
            self.over = 0.0
        self._write()

    def _beak(self, model, data) -> None:
        w = self.world
        held = w.held if w is not None else set()
        for e, kn in enumerate(self.knots):
            now = int(model.body_rootid[kn._bid]) in held
            if now and self.tied[e] is not None:
                self.held_t[e] += self.bus.dt
                if self.held_t[e] >= float(self.cfg["untie_time"]) - 1e-9:
                    self.untie(model, data, e, lay=False)       # stays in the beak
                    self.held_t[e] = 0.0
                    self.hold_untied[e] = True
            else:
                self.held_t[e] = 0.0
            if self.was_held[e] and not now and self.tied[e] is None and not self.hold_untied[e]:
                other = self.tied[1 - e]
                h = self.nearest(data, data.site_xpos[kn.sid], float(self.cfg["clip_radius"]),
                                 {other} if other is not None else set())
                if h is not None:
                    self.tie(model, data, e, h)
            if not now:
                self.hold_untied[e] = False
            self.was_held[e] = now

    def _write(self) -> None:
        self.write(float(self.tied[0] is not None), self.name + ".attached_a")
        self.write(float(self.tied[1] is not None), self.name + ".attached_b")
        self.write(float(np.clip(self.tension / float(self.cfg["max_tension"]), 0.0, 1.0)), self.name + ".tension")
        self.write(float(self.snapped), self.name + ".snapped")
        self.write(self.pending, self.name + ".attach_failed")
        self.pending = 0.0
        if self.writes_body and self.world is not None:
            self.write(float(any(r.on_body for r in self.world.ropes)), BODY_TOWING)

    def state(self):
        return 0.5 * sum(t is not None for t in self.tied)

    def failed(self):
        return self.snapped and self.cfg["on_snap"] == "fail"

    # ---- checks -----------------------------------------------------------------
    def _hitch_box(self, env, end: int) -> tuple[np.ndarray, np.ndarray]:
        """(nominal world point, xy random half-range) of end `end` at reset."""
        d = env.data
        if self.tied0[end]:
            h = self.hitches[self._index[self.ends_cfg[end]]]
            p = d.site_xpos[h.sid].copy()
            if h.kind == "body":
                rnd = np.asarray(env.room.spawn_random[:2], dtype=float)
            else:
                rnd = np.asarray(h.part.random, dtype=float)
            return p, rnd
        top = env.floor_height(self.rest[end])[0] or 0.0
        return np.array([*self.rest[end], top + KNOT_HZ]), np.zeros(2)

    def check_static(self, env) -> list[str]:
        c = self.cfg
        L = float(c["length"])
        out = []
        for key in ("hitch_radius", "clip_radius"):
            if float(c[key]) > L:
                out.append(f"rope {self.name!r}: {key} {c[key]:g} m is longer than its length {L:g} m")
        env.reset(options={"layout": env.sample_layout(None)})
        (pa, ra), (pb, rb) = self._hitch_box(env, 0), self._hitch_box(env, 1)
        dxy = np.abs(pa[:2] - pb[:2]) + ra + rb
        worst = float(math.sqrt(float(dxy @ dxy) + (pa[2] - pb[2]) ** 2))
        if worst > L + 1e-6:
            out.append(f"rope {self.name!r}: its ends at reset can be {worst:.2f} m apart (random ranges "
                       f"included), longer than its length {L:g} m")
        return out + pit_fill_problems(env, self)

    def check_warnings(self, env) -> list[str]:
        out = []
        if self.writes_body:
            for p in hitch_parts(env.parts):
                if p.kind == "object":
                    f = float(p.cfg.get("friction", 1.0)) * float(p.mass) * G
                    if f > DRAG_WARN:
                        out.append(f"part {p.name!r}: dragging it takes about {f:.2f} N (friction x weight), "
                                   f"above {DRAG_WARN:g} N; the walking body slows below 0.8 of its speed (OQ9)")
        out += tow_path_warnings(env, self)
        return out


# ---- NPC leash (R29) --------------------------------------------------------------
def sync_npcs(world, model, data) -> None:
    """Every NPC robot a rope is tied to: zero gains and no leash limits; the others back
    to their own (`Npc.set_towed`)."""
    if world is None:
        return
    towed: dict[str, Any] = {}
    for r in world.ropes:
        for h in r.hitches:
            if h.kind == "npc":
                towed.setdefault(h.name, [h.part, False])
                if h.name in {r.hitches[i].name for i in r.tied if i is not None}:
                    towed[h.name][1] = True
    for part, on in towed.values():
        part.set_towed(model, data, on)


# ---- warps (R14) ------------------------------------------------------------------
def warp_bundle(env, key, pose, warp_one) -> None:
    """Warp `key` to `pose` with ropes: an object comes loose first; the body takes along
    everything tied to it through ropes (anchored ends come loose), each keeping its pose
    relative to the body (`warp_one` moves the body itself)."""
    m, d, w = env.model, env.data, env.world
    if key != "body":
        w.untie(m, d, int(key))
        warp_one(key, pose)
        return
    body_rid = int(m.body_rootid[env.body.root_body_id(m)])
    seen, frontier, ropes = {body_rid}, [body_rid], []
    while frontier:
        rid = frontier.pop()
        for r in w.ropes:
            for e in range(2):
                h = r.tied_hitch(e)
                if h is None or h.rid != rid:
                    continue
                if r not in ropes:
                    ropes.append(r)
                o = r.tied_hitch(1 - e)
                if o is None:
                    continue
                if o.fixed:
                    r.untie(m, d, 1 - e)
                elif o.rid not in seen:
                    seen.add(o.rid)
                    frontier.append(o.rid)
    if not ropes:
        warp_one(key, pose)
        return
    s0 = env.read_body()
    c0, yaw0 = s0.pos[:2].copy(), float(s0.yaw)
    movers: list[tuple[int, Any]] = [(rid, w.objects.get(rid)) for rid in seen - {body_rid}]
    movers += [(int(m.body_rootid[kn._bid]), kn) for r in ropes for kn in r.knots]
    rec = []
    for rid, owner in movers:
        if rid in w.held:
            continue
        if getattr(owner, "planar", False):
            rec.append((rid, owner, owner.pos(d), owner.yaw(d), None))
        else:
            jnt = int(m.body_jntadr[rid])
            qa = int(m.jnt_qposadr[jnt])
            rec.append((rid, owner, d.qpos[qa:qa + 3].copy(), None, d.qpos[qa + 3:qa + 7].copy()))
    warp_one(key, pose)
    dyaw = float(pose[2]) - yaw0
    cz, sz = math.cos(dyaw), math.sin(dyaw)
    rot = np.array([[cz, -sz], [sz, cz]])
    qz = np.array([math.cos(dyaw / 2), 0.0, 0.0, math.sin(dyaw / 2)])
    c1 = np.asarray(pose[:2], dtype=float)
    for rid, owner, p, yaw, quat in rec:
        xy = c1 + rot @ (np.asarray(p[:2]) - c0)
        if quat is None:
            owner.teleport(d, (float(xy[0]), float(xy[1]), yaw + dyaw))
            continue
        top_o, rim_o = env.floor_height(p[:2])
        top_n, rim_n = env.floor_height(xy)
        z = (top_n if top_n is not None else rim_n) + p[2] - (top_o if top_o is not None else rim_o)
        jnt = int(m.body_jntadr[rid])
        qa, va = int(m.jnt_qposadr[jnt]), int(m.jnt_dofadr[jnt])
        d.qpos[qa:qa + 3] = [xy[0], xy[1], z]
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, qz, quat)
        d.qpos[qa + 3:qa + 7] = q
        d.qvel[va:va + 6] = 0.0
    mujoco.mj_kinematics(m, d)
    for r in ropes:                       # tied knots exactly on their hitches again
        for e in range(2):
            h = r.tied_hitch(e)
            if h is not None:
                r.knots[e].put(d, d.site_xpos[h.sid].copy())
    mujoco.mj_forward(m, d)


# ---- static warnings on the demo way ------------------------------------------------
def tow_path_warnings(env, rope: Rope) -> list[str]:
    """Doors narrower than the towed thing and bends sharper than a right angle within
    `length` on the `demo:` way while the rope tows: from `tow <rope> ...` (or from the
    start when the rope ties the body to an object or NPC at reset) to `untow <rope>`,
    and on every `drag <rope> <part>` leg (towing the rope's configured end that is not
    the body when no `tow` named one)."""
    demo = list(getattr(env.room, "demo", []) or [])
    start = _start_towed(env, rope)
    if start is None and not any(e[0] in ("tow", "drag") and e[1] == rope.name for e in demo):
        return []
    plain = [h for h in rope.ends_cfg if h and h != BODY_HITCH]
    from ...logic import free_mask

    grid, parts = env.grid, env.parts
    on = {p.name for p in parts.values() if p.role in ("switch", "warp")}
    free = free_mask(grid, env.walls, parts, on, inflate=env.body.footprint_radius)
    legs: list[tuple[np.ndarray, np.ndarray, str]] = []   # (from, to, towed hitch name)
    here = np.asarray(env.room.spawn_pos, dtype=float)
    towing: str | None = start
    for e in demo:
        if e[0] == "tow" and e[1] == rope.name:
            st = _station_for(parts, rope, e[2])
            to = st.xy if st is not None else parts[e[2]].base_xy
            towing = e[2]
            here = np.asarray(to, dtype=float)
            continue
        if e[0] == "untow" and e[1] == rope.name:
            towing = None
            continue
        if e[0] == "drag" and e[1] == rope.name and e[2] in parts:
            to = np.asarray(parts[e[2]].xy, dtype=float)
            towed = towing or (plain[-1] if plain else None)
            if towed is not None:
                legs.append((here, to, towed))
            here = to
            continue
        if e[0] in ("goto", "press") and e[1] in parts:
            to = np.asarray(parts[e[1]].target_xy(), dtype=float)
            if towing is not None:
                legs.append((here, to, towing))
            here = to
    out = []
    L = float(rope.cfg["length"])
    for a, b, towed in legs:
        path = _grid_path(grid, free, a, b)
        if len(path) < 2:
            continue
        width = _width(env, parts.get(towed))
        for g in parts.values():
            gw = g.cfg.get("width") if g.role == "gate" else None
            if gw is None or width <= float(gw):
                continue
            segs = g.blockers(False)
            if any(_near_segment(path, s, 0.1) for s in segs):
                out.append(f"rope {rope.name!r}: towed {towed!r} ({width:.2f} m wide) passes door {g.name!r} "
                           f"({float(gw):.2f} m wide) on the demo way")
        sharp = _sharp_bend(path, L)
        if sharp is not None:
            out.append(f"rope {rope.name!r}: the demo way bends more than 90 deg within the rope length "
                       f"{L:g} m near ({sharp[0]:.2f}, {sharp[1]:.2f}); the towed {towed!r} catches on the corner")
    return out


def _start_towed(env, rope: Rope) -> str | None:
    """The part the rope tows from the start: its other end when it ties the body to an
    object or NPC at reset (not an anchor), else None."""
    ends = rope.ends_cfg
    if BODY_HITCH not in ends:
        return None
    e = ends.index(BODY_HITCH)
    other = ends[1 - e]
    if rope.tied0[e] and rope.tied0[1 - e] and not getattr(env.parts.get(other), "is_anchor", False):
        return other
    return None


def pit_fill_problems(env, rope: Rope) -> list[str]:
    """A box the `demo:` drags into a pit (`drag <rope> <pit>`) must fill it flush: its
    height equal to the pit's depth (rim to the catch plane) within FLUSH_TOL, else the
    walk across it steps down or up (R31 `pit_bridge`). The towed box is the last `tow`
    hitch, else the rope's configured end that is not the body."""
    from ...floor import CATCH_DEPTH
    from ..trap.pit import Pit

    demo = list(getattr(env.room, "demo", []) or [])
    plain = [h for h in rope.ends_cfg if h and h != BODY_HITCH]
    towing = _start_towed(env, rope)
    out = []
    for e in demo:
        if e[1:2] != (rope.name,):
            continue
        if e[0] == "tow":
            towing = e[2]
        elif e[0] == "untow":
            towing = None
        elif e[0] == "drag" and isinstance(env.parts.get(e[2]), Pit):
            towed = towing or (plain[-1] if plain else None)
            box = env.parts.get(towed) if towed is not None else None
            if box is None or not box.has_body:
                continue
            pit = env.parts[e[2]]
            depth = float(pit.height_at(pit.xy)[1]) + CATCH_DEPTH
            h = _height(env, box)
            if abs(h - depth) > FLUSH_TOL:
                out.append(f"rope {rope.name!r}: {towed!r} ({h:.2f} m high) dragged into pit {pit.name!r} "
                           f"({depth:.2f} m deep) leaves its top {abs(depth - h):.2f} m "
                           f"{'below' if h < depth else 'above'} the floor; a box filling a pit must be "
                           f"flush within {FLUSH_TOL * 100:.0f} cm")
    return out


def _height(env, part) -> float:
    """Height [m] of `part`'s colliding geoms (body frame z extent, 0 if none)."""
    m = env.model
    lo, hi = math.inf, -math.inf
    for g in np.nonzero(m.geom_bodyid == part._bid)[0]:
        if m.geom_contype[g] or m.geom_conaffinity[g]:
            c = float(m.geom_pos[g][2] + m.geom_aabb[g][2])
            lo, hi = min(lo, c - float(m.geom_aabb[g][5])), max(hi, c + float(m.geom_aabb[g][5]))
    return hi - lo if hi > lo else 0.0


def _station_for(parts, rope: Rope, hitch: str):
    sts = [p for p in parts.values() if getattr(p, "rope_name", None) == rope.name
           and p.mode in ("attach", "toggle")]
    if not sts:
        return None
    if hitch in parts:
        hx = parts[hitch].base_xy
        sts.sort(key=lambda s: float(np.linalg.norm(s.base_xy - hx)))
    return sts[0]


def _grid_path(grid, free, a, b) -> np.ndarray:
    field = grid.distance(free, grid.disk(b, 0.06))
    pts = [np.asarray(a, dtype=float)]
    for _ in range(2000):
        nxt = grid.descend(field, pts[-1], grid.res * 2)
        if np.linalg.norm(nxt - pts[-1]) < 1e-9:
            break
        pts.append(nxt)
        if grid.lookup(field, nxt) <= 0.0:
            break
    return np.array(pts)


def _width(env, part) -> float:
    """Widest horizontal extent [m] of `part`'s colliding geoms (0 if none)."""
    if part is None or not part.has_body or part._bid < 0:
        return 0.0
    m = env.model
    w = 0.0
    for g in np.nonzero(m.geom_bodyid == part._bid)[0]:
        if m.geom_contype[g] or m.geom_conaffinity[g]:
            w = max(w, 2.0 * float(max(m.geom_aabb[g][3], m.geom_aabb[g][4])))
    return w


def _near_segment(path, seg, tol: float) -> bool:
    p0, p1 = np.asarray(seg.p0, float), np.asarray(seg.p1, float)
    d = p1 - p0
    n = float(d @ d)
    t = np.clip(((path - p0) @ d) / n, 0.0, 1.0) if n > 0 else np.zeros(len(path))
    dist = np.linalg.norm(path - (p0 + t[:, None] * d), axis=1)
    return bool((dist <= seg.thickness / 2 + tol).any())


def _sharp_bend(path, L: float):
    """A path point where the heading over the last `L` and the next `L` differ by more
    than 90 deg, or None."""
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 2 * L:
        return None
    for i in range(len(path)):
        if s[i] < L or s[-1] - s[i] < L:
            continue
        j = int(np.searchsorted(s, s[i] - L))
        k = int(np.searchsorted(s, s[i] + L))
        u, v = path[i] - path[j], path[min(k, len(path) - 1)] - path[i]
        if np.linalg.norm(u) < 1e-6 or np.linalg.norm(v) < 1e-6:
            continue
        if float(u @ v) < 0.0:
            return path[i]
    return None
