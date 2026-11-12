"""NPC robot (R29): a small robot that slides over the floor by itself.

Body: a box chassis (`size` [x, y, z], `mass`) `CLEAR` above the floor on three
joints of its root body: slide x, slide y (world axes) and a z hinge, each
driven by a velocity actuator (force capped at `mass * FORCE_ACC`). It has no
vertical joint, so it never sinks or tips: it stays at the floor height under
its reset position (raised map cells, floor tiles) and a step is a wall to it.
It collides with walls, the walking body, objects and other robots. Buttons, zones and lasers see its
centre like an object's (sensor `trigger: npc | any`); a pressure plate counts
its `mass` while its centre is over the plate (input/pressure.py), since a
robot without a vertical joint puts no weight on the floor. It carries an
ArUco marker on top (R26).

Every body step its behaviour rule (behaviors.py, `behavior`) gives a point
or a direction and a speed (at most `speed`, run-time). The robot then
- casts five horizontal rays (0, +-35, +-70 deg off the wanted heading, `mj_ray`)
  and takes the first clear one (0 first, then the side with more room), so a
  blocked robot slides along a wall; none clear: it stops;
- keeps its centre inside `leash` [x0, y0, x1, y1] (world): directions leaving
  it count as blocked, and the slide joints are limited to the box, so a push
  cannot take it out either;
- changes its velocity by at most `accel` per second and turns to face where it goes.
`enable` off: it brakes to a stop where it is (the behaviour pauses).

Channels: `<name>.moving` (1 while it moves faster than MOVING m/s) and
`<name>.on` (marker id of the first button, room order, its centre is over;
-1 on none or on a button without a marker). Observation item `npcs`.
`speed` at or above the walking body's speed (DUCK_SPEED) is a `mpg check`
warning: the body could not catch a fleeing robot.

Snapshots (R6) hold the joints in the MuJoCo state, the leash limits
(`jnt_range`, snapshot.MODEL_FIELDS) and the rule memory in `state_keys`.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ... import markers
from ...schema import Field, SchemaError
from ..base import TRIGGERS, Part
from ..input._sense import in_rect
from . import behaviors

CLEAR = 0.006          # [m] chassis bottom above the floor (clears 4 mm pressure plates)
FORCE_ACC = 5.0        # [m/s^2] actuator force cap = mass * this
KV_RATE = 30.0         # [1/s] velocity actuator gain = mass (inertia) * this
TURN_GAIN = 4.0        # [1/s] yaw rate per rad of heading error
MAX_TURN = 3.0         # [rad/s]
RAY_ANGLES = (35.0, 70.0)
LOOK = 0.05            # [m] ray clearance needed beyond the chassis half diagonal
LEASH_LOOK = 0.03      # [m] a direction is in the leash if this far along it is
LEASH_SOLREF = [0.01, 1.0]   # stiff leash limits (2 physics steps): a hard shove stays inside
RAY_RANGE = 1.0        # [m] ray length (free distance compared between the sides)
MOVING = 0.01          # [m/s]
DUCK_SPEED = 0.16      # [m/s] walking body speed (spec R29 "about 0.16 m/s")
BODY_RGBA = (0.3, 0.45, 0.85, 1.0)
WHEEL_RGBA = (0.15, 0.15, 0.15, 1.0)
SEEN = ".seen"


class Npc(Part):
    type_name = "npc"
    kind = "npc"
    stage = "device"
    stateful = True              # its channels come from the robot's motion, not this step's input
    marked = True
    marker_anchor = "/chassis"
    marker_faces = markers.FACES_FLAT
    suffixes = (".moving", ".on")
    floor_z = 0.0                # floor top under `pos`, set by the env before reset (R10/R37 heights)
    planar = True                # env warps / conveyors / springs move its slide joints
    state_keys = ("cmd", "wp", "timer", "acting", "on_id", "now")
    fields = (
        Field("behavior", "enum", "flee", choices=tuple(behaviors.RULES),
              doc="flee, follow, lure, patrol, guard, shy_gaze, seek_button or sleepy"),
        Field("size", "vec", [0.12, 0.12, 0.07], length=3, unit="m", min=0.04, max=0.5,
              doc="chassis [x, y, z]"),
        Field("mass", "float", 0.3, unit="kg", min=0.02, max=5.0),
        Field("speed", "float", 0.08, unit="m/s", min=0.0, max=1.0, runtime=True,
              doc="top speed; at or above the body's 0.16 m/s `mpg check` warns"),
        Field("accel", "float", 0.5, unit="m/s^2", min=0.05, max=10.0, runtime=True),
        Field("sense", "float", 0.4, unit="m", min=0.05, max=5.0, runtime=True,
              doc="reacts to the body this close (flee, follow, lure, patrol, guard)"),
        Field("leash", "vec", None, length=4, unit="m", min=-50.0, max=50.0,
              doc="[x0, y0, x1, y1] box (world) its centre never leaves; default: none"),
        Field("keep", "float", 0.3, unit="m", min=0.0, max=3.0, runtime=True,
              doc="follow, lure, shy_gaze: stops this far from the body / bait"),
        Field("reach", "float", 0.25, unit="m", min=0.0, max=3.0, runtime=True,
              doc="guard: how far it steps out from its post"),
        Field("path", "any", None,
              doc="patrol, sleepy: [[dx, dy], ...] from `pos`, 2 or more points, a loop"),
        Field("button", "str", None, doc="seek_button: the part it goes onto"),
        Field("bait", "str", None, doc="lure: the object the body must hold"),
        Field("awake", "float", 2.0, unit="s", min=0.1, max=60.0, runtime=True,
              doc="sleepy: seconds moving"),
        Field("asleep", "float", 2.0, unit="s", min=0.1, max=60.0, runtime=True,
              doc="sleepy: seconds still"),
        Field("invert", "bool", False, doc="shy_gaze: move while looked at, stop otherwise"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"][:2]))

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        owner = f"part {name!r}"
        self.behavior = c["behavior"]
        self.half = np.asarray(c["size"], dtype=float) / 2
        self.mass = float(c["mass"])
        self.leash = None if c["leash"] is None else np.asarray(c["leash"], dtype=float)
        if self.leash is not None and not (self.leash[0] < self.leash[2] and self.leash[1] < self.leash[3]):
            raise SchemaError(f"{owner}: leash must be [x0, y0, x1, y1] with x0 < x1 and y0 < y1, "
                              f"got {c['leash']}")
        self.path = None
        if self.behavior in ("patrol", "sleepy"):
            raw = c["path"]
            ok = (isinstance(raw, (list, tuple)) and len(raw) >= 2
                  and all(isinstance(p, (list, tuple)) and len(p) == 2
                          and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in p)
                          for p in raw))
            if not ok:
                raise SchemaError(f"{owner}: a {self.behavior} robot needs `path: [[dx, dy], ...]` "
                                  f"(2 or more points from pos), got {raw!r}")
            self.path = np.asarray(raw, dtype=float)
        for key, beh in (("button", "seek_button"), ("bait", "lure")):
            if self.behavior == beh and not c[key]:
                raise SchemaError(f"{owner}: a {beh} robot needs `{key}: <part name>`")
        if self.behavior == "shy_gaze":
            self.inputs.append(name + SEEN)          # the room adds the gaze sensor (R27)
        self.cmd = np.zeros(2)
        self.wp = 0
        self.timer = 0.0
        self.acting = False
        self.on_id = -1
        self.now = self.base_xy.copy()
        self._buttons: list[Part] = []
        self._target: Part | None = None
        self._bait: Part | None = None
        self._gid = np.zeros(1, dtype=np.int32)

    @classmethod
    def add_room_parts(cls, parts):
        """Check `button` / `bait` names, then the plan hooks (R29: "plan hooks treat
        NPCs") for input parts a robot sets off (trigger counts npc, no `objects` list):
        - a seek_button robot's `button` is on for the planner once the robot's `in`
          are on (no `in`: always), and the body is never sent onto it (role sensor);
        - other such sensors inside a herdable robot's leash count as switches, so
          the solution order can press them with `herd`."""
        npcs = [p for p in parts.values() if isinstance(p, Npc)]
        for n in npcs:
            for key in ("button", "bait"):
                ref = n.cfg[key] if n.behavior in ("seek_button", "lure") else None
                if ref and ref not in parts:
                    raise SchemaError(f"part {n.name!r}: {key} {ref!r} is not a part "
                                      f"(parts: {sorted(parts)})")

        def npc_sets_off(p) -> bool:
            trig = getattr(p, "trigger", None)
            return (p.stage == "input" and trig in TRIGGERS and "npc" in TRIGGERS[trig]
                    and not getattr(p, "names", None))

        for n in npcs:
            target = parts[n.cfg["button"]] if n.behavior == "seek_button" else None
            if target is not None and npc_sets_off(target):
                target.plan_driver = n
                target.role = "sensor"
        herders = [n for n in npcs if behaviors.can_herd(n, parts)]
        for p in parts.values():
            if p.role != "sensor" or p.plan_driver is not None or not npc_sets_off(p):
                continue
            if any(n.leash is None or in_rect(p.base_xy, (n.leash[:2] + n.leash[2:]) / 2,
                                              (n.leash[2:] - n.leash[:2]) / 2) for n in herders):
                p.role = "switch"

    def plan_drives(self, get) -> bool:
        """Planning: whether this seek_button robot heads for its `button` (its `in`
        on, read with `get`; no `in`: always)."""
        return not self.inputs or self.plan_inputs_on(get)

    @classmethod
    def shift_cfg(cls, cfg, dx, dy):
        out = super().shift_cfg(cfg, dx, dy)
        if isinstance(out.get("leash"), (list, tuple)) and len(out["leash"]) == 4:
            x0, y0, x1, y1 = out["leash"]
            out["leash"] = [x0 + dx, y0 + dy, x1 + dx, y1 + dy]
        return out

    # ---- scene ------------------------------------------------------------------
    def build(self, spec):
        hx, hy, hz = self.half
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0])
        lim = mujoco.mjtLimited.mjLIMITED_TRUE if self.leash is not None else mujoco.mjtLimited.mjLIMITED_FALSE
        for i, ax in enumerate(("x", "y")):
            axis = [0.0, 0.0, 0.0]
            axis[i] = 1.0
            rng = ([self.leash[i] - self.base_xy[i], self.leash[i + 2] - self.base_xy[i]]
                   if self.leash is not None else [0.0, 0.0])
            body.add_joint(name=f"{self.body_name}/slide_{ax}", type=mujoco.mjtJoint.mjJNT_SLIDE,
                           axis=axis, range=rng, limited=lim, solref_limit=LEASH_SOLREF)
        body.add_joint(name=self.body_name + "/yaw", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
        zc = CLEAR + hz
        body.add_geom(name=self.body_name + "/chassis", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[hx, hy, hz], pos=[0, 0, zc], mass=self.mass, rgba=BODY_RGBA)
        body.add_geom(name=self.body_name + "/nose", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[0.008, hy * 0.5, hz * 0.3], pos=[hx + 0.008, 0, zc],
                      rgba=(0.95, 0.85, 0.2, 1.0), contype=0, conaffinity=0, mass=0.0)
        r = min(hz, 0.025)
        for sx in (-1, 1):
            for sy in (-1, 1):
                body.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[r, 0.006],
                              pos=[sx * hx * 0.6, sy * (hy + 0.006), r],
                              quat=[math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0],
                              rgba=WHEEL_RGBA, contype=0, conaffinity=0, mass=0.0)
        izz = self.mass * (hx * hx + hy * hy) / 3.0
        for ax, kv, f in (("x", self.mass * KV_RATE, self.mass * FORCE_ACC),
                          ("y", self.mass * KV_RATE, self.mass * FORCE_ACC),
                          ("yaw", izz * KV_RATE, izz * FORCE_ACC * 20)):
            j = f"{self.body_name}/{'slide_' + ax if ax != 'yaw' else 'yaw'}"
            act = spec.add_actuator(name=f"{self.body_name}/motor_{ax}", target=j,
                                    trntype=mujoco.mjtTrn.mjTRN_JOINT)
            act.set_to_velocity(kv=kv)
            act.forcerange = [-f, f]
            act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._j = [model.joint(f"{self.body_name}/{n}").id for n in ("slide_x", "slide_y", "yaw")]
        self._q = [int(model.jnt_qposadr[j]) for j in self._j]
        self._v = [int(model.jnt_dofadr[j]) for j in self._j]
        self._act = [model.actuator(f"{self.body_name}/motor_{a}").id for a in ("x", "y", "yaw")]
        self._colliding = (model.geom_contype != 0) | (model.geom_conaffinity != 0)

    def place(self, model, xy):
        super().place(model, xy)
        if self.leash is not None:
            for i in range(2):
                model.jnt_range[self._j[i]] = [self.leash[i] - self.xy[i], self.leash[i + 2] - self.xy[i]]

    def reset(self, model, data):
        model.body_pos[self._bid, 2] = self.floor_z          # no z joint: stand on raised floor
        for i in range(3):
            data.qpos[self._q[i]] = data.qvel[self._v[i]] = data.ctrl[self._act[i]] = 0.0
        self.cmd = np.zeros(2)
        self.wp, self.timer, self.acting = 0, 0.0, False
        self.now = self.xy.copy()
        self._parts = {p.name: p for p in self.bus.order} if self.bus is not None else {}
        from ..input.button import Button
        self._buttons = [p for p in self._parts.values() if isinstance(p, Button)]
        self._target = self._parts.get(self.cfg["button"]) if self.behavior == "seek_button" else None
        self._bait = self._parts.get(self.cfg["bait"]) if self.behavior == "lure" else None
        self.on_id = self._button_under(self.now)
        self.write(0.0, self.name + ".moving")
        self.write(float(self.on_id), self.name + ".on")

    # ---- what the rules read -------------------------------------------------------
    def pos(self, data) -> np.ndarray:
        """Centre now (world xy)."""
        return data.xpos[self._bid][:2].copy()

    def vel(self, data) -> np.ndarray:
        """Velocity now (world xy) [m/s]."""
        return np.array([data.qvel[self._v[0]], data.qvel[self._v[1]]])

    def moving(self, data) -> bool:
        return float(np.linalg.norm(self.vel(data))) > MOVING

    def yaw(self, data) -> float:
        return float(data.qpos[self._q[2]])

    def path_xy(self) -> np.ndarray | None:
        return None if self.path is None else self.path + self.xy

    def button_xy(self) -> np.ndarray:
        return np.asarray(self._target.xy, dtype=float)

    def bait_pos(self) -> np.ndarray | None:
        """The bait's position while the body holds it (World.held, R18), else None."""
        w, b = self.world, self._bait
        if w is None or b is None or self._data is None:
            return None
        for rid in b.root_body_ids(self._model):
            if int(self._model.body_rootid[rid]) in w.held:
                return self._data.xpos[rid][:2].copy()
        return None

    def looked_at(self) -> bool:
        return self.bus.on(self.name + SEEN)

    def _button_under(self, xy) -> int:
        for b in self._buttons:
            if in_rect(xy, b.xy, b.half):
                return int(b.marker.id) if b.marker is not None else -1
        return -1

    # ---- motion ----------------------------------------------------------------------
    def _ray(self, model, data, origin, d: np.ndarray, length: float) -> float:
        """Free distance along horizontal unit `d` up to `length` (colliding geoms only)."""
        vec = np.array([d[0], d[1], 0.0])
        start, travelled = origin.copy(), 0.0
        for _ in range(6):
            hit = mujoco.mj_ray(model, data, start, vec, None, 1, self._bid, self._gid)
            if hit < 0 or travelled + hit >= length:
                return length
            if self._colliding[int(self._gid[0])]:
                return travelled + hit
            step = hit + 1e-4
            start, travelled = start + vec * step, travelled + step
        return length

    def _in_leash(self, xy) -> bool:
        L = self.leash
        return L is None or bool(L[0] <= xy[0] <= L[2] and L[1] <= xy[1] <= L[3])

    def _steer(self, model, data, me, want: np.ndarray) -> np.ndarray | None:
        """First clear heading near unit `want` (rays, leash), or None."""
        origin = np.array([me[0], me[1], self.floor_z + CLEAR + self.half[2]])
        look = float(np.hypot(*self.half[:2])) + LOOK
        a0 = math.atan2(want[1], want[0])

        rays = []                                  # (heading, free distance), 5 rays
        for deg in (0.0, *(s * a for a in RAY_ANGLES for s in (1, -1))):
            a = a0 + math.radians(deg)
            h = np.array([math.cos(a), math.sin(a)])
            rays.append((h, self._ray(model, data, origin, h, RAY_RANGE)))
        order = [rays[0]]
        for i in (1, 3):                           # each +-pair: the side with more room first
            pair = rays[i:i + 2]
            order += pair if pair[0][1] >= pair[1][1] else pair[::-1]
        for h, free in order:
            if free >= look and self._in_leash(me + h * LEASH_LOOK):
                return h
        return None

    def update(self, model, data, body):
        self._model, self._data = model, data
        me = self.pos(data)
        self.now = me
        dt = self.bus.dt
        target = np.zeros(2)
        self.acting = False
        if self.enabled() and body is not None:
            intent = behaviors.RULES[self.behavior](self, me, np.asarray(body.pos[:2], dtype=float), dt)
            self.acting = intent.acting
            want = intent.away
            if intent.goal is not None:
                d = intent.goal - me
                n = float(np.linalg.norm(d))
                want = d / n if n > 1e-9 else None
            if want is not None and intent.speed > 0:
                h = self._steer(model, data, me, want)
                if h is not None:
                    target = h * min(intent.speed, float(self.live["speed"]))
        step = float(self.live["accel"]) * dt
        dv = target - self.cmd
        n = float(np.linalg.norm(dv))
        self.cmd = target if n <= step else self.cmd + dv / n * step
        data.ctrl[self._act[0]], data.ctrl[self._act[1]] = self.cmd
        turn = 0.0
        if np.linalg.norm(self.cmd) > MOVING:
            err = math.atan2(self.cmd[1], self.cmd[0]) - self.yaw(data)
            err = (err + math.pi) % (2 * math.pi) - math.pi
            turn = float(np.clip(TURN_GAIN * err, -MAX_TURN, MAX_TURN))
        data.ctrl[self._act[2]] = turn
        moving = self.moving(data)
        self.on_id = self._button_under(me)
        self.write(float(moving), self.name + ".moving")
        self.write(float(self.on_id), self.name + ".on")

    _model = None
    _data = None

    # ---- env moves (warp R14, conveyor R12, spring R15) ---------------------------------
    def shift(self, data, vec) -> None:
        for i in range(2):
            data.qpos[self._q[i]] += float(vec[i])

    def kick(self, data, vec) -> None:
        for i in range(2):
            data.qvel[self._v[i]] = float(vec[i])

    def teleport(self, data, pose) -> None:
        for i in range(2):
            data.qpos[self._q[i]] = float(pose[i]) - float(self.xy[i])
            data.qvel[self._v[i]] = 0.0
        if len(pose) > 2:
            data.qpos[self._q[2]] = float(pose[2])
        data.qvel[self._v[2]] = 0.0
        self.cmd = np.zeros(2)

    # ---- observation, planning, checks ----------------------------------------------------
    def state(self):
        return float(self.acting)

    def target_mask(self, grid):
        return grid.disk(self.now, float(np.hypot(*self.half[:2])))

    def check_warnings(self, env) -> list[str]:
        if float(self.cfg["speed"]) >= DUCK_SPEED:
            return [f"part {self.name!r}: speed {self.cfg['speed']:g} m/s is not below the body's "
                    f"walking speed (~{DUCK_SPEED:g} m/s); the body cannot catch it"]
        return []
