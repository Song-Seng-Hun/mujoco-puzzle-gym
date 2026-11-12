"""NPC behaviour rules (R29): one function per behaviour, run every body step.

Each rule looks at the robot (`npc`, a `rover.Npc`), its position `me` and the
walking body's position `duck` (trunk xy) and returns an `Intent`: the point
it heads for (or a direction), the most speed it wants and whether the rule is
acting now (observation `npcs`). The rover then turns that into a velocity
through its five-ray wall avoidance, its leash and its acceleration limit.
Rules are deterministic: no random numbers, so one seed gives one trajectory.

- flee:        the body within `sense` -> straight away from it.
- follow:      the body within `sense` -> towards it, stopping `keep` short.
- lure:        the body holds the `bait` object (World.held, filled by the R18
               grab) and is within `sense` -> towards the bait, `keep` short.
- patrol:      round `path` (offsets from `pos`, a loop); stops while the body
               is within `sense` and ahead of it (angle < BLOCK_ANGLE).
- guard:       stays on its post (`pos`); the body within `sense + reach` of
               the post -> steps up to `reach` towards it, staying out of its way.
- shy_gaze:    not looked at (`<name>.seen`, R27 gaze sensor) -> towards the
               body, `keep` short; looked at -> stops (`invert: true` swaps).
- seek_button: its inputs on (`in`; no `in`: always) -> onto the centre of
               part `button`; off -> stops.
- sleepy:      `awake` seconds round `path` as patrol does, then `asleep`
               seconds still, again and again.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .rover import Npc

ARRIVE = 0.02          # [m] waypoint / spot reached
BLOCK_ANGLE = 50.0     # [deg] patrol: the body this far off the way ahead blocks it
GAIN = 2.0             # [1/s] speed per metre left near a spot (slows before it)
BLOCK_GAP = 0.2        # [m] guard: centre gap kept to the body (its footprint + the robot)


@dataclass
class Intent:
    goal: np.ndarray | None = None     # point to reach (stops on it)
    away: np.ndarray | None = None     # or a unit direction to run along
    speed: float = 0.0                 # most speed wanted [m/s]
    acting: bool = False


STILL = Intent()


def _toward(me, point, speed: float, stop_short: float = 0.0, acting: bool = True) -> Intent:
    d = point - me
    dist = float(np.linalg.norm(d))
    if dist <= stop_short + ARRIVE:
        return Intent(acting=acting)
    goal = point - d / dist * stop_short if stop_short > 0 else point
    return Intent(goal=goal, speed=min(speed, GAIN * (dist - stop_short)), acting=acting)


def flee(npc: "Npc", me, duck, dt) -> Intent:
    d = me - duck
    dist = float(np.linalg.norm(d))
    if dist >= npc.live["sense"]:
        return STILL
    away = d / dist if dist > 1e-6 else np.array([1.0, 0.0])
    return Intent(away=away, speed=npc.live["speed"], acting=True)


def follow(npc: "Npc", me, duck, dt) -> Intent:
    if np.linalg.norm(duck - me) >= npc.live["sense"]:
        return STILL
    return _toward(me, duck, npc.live["speed"], npc.live["keep"])


def lure(npc: "Npc", me, duck, dt) -> Intent:
    bait = npc.bait_pos()
    if bait is None or np.linalg.norm(duck - me) >= npc.live["sense"]:
        return STILL
    return _toward(me, bait, npc.live["speed"], npc.live["keep"])


def patrol(npc: "Npc", me, duck, dt) -> Intent:
    pts = npc.path_xy()
    if pts is None:
        return STILL
    wp = pts[npc.wp]
    if np.linalg.norm(wp - me) <= ARRIVE:
        npc.wp = (npc.wp + 1) % len(pts)
        wp = pts[npc.wp]
    to_duck = duck - me
    way = wp - me
    nd, nw = float(np.linalg.norm(to_duck)), float(np.linalg.norm(way))
    if nd < npc.live["sense"] and nw > 1e-6:
        cos = float(to_duck @ way) / max(nd * nw, 1e-9)
        if cos > math.cos(math.radians(BLOCK_ANGLE)):
            return Intent(acting=True)                  # waits for the body to move
    return _toward(me, wp, npc.live["speed"], acting=False)


def guard(npc: "Npc", me, duck, dt) -> Intent:
    post = npc.xy
    d = duck - post
    dist = float(np.linalg.norm(d))
    if dist >= npc.live["sense"] + npc.live["reach"] or dist < 1e-6:
        return _toward(me, post, npc.live["speed"], acting=False)
    out = min(npc.live["reach"], max(0.0, dist - BLOCK_GAP))
    return _toward(me, post + d / dist * out, npc.live["speed"])


def shy_gaze(npc: "Npc", me, duck, dt) -> Intent:
    seen = npc.looked_at()
    if seen != bool(npc.cfg["invert"]):
        return Intent(acting=True)                      # frozen while looked at
    return _toward(me, duck, npc.live["speed"], npc.live["keep"])


def seek_button(npc: "Npc", me, duck, dt) -> Intent:
    if npc.inputs and not npc.inputs_on():
        return STILL
    return _toward(me, npc.button_xy(), npc.live["speed"])


def sleepy(npc: "Npc", me, duck, dt) -> Intent:
    awake, asleep = npc.live["awake"], npc.live["asleep"]
    t = npc.timer % (awake + asleep)
    npc.timer += dt
    if t >= awake:
        return Intent(acting=True)                      # asleep
    return patrol(npc, me, duck, dt)


RULES = {"flee": flee, "follow": follow, "lure": lure, "patrol": patrol, "guard": guard,
         "shy_gaze": shy_gaze, "seek_button": seek_button, "sleepy": sleepy}
#: Behaviours the scripted solver can move with `herd` (R29): it drives a flee
#: robot from behind and leads a follow robot, and a lure robot while the body
#: holds its bait (R18 grab; `demo:` `grab <bait>` first).
HERDABLE = ("flee", "follow", "lure")


def can_herd(npc, parts) -> bool:
    """`npc` has a HERDABLE behaviour (lure: its bait can be grabbed)."""
    if getattr(npc, "behavior", None) not in HERDABLE:
        return False
    return npc.behavior != "lure" or hasattr(parts.get(npc.cfg["bait"]), "handle_points")
