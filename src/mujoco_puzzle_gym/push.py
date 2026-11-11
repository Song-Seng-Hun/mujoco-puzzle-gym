"""Scripted pushing: the `demo:` sub-goal `push <ball> to <target>` (R20, R38).

The scripted solver runs it for a `push` entry of the room's `demo:`, and for
a grid-plan target with `push_target(env)` (goal nets, pins) when the demo is
over or absent; then it pushes the nearest ball the target `push_accepts`
that is not already counted by it:

1. approach: walk around the ball (planned on the room grid, the ball a round
   obstacle) to the point `BEHIND` metres behind it on the ball -> target line;
2. align: stand and turn in place until facing along that line;
3. push: walk forward aiming a little beyond the ball on the (recomputed)
   ball -> target line, at `PUSH_FAST` until the ball is `NEAR` its push
   target, then at the slower `PUSH_FORWARD`. If the ball gets more than
   `LOST_ANGLE` off the body's way, or too far ahead, it goes back to 1.
"""

from __future__ import annotations

import math

import numpy as np

BEHIND = 0.08          # gap between the body front and the ball at the approach point [m]
BODY_FRONT = 0.11      # trunk centre to the front of the body's collision geoms [m]
AT_POINT = 0.05        # approach point reached [m]
ALIGNED = 0.2          # heading error that ends aligning [rad]
LOST_ANGLE = 0.7       # ball this far off the body -> target-line heading: approach again [rad]
LOST_GAP = 0.25        # ball this far ahead of the body front: approach again [m]
LEAD = 0.1             # push aim point beyond the ball [m]
REPLAN = 0.04          # re-plan the approach path when its goal moved this far [m]
PUSH_FORWARD = -0.5    # action[0] while pushing near the target: slow walking, fewer kicks
PUSH_FAST = 0.0        # action[0] while pushing with the ball farther than NEAR from its target
NEAR = 0.3             # ball -> push target distance below which the push slows down [m]
TURN_GAIN = 2.5
FACING = 0.4


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Pusher:
    def __init__(self, env):
        self.env = env
        self.reset()

    def reset(self) -> None:
        self.phase = "approach"
        self._goal = None
        self._field = None

    def pick_ball(self, target):
        env = self.env
        counted = set(getattr(target, "inside", []))
        here = env.body_state.pos[:2]
        best, best_d = None, math.inf
        for p in env.parts.values():
            if not hasattr(p, "radius") or getattr(p, "object_kind", None) != "ball":
                continue
            if not target.push_accepts(p) or p.name in counted:
                continue
            d = float(np.linalg.norm(p.pos(env.data)[:2] - here))
            if d < best_d:
                best, best_d = p, d
        return best

    def _steer(self, aim, slow_turn: bool) -> np.ndarray:
        s = self.env.body_state
        d = np.asarray(aim) - s.pos[:2]
        err = _wrap(math.atan2(d[1], d[0]) - s.yaw)
        turn = float(np.clip(TURN_GAIN * err, -1.0, 1.0))
        if abs(err) < FACING:
            return np.array([1.0, 0.0, turn], dtype=np.float32)
        return np.array([-1.0 if slow_turn else 0.0, 0.0, turn], dtype=np.float32)

    def _path_aim(self, goal, ball_xy, ball_r) -> np.ndarray:
        env = self.env
        if self._goal is None or np.linalg.norm(goal - self._goal) > REPLAN:
            from .solver import CLEARANCE
            grid = env.grid
            inflate = env.body.footprint_radius + CLEARANCE
            free = env.stage_free(env.stage, inflate=inflate)
            free &= ~grid.disk(ball_xy, ball_r + env.body.footprint_radius)
            self._field = grid.distance(free, grid.disk(goal, grid.res))
            self._goal = goal.copy()
        s = self.env.body_state
        if np.linalg.norm(goal - s.pos[:2]) < 0.25 or not np.isfinite(self._field).any():
            return goal
        return env.grid.descend(self._field, s.pos[:2], 0.25)

    def act(self, target, ball=None) -> np.ndarray:
        """Action pushing `ball` (default: `pick_ball(target)`) toward `target`."""
        env = self.env
        ball = ball if ball is not None else self.pick_ball(target)
        if ball is None:                       # nothing left to push: stand
            return np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        s = env.body_state
        b = ball.pos(env.data)[:2]
        g = np.asarray(target.push_target(env, b), dtype=float)
        line = g - b
        u = line / max(float(np.linalg.norm(line)), 1e-6)
        back = BODY_FRONT + ball.radius + BEHIND
        approach = b - u * back
        to_ball = b - s.pos[:2]
        off = abs(_wrap(math.atan2(to_ball[1], to_ball[0]) - math.atan2(u[1], u[0])))
        if self.phase == "push":
            gap = float(np.linalg.norm(to_ball)) - BODY_FRONT - ball.radius
            if off > LOST_ANGLE or gap > LOST_GAP:
                self.phase = "approach"
        if self.phase == "approach":
            if np.linalg.norm(approach - s.pos[:2]) < AT_POINT:
                self.phase = "align"
            else:
                return self._steer(self._path_aim(approach, b, ball.radius), slow_turn=True)
        if self.phase == "align":
            err = _wrap(math.atan2(u[1], u[0]) - s.yaw)
            if abs(err) > ALIGNED:
                if np.linalg.norm(approach - s.pos[:2]) > 2 * AT_POINT + 0.05:
                    self.phase = "approach"
                return np.array([-1.0, 0.0, float(np.sign(err))], dtype=np.float32)
            self.phase = "push"
        act = self._steer(b + u * LEAD, slow_turn=True)
        if act[0] > 0:
            act[0] = PUSH_FAST if float(np.linalg.norm(line)) > NEAR else PUSH_FORWARD
        return act
