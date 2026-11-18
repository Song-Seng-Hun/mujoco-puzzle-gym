"""Built-in reward terms (R7). Each returns an unweighted value; task.yaml gives weights.

Terms read `env.events`, filled once per env step:
- progress: metres of geodesic distance (around walls, through open doors) gained
  toward the current target this step;
- progress_free: the same, toward the nearest target that is not solved yet (R35);
- new_on:   switches that turned on this step;
- success:  a goal was reached;
- fallen:   the body fell;
- respawned: a fall or failure went back to the last checkpoint (R6).
"""

from __future__ import annotations

from .registry import register_reward


@register_reward("progress", replace=True)
def progress(env):
    return env.events["progress"]


@register_reward("progress_free", replace=True)
def progress_free(env):
    """Metres gained this step toward the nearest target that is not solved yet (R35),
    whichever it is: the solution order is not used, so a policy earns nothing for
    walking the order it memorized and has to read the room's cue instead. `progress`
    stays the default; a task that wants this one asks for it by name."""
    return env.events["progress_free"]


@register_reward("stage", replace=True)
def stage(env):
    return float(env.events["new_on"])


@register_reward("success", replace=True)
def success(env):
    return float(env.events["success"])


@register_reward("fall", replace=True)
def fall(env):
    return float(env.events["fallen"])


@register_reward("respawn_penalty", replace=True)
def respawn_penalty(env):
    """1 when the body went back to a checkpoint this step; the default weight is -1."""
    return float(env.events.get("respawned", False))


@register_reward("coin_pick", replace=True)
def coin_pick(env):
    """Coins picked since the last env step (R36); a coin put back by a checkpoint
    restore pays again when picked again."""
    n = 0
    for p in env.parts.values():
        if hasattr(p, "picked_ratio"):
            n += p.picks - p.rewarded
            p.rewarded = p.picks
    return float(n)


@register_reward("ball_progress", replace=True)
def ball_progress(env):
    """Metres per env step the ball is moving toward the current push target
    (a goal net or pin, R38): ball velocity along ball -> target, over hz.
    The ball is the accepted one nearest the target that is not in it yet."""
    import numpy as np

    t = env.current_target()
    if t is None or not hasattr(t, "push_target"):
        return 0.0
    counted = set(getattr(t, "inside", []))
    best, best_d = None, np.inf
    for p in env.parts.values():
        if getattr(p, "object_kind", None) == "ball" and t.push_accepts(p) and p.name not in counted:
            d = float(np.linalg.norm(p.pos(env.data)[:2] - t.xy))
            if d < best_d:
                best, best_d = p, d
    if best is None:
        return 0.0
    b = best.pos(env.data)[:2]
    line = np.asarray(t.push_target(env, b)) - b
    n = float(np.linalg.norm(line))
    return float(best.vel(env.data)[:2] @ line / n) / env.hz if n > 1e-6 else 0.0


@register_reward("grasp_progress", replace=True)
def grasp_progress(env):
    """Metres per env step the grab point closes on the object it still has to pick up
    (R127). The carry task's missing shaping term, in the shape `ball_progress` set.

    Why it is needed: `progress` pays for the body approaching its **current target**, and
    in a carry room that target is the goal - measured on `pick_by_hand`. Nothing in the
    reward pointed at the object, and measured over 4.36 M steps a policy never picked it
    up at all (`stage` 0.0 throughout).

    Two things it has to get right, both of them mistakes in the first version:

      - **"this object is done" comes from the destination, not the object.** An object's
        own `satisfied()` is False for ever (a cube has no condition of its own), so the
        term went on charging for leaving a block that was already in the box: measured, a
        scripted episode that grabs, carries and places summed **-1.736** - the term
        punished the solution. Destinations keep an `inside` list, which is what
        `ball_progress` reads too.
      - **distance is measured from the grab site, not from `BodyState.pos`.** That point
        moves between the wrist and the grasp point as the grasp pose toggles (R122, R125):
        measured jumps of 0.075, 0.046, 0.018 and 0.011 m in an episode's first four steps,
        which a distance reward would read as travel.

    Zero while something is held - carrying it is `progress`'s job from there - and zero
    when every object is delivered.
    """
    import numpy as np

    site = getattr(env.world, "beak_site", None)
    if site is None:
        return 0.0
    here = np.asarray(env.data.site_xpos[site], dtype=float)
    delivered = set()
    for part in env.parts.values():
        delivered.update(getattr(part, "inside", ()) or ())
    held = set(getattr(env.world, "held", ()) or ())
    last = getattr(env, "_grasp_last", None)
    if last is None:
        last = env._grasp_last = {}
    best, best_d, holding = None, np.inf, False
    for part in env.parts.values():
        if not hasattr(part, "handle_points") or part.parked:
            continue
        pts = np.asarray(part.handle_points(env.data), dtype=float)
        d = float(np.min(np.linalg.norm(pts - here, axis=1)))
        if any(b in held for b in part.root_body_ids(env.model)):
            last[part.name] = d                     # keep it current, pay nothing
            holding = True
            continue
        if part.name in delivered:
            last.pop(part.name, None)
            continue
        if d < best_d:
            best, best_d = part, d
    if holding or best is None:
        return 0.0
    was = last.get(best.name)
    last[best.name] = best_d
    return 0.0 if was is None else float(was - best_d)


@register_reward("grasp_hold", replace=True)
def grasp_hold(env):
    """1 the **first** time each object is held in an episode (R127).

    The first version paid every time anything became held, and called that honest because
    a contact hold can be dropped by physics. It is honest and it is farmable: measured, a
    policy learned to open and shut the jaw on the block over and over and collected
    **578** of this term per episode in the last window of a 20-minute run, with a success
    rate of 0.0. Paying once per object per episode leaves the event rewarded and the loop
    worthless.
    """
    held = set(getattr(env.world, "held", ()) or ())
    state = getattr(env, "_grasp_paid", None)
    if state is None or state[0] > env.steps:          # a new episode: `steps` went back
        state = env._grasp_paid = [env.steps, set()]
    state[0] = env.steps
    fresh = held - state[1]
    state[1] |= held
    return float(bool(fresh))


@register_reward("carry_progress", replace=True)
def carry_progress(env):
    """Metres per env step a **held** object closes on the place it belongs (R127).

    The last gap in a carry room's reward. `grasp_progress` gets the gripper to the object
    and `grasp_hold` pays for taking it; from there `progress` is supposed to take over, but
    it pays for the **body** approaching its current target, and in `pick_by_hand` that
    target is the goal rather than the box - so nothing pointed the carry at the
    destination. Measured with the first two terms alone: a `sensors_only` policy learned to
    pick the block up in 0.98 of episodes in 20 minutes, from never, and still delivered
    none of them.

    The destination is the first part that names the held object in its `objects:` and is
    not satisfied yet. Zero when nothing is held.
    """
    import numpy as np

    held = set(getattr(env.world, "held", ()) or ())
    if not held:
        return 0.0
    last = getattr(env, "_carry_last", None)
    if last is None:
        last = env._carry_last = {}
    for part in env.parts.values():
        if not hasattr(part, "handle_points") or part.parked:
            continue
        if not any(b in held for b in part.root_body_ids(env.model)):
            continue
        where = np.asarray(part.pos(env.data)[:2], dtype=float)
        for dest in env.parts.values():
            # Destinations say what they take with `accepts(obj)` - `accept:` in a room -
            # and that is what to ask: the first version looked for an `objects:` list and
            # found nothing, so the term summed 0.000 over a solved episode and failed its
            # own acceptance test.
            takes = getattr(dest, "accepts", None)
            if not callable(takes) or not takes(part):
                continue
            if getattr(dest, "satisfied", None) is not None and dest.satisfied():
                continue
            d = float(np.linalg.norm(where - np.asarray(dest.xy, dtype=float)))
            was = last.get(part.name)
            last[part.name] = d
            return 0.0 if was is None else float(was - d)
        return 0.0
    return 0.0


@register_reward("hold_time", replace=True)
def hold_time(env):
    """1 per env step while an object is held and not yet delivered (R127).

    `grasp_hold` pays once, so a policy has no reason to **keep** holding; drop the object
    and `carry_progress` simply stops paying, with nothing lost. Measured with the other
    three terms and 4.45 M steps, a policy learned to grab in 0.97 of episodes and
    collected almost none of the carry term - it took the one-off and let go.

    Per step, so a task weights it small (0.1 at hz 10 is 1.0 per second of carrying).

    **It is farmable and the measurement says so.** At weight 0.2 on `pick_by_hand` with
    `sensors_only` observation, 4.47 M steps: the policy grabbed in 0.97 of episodes and
    then **held still for 224 of the 400 steps**, collecting 44.87 of this term - more than
    every other term together - while `carry_progress` came to -0.02 and the success rate
    stayed 0.0. Holding is easier than carrying, so an independent per-step bonus for it
    becomes the whole policy. Weight it below the carry, or use potential-based shaping
    instead of a sum of bonuses; the ladder those four runs climbed is in
    `docs/training-log.md`.
    """
    held = set(getattr(env.world, "held", ()) or ())
    return float(bool(held))


def _carry_potential(env) -> float:
    """The task's progress as a single number, for `carry_potential` (R128).

    Three phases, arranged so each real step forward raises it:

        nothing held      -(1 + distance from the grab site to the object)
        holding it        -(distance from the object to where it belongs)
        delivered         +1

    So taking the object is a jump of about +1, carrying it is the distance it closes, and
    dropping it gives the jump straight back. The size of that jump is what makes holding
    worth more than hovering, and the shape is what makes none of it farmable.
    """
    import numpy as np

    site = getattr(env.world, "beak_site", None)
    if site is None:
        return 0.0
    here = np.asarray(env.data.site_xpos[site], dtype=float)
    held = set(getattr(env.world, "held", ()) or ())
    delivered = set()
    for part in env.parts.values():
        delivered.update(getattr(part, "inside", ()) or ())
    best = None
    for part in env.parts.values():
        if not hasattr(part, "handle_points") or part.parked:
            continue
        if part.name in delivered:
            continue
        mine = any(b in held for b in part.root_body_ids(env.model))
        pts = np.asarray(part.handle_points(env.data), dtype=float)
        d = float(np.min(np.linalg.norm(pts - here, axis=1)))
        if mine:
            for dest in env.parts.values():
                takes = getattr(dest, "accepts", None)
                if callable(takes) and takes(part):
                    away = float(np.linalg.norm(np.asarray(part.pos(env.data)[:2], float)
                                                - np.asarray(dest.xy, float)))
                    return -away
            return 0.0
        if best is None or d < best:
            best = d
    if best is None:
        return 1.0                      # everything is where it belongs
    return -(1.0 + best)


@register_reward("carry_potential", replace=True)
def carry_potential(env):
    """Potential-based shaping for a carry room: the **change** in `_carry_potential` (R128).

    One term instead of the four bonuses R127 measured, because each of those became the
    easiest reward in the room as soon as it existed - the ladder is in
    `docs/training-log.md`: a grab bonus taught grabbing and nothing else, a per-step hold
    bonus taught holding still for 224 of 400 steps. A difference of a state function
    cannot be collected twice: standing still pays zero, dropping the object hands back
    exactly what taking it paid, and the only way to keep earning is to get closer to
    done (Ng, Harada and Russell, 1999).
    """
    now = _carry_potential(env)
    was = getattr(env, "_carry_phi", None)
    env._carry_phi = now
    return 0.0 if was is None else float(now - was)
