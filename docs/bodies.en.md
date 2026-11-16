# Bodies and parts (R46)

This document records **what each part declares it needs from the body** and **whether that
declaration is actually true**. The `body` stage of `mpg check` reads exactly this declaration
(`requires_body`), and if the second column of the table below disagrees with the code,
`tests/test_capability_gate.py` fails.

Why a separate document is needed: up to M2 this repository had only one body. So
`Capabilities.missing()` **always returned an empty list**, and every part's `requires_body` was a
claim that had never once been disproved. An abstraction with a single adapter is not an
abstraction, it is a guess. Attaching a second body turned up four places where the declaration was
a lie (§4).

## 1. What a body declares

| | `microduck` | `go2` |
|---|---|---|
| Capability names | `walk`, `grab`, `roller`, `tow`, `chest_push` | `walk` |
| `max_lift_mass` [kg] | 0.05 | 0.0 (nothing to lift with) |
| `max_step` [m] | 0.01 | 0.03 |
| `max_slope` [deg] | 5.0 | 6.0 |
| `reach_height` [m] | 0.21 | not declared |
| `walk_speed` [m/s] | 0.16 | 1.0 |
| Total mass [kg] | 0.737 | 15.206 |
| `footprint_radius` [m] | 0.11 | 0.41 |
| `fall_height` [m] | 0.07 | 0.15 |

Where the numbers come from: for `max_step`, `max_slope` and `walk_speed`, the measurement
conditions (number of seeds, command, pass criterion) are written in the comments of each body
adapter. The mass is read from the model (`BodyAdapter.total_mass`), and `footprint_radius` is the
radius of a circle wrapped around the body — it has to cover the long side, so it is conservative
about the width needed to pass sideways (§6).

**A numeric limit that is not declared reads as 0** (`bodies.declared_limit`, R45). Reading an
unknown limit as infinite is the dangerous direction: then `mpg check` says a body can solve a room
it cannot get through.

### A name does not carry anatomy (R67, 2026-11-17)

`beak_grab` and `beak_height` are now **`grab` and `reach_height`**. Because the first body
had a beak, a jaw, a hand and a 29-joint humanoid all had to declare "picks things up with
its beak" - the first body's anatomy leaking into shared vocabulary, which is the leak R2
exists to stop. **The old names keep working** (`ALIASES` in `bodies/base.py`): a required
name and a declared limit key are both normalised before anything is compared, so a room,
preset or plugin part written against the old vocabulary still runs. A refusal reports the
name the caller used, since that is the string they can find in their own file.

And **a capability name no part requires now fails a test**
(`tests/test_capability_names.py`). The gate compares names, so a name nothing asks for can
neither refuse nor admit a room; it reads as a promise the environment does not keep. The
arm declared `reach` and `gripper` with no part requiring either, and they are gone. The
dimensions they stood for are already compared as numbers: height against `reach_height`,
mass against `max_lift_mass`. M4 left `two_hands` unbuilt on exactly this reasoning and did
not apply it to the arm in the same PR, which is why this is a test and not a rule.

## 2. Parts × bodies

`requires_body` is what a part built with default settings declares. Parts whose declaration depends
on their settings are listed separately in §3. `o` means that body has every declared capability,
and `x` means the `body` stage of `mpg check` refuses, naming the part and the missing capability.

| Part | `requires_body` | microduck | go2 | Reason |
|---|---|---|---|---|
| `addon_station` | `walk`, `roller` | o | x | Device that attaches an add-on to the body |
| `anchor` | - | o | o | A post standing on the floor, nothing to do with the body |
| `ball` | `walk` | o | o | The body walks up and pushes it to roll |
| `brake_strip` | `walk`, `roller` | o | x | A strip that stops the wheels |
| `button` | `walk` | o | o | Stepping on it. Judged by the body's centre coordinates |
| `cannon` | - | o | o | Fires on a signal, nothing to do with the body |
| `checkpoint` | `walk` | o | o | Stepping on it |
| `coin` | `walk` | o | o | Collected by passing through |
| `coin_group` | - | o | o | Signals only |
| `coin_lock` | - | o | o | Signals only |
| `conveyor` | - | o | o | Moves the root position directly (mass-independent, §5) |
| `counter` | - | o | o | Signals only |
| `cube` | `grab` | o | x | An object picked up with the beak |
| `destroyer` | - | o | o | Strikes down on a signal |
| `dial` | `walk`, `chest_push` | o | x | Push a spoke with the chest for one notch (§4) |
| `dispenser` | `walk` | o | o | The body walks up and receives |
| `door` | - | o | o | A door that opens on a signal; the room sets the width (§6) |
| `drawer` | `grab` | o | x | Bite the handle and pull |
| `egg` | `grab` | o | x | An object picked up with the beak |
| `floor_tile` | - | o | o | A floor tile; the room sets the height |
| `gate` | - | o | o | Signals only |
| `goal` | - | o | o | A position test, with no mechanism to operate |
| `goal_net` | `walk` | o | o | A goal the body enters; the room sets the dimensions (§6) |
| `handle_ball` | `grab` | o | x | An object picked up with the beak |
| `hitch_station` | `walk`, `tow` | o | x | A pad that ties off the body's rope end (§4) |
| `key` | `grab` | o | x | An object picked up with the beak |
| `laser` | `walk` | o | o | The body breaks the beam |
| `laundry` | `grab` | o | x | An object picked up with the beak |
| `lever` | `grab` | o | x | Default `mode: grab`. For `push` see §3 |
| `low_bar` | `roller` | o | x | A low bar you crouch under (§4) |
| `npc` | - | o | o | Moves on its own. The speed warning is relative to the body (§5) |
| `pendulum` | - | o | o | An obstacle swung by a position servo |
| `pillar` | - | o | o | A static obstacle |
| `pin` | `walk` | o | o | A bowling pin, knocked over by the body or a ball |
| `pit` | - | o | o | A pit. The room sets the size (§6) |
| `platform` | - | o | o | A moving floor; the room sets the height |
| `pressure` | `grab` | o | x | Default `trigger: object`. For `any` see §3 |
| `pull_cord` | `grab` | o | x | Bite the handle and pull |
| `pusher` | - | o | o | An obstacle pushed out by a position servo |
| `receiver` | - | o | o | Signals only |
| `receptacle` | `grab` | o | x | A bin you put objects into |
| `stack` | - | o | o | Reads a tower (it only looks, so it needs nothing) |
| `relay` | - | o | o | Signals only (for `movable` see §3) |
| `rope` | - | o | o | `a: body` is refused at load time (§4) |
| `rope_door` | - | o | o | Opens on a signal |
| `setter` | - | o | o | Signals only |
| `source` | - | o | o | Signals only |
| `spinner` | - | o | o | An obstacle turned by a velocity servo |
| `spring` | - | o | o | Writes velocity directly (mass-independent, §5) |
| `stick` | `grab` | o | x | A tool picked up with the beak |
| `swing_door` | - | o | o | Opens on a signal |
| `switch` | `walk` | o | o | Stepping on it |
| `target` | - | o | o | Only cannonballs hit it |
| `transmitter` | - | o | o | Signals only |
| `trapdoor` | - | o | o | A floor that opens on a signal |
| `turntable` | - | o | o | A turning floor |
| `warp` | `walk` | o | o | A pad that moves the body |
| `wind` | - | o | o | Applies a force. The strength warning is relative to the body (§5) |
| `wire` | - | o | o | Broken by a cannonball by default. For `by: [peck]` see §3 |
| `zone` | `walk` | o | o | Default `trigger: body`. For `object` see §3 |

## 3. Declarations that depend on the settings

| Part setting | `requires_body` | go2 | Why |
|---|---|---|---|
| `button` `mount: wall` | `walk`, `grab` | x | A wall button is pressed with the beak tip |
| `button` `trigger: object` | `grab` | x | An object has to be placed on it |
| `lever` `mode: push` | `walk`, `chest_push` | x | Push the paddle up with the chest (§4) |
| `pressure` `trigger: any`/`body` | `walk` | o | It also turns on when the body stands on it — the meaning changes (§5) |
| `zone` `trigger: object` | `grab` | x | An object has to be carried in |
| `wire` (or another `breakable`) `by: [peck]` | `grab` | x | Pecking is beak tip + grab action (§4) |
| `relay` `movable: true` | - | o | A box pushed around. At 0.5 kg a heavy body kicks it far away |

## 4. The four places where the declaration was a lie (audit of 2026-11-16)

In all four, before the fix, `check_body` and `check_parts` were **all `ok=True`** on Go2 while the
room was unsolvable.

| Part | Before | After | Evidence |
|---|---|---|---|
| `lever` `mode: push` | `[]` | `walk`, `chest_push` | On Go2, all 5 seeds left `.angle` at its initial value of -1.000, with 0 paddle/rod contacts. The underside of the torso, at 0.237–0.243 m, passes above the paddle's top face at 0.120 m, and the feet pass outside the paddle's half-width of 0.060 m (\|y\| 0.07–0.14 m). **It is not hard, it does not touch.** The Microduck pushed the same room to 1.000 in 5/5 |
| `dial` | `[]` | `walk`, `chest_push` | On the pass line written in the docs (0.10 m from the axis), Go2 over 5 seeds: +1.16, −0.32, 0.00, −0.04, 0.00 notches — 1/5 got past one notch and **2/5 tripped on a spoke and fell** (torso z 0.057). At 0.12 m: 0, +2.07, +2.00, +1.00, 0 notches. A pass in the opposite direction, which should do nothing, turned it +2.0–+5.1 notches. Shin, thigh, hip and torso all hit the spokes at once. The Microduck gives exactly +1.000 (90.0°) in 5/5 |
| `hitch_station` | `[]` | `walk`, `tow` | A room with a rope and an attach station passes every stage on Go2, yet the rope's hitch list is `['sled','post']` — **`body` is not in it**. Step on the pad and nothing is ever tied. The station exists precisely for the body's rope end, and it did not declare that |
| `breakable` `by: [peck]` | `[]` | `grab` | A room with a wall wire set to `by: [peck]` passes on Go2 while `World.beak_site` is −1. It can never be broken |

One more: `low_bar` went from `[]` to `roller`. It is a different kind of problem from a lie:
`bottom` is pinned to 0.21–0.246 m in the schema, so **no room setting whatsoever** lets a larger
body through (the top of the Go2 torso is at 0.41 m). It was right to be blocked, but it was blocked
without saying why. Now `mpg check` refuses and names the part.

Conversely, a place that was **already honest**: using `rope` with `a: body` blows up loudly at load
time with `a: 'body': this body has no tow hitch`. It does not pass quietly.

## 5. Where the numbers do not fit the body

A part's force, mass and speed numbers were measured against one body. On a body 20 times heavier
they mean something else. **The existing numbers were not changed** — changing them would shake
every existing room and measurement record. Instead, when a number is meaningless on that body, the
`parts` stage of `mpg check` now **says so** (a warning, not a failure).

| Part | What is off | The one action taken |
|---|---|---|
| `wind` | Applies a force [N]. The preset 1.0 N is 14 % of the light body's weight and 0.7 % of the body 20 times heavier. Even the schema's upper limit of 20 N is 13 % of that body's weight, so the range that was measured to knock the body over (28 %) is **legally unreachable** | Warn below 5 % of the body's weight. The boundary comes from the measurement table: 0.5 N (7 % of weight) passed 20/20, 1.0 N (14 %) fell in 8/20, 2 N (28 %) in 10/10 |
| `pressure` with a `trigger` that counts the body | Two things are wrong at once. (1) If the plate is narrower than the body's footprint, only one or two feet land on it and `.mass` swings with every step — on the default 0.25 m plate, Go2 read a mean of 2.2–5.2 kg, a maximum of 9.2 kg, and **0 kg** on 1–5 % of the steps, and `min_mass` of 0.1, 1.0 and 5.0 all turned on at some step while only 2–3 of 5 seeds were still on at the end of 3 s. Widening it to 0.9 m puts all four feet on and reads the true value, 13.7 kg. The Microduck (footprint 0.11 m) read 0.730–0.751 kg on the default plate and never read 0. (2) If the threshold is below the body's own mass, merely standing on it turns it on | Warn on both: when the plate is narrower than the footprint diameter, and when `min_mass` is at or below the body's mass |
| `npc` speed | The warning hardcoded `DUCK_SPEED = 0.16` for the comparison. Measured: at maximum command, the Microduck does 0.220 m/s and Go2 1.019 m/s (4.6×). It warned that a 0.5 m/s robot, which Go2 catches easily, "cannot be caught" | Compare against the body's `walk_speed` capability value. The Microduck declares 0.16 as before, so its behaviour is unchanged |

**What differed from the plan.** The spring and the conveyor have nothing to do with weight. `kick`
writes `qvel` and `shift` writes `qpos` directly (`bodies/base.py`). Mass does not enter the
formula. The measurements confirm it — a 1.5 m/s kick lifted Go2 from 0.358 to 0.470 m without
falling in 5/5, and it was still walking 3 s later (2.25 m); the conveyor carried it a full
0.85–0.90 m in 3 s (after subtracting the 0.26–0.35 m of forward drift in place with the command at 0).
The rope knot's `max_hold_force` of 3 N is a value tied to the beak, so on a body without
`grab` it is refused in the first place.

## 6. To be solved with dimensions (handed to R47)

These are not declaration problems: the **room numbers** are sized for one body. The schema ranges
are generous, so raising them in a Go2 room preset is enough.

| Part | Default | Go2 measurement |
|---|---|---|
| `door`/`swing_door`/`rope_door` `width` | 0.5 m | Passes an open 0.5 m gap 5/5; 0.7 and 0.9 the same. Inside the doorway the body's maximum y width is 0.273–0.288 m (about 0.10 m of clearance on each side). The closed control group passes 0/5 |
| `pit` `size` | 0.3 m | At 0.3 m it **does not fall in** — only 1 of 5 seeds crossed, and 4 tripped on the rim and ended in an ordinary fall rather than a "pit failure". At 0.6 and 0.9 m it falls in 5/5. The boundary is known only to be between 0.3 and 0.6 m (nothing in between was measured) |
| `goal_net` `height`/`width` | 0.12 / 0.36 m | The top of the torso is at 0.41 m, so the defaults cannot be entered (schema upper limits 0.5 / 2.0 m) |
| Example room size | 2.4 × 1.8 m | Tight for a body 0.70 m long. The audit used 5.0 × 4.0 m |
| `footprint_radius` | 0.41 m | Wrapping a circle around the body covers the long side (0.70 m). Path planning then demands a gap of about 0.82 m, while in practice it gets through 0.5 m. **It is left conservative on purpose**: set it small and the planner routes the body through gaps it does not fit |

## 7. Still open

- **The environment does not check capabilities.** A room with a `mode: grab` lever is still built
  and reset with `body: go2`. The only thing that catches it is `mpg check`. A user who skips the
  check gets a room they quietly cannot solve.
- **Bodies without eyes.** The `gaze` and `camera` observation terms use
  `view_frame`/`view_camera`, and on a body that has neither they blow up with
  `NotImplementedError` and `ObsParamError` respectively. It is a loud failure, but unlike a part
  declaration, `mpg check` does not say so in advance.
- **Comments that say "duck".** There is still a lot of prose in `parts/` that assumes one body
  (`breakable.py`, `wire.py`, `relay.py`, `brake.py` and others). The forbidden list in
  `tests/test_boundary.py` only looks for specific names (`microduck`, `go2`, …), so this prose
  passes. It is not wrong, but it reads as written for one body.
- **The `pit` boundary** is known only to be between 0.3 and 0.6 m. It will need narrowing if it
  ever matters.
- **Ball-pushing steering has not been measured on a large body.** The walking steering turns on the
  spot when the body declares `stand_to_turn_above` (R47), but the ball-pushing side (`push`) does
  not read that declaration and still turns through the middle of the command range in some cases. On a body with a large turning
  radius that becomes a movement that swings wide around the ball. `ball` requires only walking, so
  a large body gets that room too, but a ball-pushing room has never been measured with a large
  body.

## 8. Asset provenance and licences

Each body pins its model and policy to a fixed revision (`mpg fetch-assets --body <name>` verifies
the SHA256 of each file). Each body has its own lock file.

The licences below are copied straight from the `license` entries of the lock files. They differ by
kind of file even within one repository, so they are not lumped together.

| | Microduck | Go2 |
|---|---|---|
| Model | [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) `0daab16` | `unitree_go2/` of [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) `370c439` |
| Meshes | **CC BY-NC-SA 4.0** (non-commercial) — `*.stl` | BSD-3-Clause |
| MJCF | Apache-2.0 | BSD-3-Clause |
| Walking policy | `velstand.onnx` of Hugging Face `pollen-robotics/microduck-policies` `283771b` | `policy.onnx` + `policy.onnx.data` of Hugging Face `diasAiMaster/unitree-go2-velocity-flat` `9a723b7` |
| Policy licence | Apache-2.0 | BSD-3-Clause |
| Lock file | `upstream.lock.json` | `upstream.go2.lock.json` |

**The Go2 walking policy is not an official Unitree release.** It is a checkpoint uploaded by the
community. What Unitree published is training code (`unitree_rl_gym`, `unitree_rl_mjlab`), and a
survey on 2026-11-15 found no frozen locomotion ONNX. So this checkpoint is what we use (user
decision, 2026-11-15).

What has been confirmed about that policy. All of it was read from the model card and the pinned
files.

- It was **trained on flat ground** (task name `Mjlab-Velocity-Flat-Unitree-Go2`). Stairs and slopes are outside the training distribution.
- It was trained **without** the `gait_phase` observation, and the action scale is **0.5**.
- The training code did **not modify** upstream `unitree_rl_mjlab` (the diff uploaded with the repository is an empty diff).
- The model card says to use that repository's `params/deploy.yaml` because "it may differ from the
  upstream default `deploy.yaml`", so that file is pinned as well and the adapter reads its numbers
  from there. The same numbers are not written twice in the code.

On the Microduck side, what is **non-commercial** is the meshes (`*.stl`). The MJCF and the policy
are Apache-2.0. That body cannot be simulated without the meshes, so for commercial use it has to be
taken that the body itself cannot be used. On the Go2 side the model and the policy are both
BSD-3-Clause.

## 9. A body that does not walk (fixed-base arm)

The third body does not walk. It is an arm bolted to a tabletop, and **the gripper is the body**.
The puzzle code asks the body only four things — position on a plane, heading, velocity, and whether
it has fallen — and planning happens on a 2D grid. Tabletop manipulation is a planar problem too, so
answering those four **for the gripper rather than for a torso** keeps the same boundary in place.
Not one line of `BodyAdapter` changed.

| | Value | How it was found |
|---|---|---|
| Reach radius | 0.001–0.480 m | Swept a 9⁴ joint grid of 6,561 points with `mj_kinematics`. There is no published official figure |
| Reach height (kinematic) | z −0.223 … 0.520 m | The same sweep |
| Reach height (**controlled**) | z 0.111–0.122 m | Measured by driving each command limit for 4 s. The shared command `(vx, vy, wz)` has no vertical axis and `step` holds the gripper at `WORK_Z` 0.12 m, so the capability value `reach_height` is **0.12** - the 0.45 first declared was the kinematic figure and would have admitted a wall button at 0.40 m |
| Working band | out to a radius of 0.480 m at z 0.02–0.15 m | The same sweep |
| Envelope shape | a 220° **sector** (shoulder rotation ±1.920 rad) | A sector, not a ring. The room has to fit inside it |
| Degrees of freedom | 5 arm axes + 1 jaw axis, no free joint | `nq = nv = 6` |
| Mass | 0.645 kg | The model's `body_subtreemass` |
| Gripper tracking | about 1 cm of error over a 10 cm move at mid radius | Measured by giving the command 2 seconds. Folded back toward the base it grows to 5 cm |

**This body's rooms are not training tasks yet** (measured 2026-11-17, R78). Success here
is "a part with `role: goal` is satisfied", and every shipped room makes its puzzle by
putting a door in the way. That does not transfer to this body: everything behind a door is
outside its 0.48 m ring, so the door is scenery and the goal in front of it is reachable
from the start. `tabletop/pick_and_place` still succeeds **5/5** with its demo cut to a
single `goto goal` step. The grab and the carry were measured - the scripted solver cannot
skip a demo step, and before the holonomic fix it scored 0/20 - but the win condition was
not. Gating the goal with `enable: box.ok` is not the fix (the solver then fails at the
grab step), and no shipped room gates a goal at all, so that path is untested. What is
needed is a goal part whose satisfaction *is* a channel.

**The reach it declares is now used by `mpg check`** (R77). For a body whose base does not
move - one that declares `reach_max` and no `walk` - every part the room's `demo:` says it
must touch by hand has to lie inside the ring around its base. Outside it, the `body` stage
refuses with the distance and the range: `the solution touches 'goal' at 0.6 m from the
body's base, outside its reach [0.0, 0.48] m`. Before this, nothing in the package read
`reach_max` or `reach_min`, so a room with a target at 0.6 m passed every static stage and
failed only the scripted solver - and "the solver did not reach the goal" does not say the
body never could.

A part the `demo:` does not name is not checked. The door in `tabletop/pick_and_place` sits
0.72 m from the base, outside the envelope, and that room solves 20/20, because a door is
opened by a signal rather than by hand: a stage that checked every part would refuse a room
that demonstrably works. The envelope's **sector is not checked yet**: a part inside the
ring but behind the base (the dead wedge) still passes, because no body declares a sector.

The four ways this body behaves differently are exposed, not hidden.

- **The base is fixed.** With no free joint, `reset` cannot move the body to the room's start
  position — it only returns the home joint angles, and the base stays where `spawn` put it. So a
  room that draws a random start position randomizes nothing for this body, and the adapter's
  `check_warnings` says so.
- **It does not fall.** `fallen` is always false and the tilt is always vertical, so the `fall`
  reward and termination never fire on this body (turning them off in the task is the honest thing
  to do).
- **It does not walk.** It does not declare `walk`, so the 15 parts that require walking refuse
  themselves **by name**. Checking the example room with this body gives `part 'switch_a' needs body
  capabilities ['walk']` — it does not become a room you quietly cannot solve.
- **Heading cannot be commanded.** Measurement shows that the only joint that changes the gripper's
  azimuth is the shoulder rotation (+0.2 rad gives an azimuth of −0.200 rad, all other joints
  0.000), and that joint also sets the gripper's **position**. That is, the gripper always faces
  outward from the base. Steering that "turns and then goes forward" can only move radially, so this
  body declares `holonomic` and the shared steering drives **by heading**. A lateral command draws a
  tangential arc around the base rather than a straight line (over 2 s at 0.1 m/s the radius holds
  and the arc is 0.2 m).

**It declares `grab`, and the name is wrong.** What the gate asks is "can you pick something up
with your own end effector", and this arm can. The name merely came from the first body's anatomy,
and that is the same kind of leak R2 exists to stop. Renaming it to `grab` and leaving `grab`
as an alias is M4 R60 work.

Assets: Menagerie `robotstudio_so101/` (Apache-2.0), inside the commit `370c4396` this repository
**already pins**. There is no pre-trained policy under a usable licence, so control is inverse
kinematics (damped least squares). The cache paths are under `arm/` — sharing `robot/` with the
quadruped body that uses the same commit overwrites each other's files (it actually happened once,
and `tests/test_assets.py` now refuses that collision).

## 10. Humanoid (Unitree G1)

The fourth body is a 29-joint humanoid that walks on two legs. The structure is the same as the
quadruped body's — a pinned MJCF, a **frozen policy** that takes a velocity command `(vx, vy, wz)`,
planar commands in and `BodyState` out. The three differences are written into the code rather than
hidden.

| | Value | How it was found |
|---|---|---|
| Mass | 33.34 kg | The model's `body_subtreemass` (agrees with the manufacturer's published figure of about 35 kg) |
| Standing height | pelvis 0.79 m | The model's `stand` keyframe |
| Footprint radius | 0.316 m → **declared 0.32** | The farthest distance in the standing pose, geom sizes included |
| Walking speed | **0.455 m/s** at the command ceiling of 0.6 | A 6 s run. It follows about 3/4 of the command |
| Lateral speed | 0.211 m/s at command 0.3 | The same method. It does not fall |
| Turn on the spot | **0.44 rad/s** at command 0.5 (0.096 m of travel) | The same method |
| Turning arc | 0.34 m at mid command, 1.03 m at the ceiling | Speed ÷ turn rate. The middle is small, so `stand_to_turn_above` is left at infinity (the default) |
| `fall_height` | **0.45 m** | A lateral kick while walking: 1.0 m/s holds (low of 0.722 → back up to 0.766), 2.0 m/s falls (0.058–0.118, body-frame gravity +0.18) |
| A 20 s run | 6.21 m, no falls | Command 0.4 |

**The policy is for 12 leg joints and the model has 29.** The upstream observation puts 12 values
into `obs[9:21] = d.qpos[7:]`, so that robot has exactly 12 joints — a legs-only model. This body
feeds the policy the **12 leg joints** of the 29-joint model and holds the waist and both arms at
the model's own standing pose. The policy has never seen arms, so whether it still walks is **a
measurement, not an assumption**, and the table above is that measurement.

**There is a gait clock in the observation.** The last two of the 47 values are the sin and cos of a
phase with a 0.8 s period. The quadruped body's policy was trained without a phase, so this adapter
runs the clock off simulation time, the way upstream does.

**The checkpoint is TorchScript, not ONNX.** It is read with `torch.jit.load`.

**It solves the existing large-body rooms unchanged.** `puzzles/wide/switch_door` 20/20 (18.29 s
mean), `puzzles/wide/pit_detour` 20/20 (19.62 s mean), 0 falls in either, and **the episode length
stays at the default 40 s**. The rooms were sized for a body with a footprint radius of 0.41 m and
this body is 0.32 m, so every gap already fits. The mean time is shorter even though the speed is
45 % of the quadruped's, because this body does not swing wide — it **walks sideways**.

Two bugs of mine came before that conclusion. Neither was "code that assumes one body": both were
**newly written by me**. ① Leaving `action[0] = -1` on the default linear mapping made it full-speed
**reverse**, and the solution's "turn on the spot when a wall is ahead" value walked the body
backwards for 13 s, out of the room and over (5/5). Following the repository convention,
`command_from_action` is now overridden (the same thing the quadruped body did for the same reason
in R44). ② The upper body was computed as "every actuator the policy does not use", and that is
**the whole scene**, so this body overwrote the room's door actuator with its own pose every step. A
door linked to a pressed button stayed closed to the end. Now it touches only actuators that carry
its own prefix, and a test keeps it that way.

Assets: the model is Menagerie `unitree_g1/` (BSD-3-Clause), inside the commit `370c4396` this
repository **already pins**. The policy is `deploy/pre_train/g1/motion.pt` from
`unitreerobotics/unitree_rl_gym` (BSD-3-Clause, 145,745 B). **This policy, too, is not an official
Unitree release of a frozen policy** — it is the checkpoint that repository uploaded along with its
deployment example. The cache path is `humanoid/`, and the observation order was copied from
`deploy/deploy_mujoco/deploy_mujoco.py` at the pinned commit. The numbers (gains, default pose,
scales) are read from `policies/g1.yaml` and not written twice in the code.

### The hands were measured, and not shipped (2026-11-17)

This body's hands had been "the next thing" since M4. The assets are real and the policy
tolerates the extra joints; what it does not tolerate is being asked to *use* them.

| what | measured |
|---|---|
| hands model | `unitree_g1/g1_with_hands.xml` at the pinned commit 370c4396, 34,372 B, sha256 `a6d88c9efde50e9715c71e590ce1a8d3515a0099df7f36e62add8204dae758ad`. All 49 mesh references were **already in the lock** |
| size | 43 actuators (29 + 7 a hand), nq 50, mass 34.39 kg against 33.34 |
| walking, arms at the model's own pose | 0.218 m/s at cmd 0.3 (0.227 without hands), 0.445 at the ceiling (0.454), no falls, same pelvis height |
| grab site, arms down | 0.238 m **to the right** of the pelvis, +0.005 m forward, 0.561 m high; steady while walking (0.535-0.576) |
| one arm raised, walking | 0.338 m/s at cmd 0.3 with +16.1 deg of yaw drift over 10 s (bare body: 0.227 and -12.9) |
| both arms raised, walking | **0.832 m/s** at cmd 0.3 - the command no longer sets the speed |
| both arms raised, standing | walks away **3.655 m in 6 s** at a zero command |
| one arm raised, standing | drifts 0.378 m in 6 s (arms down: 0.187), no fall; site 0.429 m ahead, 0.750 m high |
| twelve carry poses swept (shoulder pitch x elbow) | **every one** broke speed tracking: 0.36-0.83 m/s at a 0.3 command, +10 to +52 deg of drift, one fell |
| hip-height site passing a fixed point, 2 m approach | closest **0.448 m**, eleven times the 0.04 m grab radius |

**The reason is in the policy's observation.** The pinned checkpoint is legs-only: its 47
observations hold twelve leg joint angles, velocities and previous actions, the base's
angular velocity, gravity, the command and a gait phase - and **no arms**. So when an arm
pose moves the centre of mass, the policy cannot see the cause and does not compensate: it
leans, and the gait leaves the command behind. The only arm pose that leaves the measured
gait intact is the one it was trained with, and that pose hangs the hand at the hip.

**Which is why it cannot ship as a grabbing body.** The shared carrier aims the body's
*centre* at an object, because every grabbing body so far has its grab point ahead on its
centreline (a beak; a jaw that is the whole body). A grab point 0.24 m off to one side is
never brought within 0.04 m by aiming the centre, and the measurement says so: 0.448 m at
closest. Declaring a capability the body cannot exercise is the decoration R67 forbids.

**What it would take, in order of honesty.** (1) A policy trained with the arms in its
observation - none is published for this model. (2) Arm IK plus a whole-body balance
controller, which is a project rather than a milestone. (3) Teaching the carrier to aim a
body's grab point instead of its centre - the smallest change, and one that would make the
first body's approach more accurate too, but on its own it only buys a hip-height grab that
still has to beat 0.19 m of standing sway with a 0.04 m radius.

Nothing of the attempt ships: no adapter, no body registration, and the hands model is
unpinned again, so the lock holds no file the package reads. The numbers above are the
record, so the next attempt starts from this table rather than from a download.

### Which body a room belongs to is now a table (R76, 2026-11-17)

`mpg check <room> --all-bodies` runs every registered body against one room and says which
stage decided each cell: `capability` (the gate refused by name), `unreachable` (outside a
fixed reach ring, R77), `static` (a part's own refusal), `unplannable` (no path on the
grid), `actions` (the task's `actions:` lack what the plan needs), `solver` (planned, not
solved), `pass`.

Before this, every nameless exclusion arrived as "the solver did not reach the goal", so a
hard room and an impossible one looked alike - which is why "only this body solves it" in a
preset comment was my word for it. Run, it turned out some of those words were wrong (3
seeds):

| room | microduck | Go2 | arm | G1 |
|---|---|---|---|---|
| `wide/switch_door` | pass 22.9 s | pass 25.4 s | `capability` ['walk'] | pass 20.2 s |
| `wide/pit_detour` | pass | pass | `capability` ['walk'] | pass |
| `tabletop/reach_target` | **pass 1.5 s** | solver | pass 0.6 s | **pass 11.8 s** |
| `tabletop/pick_and_place` (with its task) | **pass** | `capability` ['grab'] | pass | `capability` ['grab'] |

**Neither of the arm's rooms is exclusive to it.** `reach_target` holds nothing that needs a
capability, so three bodies solve it, and `pick_and_place` is solved by the first body too,
whose beak is a `grab`. The large-body rooms are solved by the small body as well, in 22.9 s
- which is the sentence "not wrong for a small body, only slow" turned into a number.

**Pass `--task` with it.** Without one the built-in task's `actions:` have no `grab`, so
every cell of a grabbing room looks like a failure; that case now reads `actions` rather
than `solver`, because what a body cannot do and what a task did not grant are different
facts.

### Correction: this body does not pick anything up yet (measured 2026-11-17)

M4 wrote "the arm picks things up - `tabletop/pick_and_place` 20/20, mean 2.9 s". **That is
wrong.** In the room as shipped, with that room's own task, at the moment the episode ends:

- success comes at 2.8 s (28 steps), `box.ok` is **never on at any point**, and the block has
  **not moved at all** from its start at (-0.05, -0.12), which is (-0.05, -0.26) m from the
  box centre.
- Traced: during the grab step the gripper moves **away** from the block. The block is at
  y = -0.12 and the gripper's y goes 0.001 -> 0.154, the wrong sign. After 11 steps the
  approach phase flips to "arrived", the command reverses, and it oscillates. The grab never
  happens, the demo steps advance on a stuck counter, and the goal 0.1 m away is reached by
  accident.

So M4's 20/20 measured **reaching the goal**, not grabbing and not carrying. My own commit
message earlier today ("grabbing and carrying are measured") is wrong for the same reason:
it rested on "the scripted solver cannot skip a demo step", and the stuck counter breaks
that premise.

The suspect is **direction**. This body's command is in the gripper's frame (`vx` along the
way the jaw faces), so an approach that uses a world-frame direction goes somewhere else on
a body whose heading is a function of its azimuth - the same class of error M4 recorded
fixing in `solver._steer` and `carry.steer` may remain on the `_close_in` path. Not written
down as the cause, because it is not confirmed.

Order of repair (R78): (1) make the approach go toward the object - until then every
grasping number for this body is void; (2) the win condition (`box.ok` has to be the win);
(3) only then new rooms. And making `mpg check` report "a demo step advanced on its stuck
counter" as a **failure** is the guard against this shape of false success.

**Partly repaired the same day (2026-11-17): it grabs now.** The cause was height, not
direction. The approach brings the jaw to within **2 mm horizontally** of the handle and
parks **0.102 m above** it - the whole remaining distance is vertical, against a 0.04 m grab
radius. A walking body pitches its head to lower the tip (`carry._hold_tip_at`); this body
had no implementation of that channel.

No boundary change was needed. `head` already means "lower the thing that grabs"
(`set_head`: 0 level, 1 lowest), and this body's answer is the jaw's height - from `WORK_Z`
0.12 down to `GRAB_Z` 0.02. A task that lists `actions: [move, head, grab]` lets the shared
carrier use it.

Measured: in `tabletop/pick_and_place` the block is **grabbed in 10 of 10 episodes** - the
first time this body has picked anything up. The room is 20/20 and its mean went from 2.9 s
to **17.8 s**, because it now does the work. The goal moved too: at (-0.02, 0) it sat *on
the path* from the block's approach point back to the block, so the episode ended before the
grab; it is now at (0.05, -0.25).

**Still not working**: `box.ok` fires in **0 of 10**. The block is released 0.11 m past the
box centre every time, at (-0.009, +0.11) from its centre. The suspect is `place_act`'s "the
cube leads the body" rule: a walking body holds the cube out in front on its beak and stops
when that lead passes the target, but here the cube is *at* the jaw, so the lead is zero and
the rule degenerates. That is the next piece of work, and until then this room's success is
still reaching the goal.

**Then two more bugs the same day: "stand still", and the aim point.** With the grab
working, the release showed what was under it.

- **`STAND` is a walking body's action.** It is `(-1, 0, 0)`, because this repository's
  convention is that `action[0] = -1` is the slowest a body goes. The arm's action maps
  symmetrically onto a signed velocity - its holonomic steering needs that symmetry - so
  the same vector is **full speed backwards**. The shared solver and carrier hand it out at
  **twenty-seven** call sites, every one meaning "wait", so this body drove away each time
  it was told to hold. Measured at the release: that "stand still" carried the block
  0.066 m further and lifted it to z 0.077 m, out of the box. The fix follows the pattern
  the last two bodies set - **the body declares it** (`BodyAdapter.stand_action`, default
  `(-1,0,0)` so walking bodies are untouched; the arm declares `(0,0,0)`). Third of this
  shape, after `SLOW` (M4) and the humanoid's `command_from_action` (M4).
- **The aim point was winding up.** The arm's command advances a moving aim point by
  `cmd * control_dt` each step, so while the jaw is blocked - pressing a block against the
  inside of a box - the aim keeps integrating and ends far ahead of the gripper. The
  command then stops and the arm keeps travelling toward an aim that ran away: measured,
  0.22 m further and up to z 0.146 m with a zero command throughout. `MAX_LEAD` (0.03 m,
  ten control steps at the command ceiling) keeps the aim within reach of the jaw.

**Result**: the block is released 0.08 m from the box centre instead of 0.30 m. `box.ok`
still fires in 0 of 10 (0.069 m in x against the box's 0.05 m inner half), so it lands
*beside* the box. The remaining path is not the `over` test but the carrier's **stuck
counter** (`_stuck` turns the phase to "back" and walks off while still holding), and that
is the next measurement. Returning the stand action on the step that decides "over" changed
the numbers not at all (0.069, -0.042 either way), so that change was reverted rather than
shipped unmeasured.

**And the win condition (R78 part two) closed with it.** The moment placing worked, the
approach that had failed before worked unchanged: `enable: box.ok` on the goal means the
goal does not count until the block is in the box. The earlier 0/5 was never the wiring -
it was that nothing ever went in the box.

| | full demo | demo cut to one `goto goal` |
|---|---|---|
| goal with `enable: box.ok` | 5/5 | **0/5** |
| ungated (as shipped before) | 5/5 | 5/5 |

The shipped `tabletop/pick_and_place` now carries the gate: **20/20 at a mean of 27.9 s**,
and unwinnable with its demo cut to a single step. No new part was needed - `enable` is a
field every part has and success is "a `role: goal` part is satisfied", so joining the two
was the whole change. The "goal whose satisfaction is a channel" part is **not built**: a
part with no place to be used is decoration (R67).

### A room that exists because of what this body is (R75, 2026-11-17)

`puzzles/tabletop/weigh_it_down`: hold the plate down **with something**. A walking body
stands on the plate and waits (`trigger: body`). This one has **no trunk to put anywhere** -
its only mass is the gripper, and the gripper has to be somewhere else next - so it
delegates: pick the block up, set it on the plate, and the block holds the plate while the
body goes to the goal.

Measured: **20/20 at a mean of 16.3 s**, and **0/5** with the demo cut to a single
`goto goal` step, because `enable: plate` gates the goal on the plate. The numbers come
from the envelope: block 0.29 m out, plate 0.33 m, goal 0.43 m, all inside the 0.48 m ring,
and `min_mass` 0.01 kg is half the block's 0.02 kg, so the plate reads pressed while the
block sits there and reads nothing once it is lifted off.

`mpg check --all-bodies` calls this room exclusive to the arm, and the detail matters more
than the word: the quadruped and the humanoid are refused **by name** (no `grab`), and the
microduck - which has a beak and could in principle delegate the same way - fails this
room's scripted solution. That last one is **not a designed exclusion**: on a 1.2 x 1.0 m
table its own footprint and the planner's collar leave it little room. The honest reading
is "two bodies cannot, one is not shown to".

### A new part, `stack` (R71, 2026-11-17)

Designed in M4 with its field table and never built. **Stacking already worked** - a `cube`
can rest on another and a room can declare it with `rest_on`. What was missing was a part
that *reads* a stack: every receiving part (bin, socket, basket, scale) reads "went in", and
none read a height - `mpg parts` had no part taking a `height` at all.

On when three things hold: `need` accepted objects have their centres inside `size`; the
highest of them is at least `height` up; and all of them have rested for `settle`. The
second is what separates a tower from a row - three cubes side by side fill the patch
(count 3) and reach nothing. Channels: its own 0/1 and `<name>.count`.

**No `needs_body`.** This part only looks; getting the objects there is the room's business.

Three things surfaced while writing it:

- **`accepts` is already a method protocol** (`grabbable.takes` calls `part.accepts(obj)`).
  The first draft named a *field* that, so a list shadowed the method and **no room holding
  a cube would load at all**. The field is `accept` now, as a receptacle's is, and the
  protocol is implemented.
- **`state()` is one scalar** (`obs.part_state` reads a number per part). Returning two blew
  up the observation. The count goes out on the channel instead.
- **`rest_on` is one level deep** (it refuses a cube that is "stacked itself"), so **a
  three-high tower cannot be declared in a room file** - it has to be built during the
  episode. The tests record that and cover `need=3` by placing the third cube directly.

The default `height` is arithmetic: stacking `need` default cubes (0.04 m) puts the top
centre at `(need - 0.5) * 0.04`, so 0.10 m for three, and the default is 0.09 with a
centimetre to spare. **A height the room's objects cannot reach is refused by
`check_static`** - otherwise `mpg check` only says "unsolvable" after the solver has failed
twenty times, and the reason is this number.

**Whether the arm can build a tower is not measured yet.** The part's decision is tested
(stacked reads on, a row does not, one short does not, moving does not, an unaccepted object
is not counted, an unreachable height is refused), but **there is no room**. A room stacking
two blocks on one patch was written and run, and the solution stalls at `place b1 tower` -
all five seeds end on demo step 1. So `stack` can read a tower that nothing has yet built,
and that room is the next piece of work. A part no room uses edges toward the decoration R67
forbids, so the fact is recorded here until the room is measured.

**A fourth declaration attribute: how far to clear a placed object** (`place_clearance`,
2026-11-17). The shared carrier backs off **0.35 m** from something it has just put down -
about a walking body's own length, because it has to get its trunk and beak clear before it
turns. A body whose grab point is its whole self needs only to clear the object, **and
cannot do more**: its base never moves, and a point 0.35 m from a placed object can be
outside its envelope.

Measured: with the walking number the arm's `place` step **never ended**. The block was
already resting where it had been asked for - 0.033 m from the patch centre against a
0.04 m tolerance - while the carrier circled its "away" phase for the rest of the episode.
With `place_clearance = 0.10` (its own footprint plus the object's size) the step ends, and
the stacking room's solution moved from demo step 1 to **step 2**. The default `None` keeps
the shared 0.35.

`stand_to_turn_above` (M3), `holonomic` (M4), then `stand_action` and `place_clearance`
(M5): the fourth and fifth of the same contract - **the body states its own nature** - with
`free_yaw` never built because no part needed it.

**The stacking room still does not work**: at the second `grab` the jaw does not close on
the second block (demo step 2, the block never moves). The first block is placed on the
patch. That is the next measurement.

**Where the stacking room's second grab stalls (measured 2026-11-17).** The first block is
placed on the patch. At the second `grab` the jaw stops **0.056 m from the handle** (it
needs 0.04). The command is **right** - it keeps driving toward the handle, body-frame -x,
which is inward toward the base - and yet the jaw sits at (-0.075, 0.110, 0.033) for more
than a hundred steps.

It is **not the envelope**. Sweeping the joints 7^5 at the grab height, the reachable radial
band at z 0.02 +/- 0.015 is **0.010-0.402 m** (0.012-0.478 at the 0.12 m work height), and
the target block is **0.190 m** from the base, inside the band. So a pose that reaches it
exists and the damped-least-squares solve, taking one step at a time, does not get there: a
local minimum. The jaw sitting 1.3 cm above the height it is being asked for is the same
symptom.

So `stack` still has no room filling it, and the next measurement is "why can this pose not
move inward". `reach_min` 0.0 is **correct** - the jaw does reach 0.01 m at that height -
and this is recorded as a control problem rather than a declaration one.

### The dead wedge can be bridged - but not by a turntable (R70, measured 2026-11-17)

The envelope is a **220-degree sector** (shoulder pan +/-1.920 rad), so the **140 degrees
behind the base** cannot be reached in any pose. The spec's R70 designed a room where a
**turntable** rotates an object out of that wedge, and the geometry says a turntable
**cannot**: a disk turns an object about the disk's own centre, which leaves its azimuth
*from the base* unchanged. The wedge is a region of angle and a disk is a device of radius;
bridging the wedge means **moving** the object.

A `conveyor` does that - it shifts whatever stands on it by `speed * dt` along its axis.
Measured: with the block 0.45 m from the base at **azimuth 180 degrees** (squarely in the
wedge) and a belt running from behind the base to in front of it, the block comes out at the
belt's end (radius 0.25 m, azimuth 0) and **the arm picks it up**. The wedge is bridgeable.

**The room is not finished**: after the grab, the place step lets the block go near the
belt's end rather than in the receptacle (block ends at (0.252, -0.023), box at
(0.22, 0.12)). And **R78's guard catches exactly that, by name** - `` `place block box`
ended with it not in 'box' ``, in 3 of 3 seeds. That is the guard's first catch outside the
case it was written for. Fixing that place is next, and until then this room is not shipped. Narrowed further: **the
hold is lost during the place** - at k=56, with the block still on the belt at
(0.065, 0.004), 0.194 m from the box, while the phase is `approach`. The belt is **not** the
cause: `conveyor` goes through `candidates()`, which skips objects the body holds, the same
convention the sensing parts follow. So the next measurement is "why does the weld let go
while approaching".

#### Why it lets go: the block is being dragged along the floor (measured 2026-11-17)

Reading the weld force per step: 0.133 -> 1.208 -> 1.604 -> 2.180 -> 2.127 N, released at
k=56. The limit is the cube's 3.0 N `max_hold_force`, and the force crosses it inside that
last step. Throughout, `block_z = 0.012` and `head = +1.00` - the jaw stays **at grab
height** while travelling, so the arm **drags the block along the floor**. Floor friction is
what builds the force. Not the direction, not the belt, not the envelope.

So the fix has to **lift the block after the grab**. The two obvious forms of it both
measured worse and were **not shipped**:

- Commanding `head = -1` as soon as the grab completes makes `set_head` **jump** the jaw
  target from 0.02 to 0.12 m in one control period, and the inverse kinematics yanks the
  block. The release moved **earlier**, from k=56 to k=52.
- **Rate-limiting** that jump to 0.5 m/s stops the grab from happening at all in the wedge
  room (never held once in 120 steps, demo_step 0).

So the right fix has to lift without a step change in the IK target and without breaking the
approach phase. It has not been found yet, and an unexplained change to shared code is not
shipped.

#### The deeper fact: at this point the arm **cannot lift** (measured 2026-11-17)

The gentle form was built and measured - stand still, raise the head by at most 0.02 units
per step, walk only once the block is clear. **It still tears** (0.30 -> 0.54 -> 1.35 ->
2.51 N, released at k=59). Going one level down says why:

| k | commanded `work_z` | actual `tip_z` | `block_z` |
|---|---|---|---|
| 52 | 0.0210 | 0.0212 | 0.0122 |
| 55 | 0.0240 | 0.0213 | 0.0122 |
| 58 | 0.0270 | 0.0221 | 0.0124 |

**The command rises 8 mm, the tip follows by 2 mm, and the block does not move at all.** The
inverse kinematics is saturated, and the gap it leaves loads the weld past 3 N. The radius
here is **0.065 m** - directly in front of the base, a folded posture. (In the room that
works, `pick_and_place`, the grab happens at radius 0.277 m.)

That gentle-lift code also **broke a room that already worked**: `pick_and_place` fell from
20/20 to **0/20** (stalled in the place). So it was **not shipped**.

What is left is geometry, not code: lifting works only in part of the envelope, and the point
where this room hands the block over is in the part where it does not. So the next move is not
"fix the lift" but **"make the belt put the block down at a radius the arm can lift from"**, or
have the body **declare** that minimum radius (the shape of M3's `stand_to_turn_above` and M4's
`holonomic`) and have the shared grab code pick it up there. The latter is a new declaration
attribute, so it is a spec item and not something to slip in here.

### Where the grab works is not a region but a **point** (measured 2026-11-17)

That geometric conclusion was carried out - the belt was laid **beside** the base (minimum
radius 0.32 m) so the hand-over happens at r=0.300. The grab worked and **the place tore in
exactly the same way** (the block dragged from r=0.300 to 0.193, then released). Next the belt
was kept entirely inside the dead wedge so the block comes to rest **on the floor** (measured:
(-0.048, 0.250), azimuth 101 deg, radius 0.255) - and then **the arm never completed the grab**
at all (400 steps).

So the workspace was swept on a grid. Around the shipped `pick_and_place` block position,
80x80 mm in 20 mm steps, 25 cells, box/door/goal untouched, one seed (the room declares no
`randomize:`, so an episode is deterministic):

```
         dy -40  -20    0  +20  +40   [mm]
dx -40    place place place place place
dx -20     OK    OK   place place place
dx   0    grab  place  OK   grab  grab
dx +20    grab  grab  grab  grab  grab
dx +40    grab  grab  grab  grab  (past place)
```

**3 of 25 cells succeed.** Everything with dx >= +20 mm stalls in the grab; everything with
dx <= -20 mm stalls in the place. Jitter says the same: `randomize.pos` +-2 mm 9/10, +-5 mm
8/10, +-10 mm 6/10, +-20 mm 3/10.

Two things follow.

1. **The declared envelope is where the tip can go, not where the scripted grab works.** The
   220 deg sector and the 0.001-0.480 m reach are facts about the tip. Picking a thing up and
   putting it down works at **a few points** inside that. That is why a spot the declaration
   calls reachable - azimuth 101 deg, radius 0.255 - is not one the arm can pick from.
2. **The shipped room's 20/20 is not 20 trials.** That room declares no `randomize:`, so 20
   seeds are the same episode, and the number means "it works at that one point". `mpg check`'s
   `resample` stage already says "skipped: the room declares no `randomize:`", but the `solver`
   stage's `'seeds': 20` reads as 20 trials. That is a reporting problem, and it is how it came
   to be reported here.

So the wedge room (R70) and the triage room (R74) are **not a matter of more geometry**: a room
that hands the object over at a new spot cannot be expected to work while the grab is a point
rather than a region. Both move from "room unfinished" to **"held until the scripted grab works
over a region"**.

#### Found alongside: clearing `_pick_z` per sub-goal is a bug AND a **guard**

`Carrier._pick_z` - the height an object was picked up at - is cleared in `reset()`, and the
solver resets the carrier at every solution step boundary. So the `place` after a `grab`
**always reads it as None**, and `_let_go`'s "set it down rather than drop it" has never run in
a scripted room (printed None from k=52 on). It belongs to the hold, so it was made to survive,
and the shipped pick-up room was unchanged at 20/20 and 27.9 s - but the slow suite then put
`examples/rooms/laundry.yaml` below its bar (the test passes without that change and fails with
it). The comment had claimed nothing would change for a walking body, whose beak already
carries clear. That was wrong.

So the clearing is a bug and at the same time a **guard that protects the walking bodies**.
Fixing it properly means handling their path too, so only the measurement was kept and the
change was not shipped.
