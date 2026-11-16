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
| Reach height | z −0.223 … 0.520 m | The same sweep. The capability value `reach_height` is declared conservatively as 0.45 |
| Working band | out to a radius of 0.480 m at z 0.02–0.15 m | The same sweep |
| Envelope shape | a 220° **sector** (shoulder rotation ±1.920 rad) | A sector, not a ring. The room has to fit inside it |
| Degrees of freedom | 5 arm axes + 1 jaw axis, no free joint | `nq = nv = 6` |
| Mass | 0.645 kg | The model's `body_subtreemass` |
| Gripper tracking | about 1 cm of error over a 10 cm move at mid radius | Measured by giving the command 2 seconds. Folded back toward the base it grows to 5 cm |

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
