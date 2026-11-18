<p align="center"><a href="../README.md">한국어</a> · <b>English</b> · <a href="README.ja.md">日本語</a></p>

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="../assets/banner-dark.svg">
    <img src="../assets/banner-light.svg" width="640" alt="Pixel-art banner. As Claude types room.yaml line by line at a terminal, pieces sparkle into a top-view room on the right: walls with an opening in the divider, a yellow floor switch, a brown door, a blue goal circle and the signal link from the switch to the door. After a green check, a small duck robot walks onto the switch; the switch turns green and the door slides open. The duck walks through to the goal, sparkles pop, the terminal prints success 0.99 and Claude nods. The duck is driven by the trained policy, not by Claude.">
  </picture>
</p>

# mujoco-puzzle-gym — a puzzle lab that agents build

A framework in which AI coding agents build puzzle rooms inside a MuJoCo physics simulation and train a robot body in them. The robot is driven by a policy trained with reinforcement learning in those rooms; the agent never steers the robot directly. A puzzle is a program, not a fixed scene. You build a room whose state changes by wiring floor switches, doors, signals and logic together in YAML, and when a part is missing you register one more Python class. The agent runs the loop itself: build a room, check that it can be solved, train briefly, and when training fails, get a diagnosis and fix the room.

<p align="center"><img src="../assets/demo.gif" width="480" alt="Microduck simulation driven by a trained policy. The duck robot starts on the left of a room split by a wall, walks to a yellow floor switch and steps on it; the switch turns green and the door in the divider opens. The duck walks through the open door and reaches the blue goal circle on the right."></p>
<p align="center"><sub>A policy trained for 15 minutes steps on the switch, opens the door and reaches the goal on a layout it has not seen. Robot model: Pollen Robotics Microduck (CC BY-NC-SA 4.0).</sub></p>

## What is this?

- **Puzzles are built like programs.** A room is one `room.yaml`: walls, part positions, how much their positions vary each episode, and the signal links between parts. A new device is one Python class.
- **It checks a room can be solved first.** Before you spend time training, it checks in seconds that a solution order exists, that the physics is stable, and that a scripted solver can actually solve the room.
- **It trains, and tells you why training fails.** On failure it names the cause (environment bug, falling, not enough exploration, reward hacking) with evidence and a fix direction. An AI agent can run this loop without a person.
- **Rewards are configurable too.** Rewards are terms with weights in `task.yaml` (progress, stage, success, falling and so on). When `mpg diagnose` reports reward hacking or poor exploration, the agent changes the weights or registers a new reward term as one Python function, and trains again.

The agent is not included: you have the AI coding agent you already use (Claude Code, for example) edit the YAML and Python files and run the `mpg` commands. A person can use the same commands directly.

The first body is [Microduck](https://github.com/pollen-robotics/microduck_rl) by Pollen Robotics. Its published walking policy is used as is, and only "where to walk and how fast" is learned on top.

## Features

- **A part is a bundle of attributes.** Each part is a type (`type`) plus attributes: position, size, whether it stays on once stepped on, which way it opens, how much its position varies each episode, and so on. The parts and their attributes are listed in a separate [parts document](parts.en.md).
- **The agent designs puzzles by assembling parts.** Pick parts, place them and wire them with signal links (`links`), and you have a puzzle. Simple rooms like switch → door → goal and rooms with stacked conditions, such as a door that opens only when several switches are all on (`require: all`), are built the same way. A missing part can be written and plugged in.
- **Randomization is supported.** Today, part positions and the start position and heading are randomized within set ranges every episode, so a policy solves the puzzle instead of memorising the layout. Placing anywhere in a region, choosing one of several candidates, and randomizing the room's wiring and the room itself are in as well.
- **Baseline environments are included.** The example room, a verified PPO training config, a pretrained policy (`mpg demo`), a random-policy comparison on the same seeds (`mpg eval --policy random`), and a set of deliberately broken configs ([`examples/broken/`](../examples/broken/)) give a starting point to compare new rooms or new algorithms against.
- **The body can be swapped.** The puzzle code does not know the body; a body adapter takes movement commands and drives the body. Build a new body adapter and another robot trains in the same rooms (how: [guide](guide.en.md#body-adapter)). Four bodies are included today: two legs, four legs, a humanoid, and a **fixed-base arm that does not walk at all**. Because bodies differ in what they can do, a part declares the capabilities it needs and `mpg check` **refuses a body that lacks them, with the reason** — which part works with which body is tabulated in [bodies and parts](bodies.en.md).

## The parts that come with it

**68 built-in part types**, **87 presets** (the same parts with their values already set) and **44 room presets** (a whole room each). Every part's fields and numbers are in the [parts document](parts.md) and the [parts and presets guide](parts-guide.en.md), and a part that is missing can be added as a plugin.

| Group | Parts (`type`) |
|---|---|
| Goal and switch | `goal`, `switch` |
| Inputs and sensing | `button`, `pressure`, `laser`, `beam`, `zone`, `target`, `checkpoint`, `receptacle`, `stack`, `emitter`, `photocell` |
| Logic and wiring | `gate`, `counter`, `relay`, `wire`, `source`, `transmitter`, `receiver`, `budget` |
| Devices | `door`, `swing_door`, `trapdoor`, `rope_door`, `platform`, `conveyor`, `spring`, `turntable`, `warp`, `setter`, `pillar`, `low_bar`, `magnet`, `mirror`, `splitter`, `filter`, `goal_net`, `destroyer`, `rope`, `anchor`, `hitch_station`, `addon_station` |
| Floor | `floor_tile`, `coin`, `coin_group`, `coin_lock` |
| Objects | `cube`, `ball`, `pin` |
| Things to pick up and work | `key`, `egg`, `stick`, `handle_ball`, `laundry`, `lever`, `dial`, `drawer`, `pull_cord`, `dispenser` |
| Traps | `pit`, `pusher`, `spinner`, `pendulum`, `cannon`, `wind`, `brake_strip` |
| Robots that move on their own | `npc` |

## Take the challenge

There are **44 room presets**. **Three** have a trained policy's number. The other **41 cells are empty.**

<table>
<tr>
<td width="50%"><img src="../assets/challenge-hammer-lose.gif" width="300" alt=""><br><b>A deadline</b> - at 40 s a press smashes the coin lock. Fail to open the door first and the episode is over for good</td>
<td width="50%"><img src="../assets/gallery/tabletop_pick_by_hand.gif" width="300" alt=""><br><b>A real grasp</b> - no weld, two fingers pressing. Best trained policy <b>0.52</b></td>
</tr>
<tr>
<td width="50%"><img src="../assets/gallery/light_split_the_beam.gif" width="300" alt=""><br><b>Shares of a beam</b> - split one beam so two photocells each clear their threshold</td>
<td width="50%"><img src="../assets/gallery/ball_bowling.gif" width="300" alt=""><br><b>A chain</b> - push the ball, knock three pins down, the door opens</td>
</tr>
</table>

The scripted solution solves every room (the repository checks that over 20 seeds, every
time). A uniform random policy scores zero in almost all of them. **The gap between the two
is where your algorithm goes.**

> `puzzles/tabletop/pick_and_place` - PPO: **25 M steps, success 0.00**.
> Cloning the room's own scripted solution: **1.00 in 1.7 M steps**.
> How many steps does yours need, and for how much?

```bash
mpg train examples/task_pick_and_place.yaml --minutes 15 --id mine
mpg eval runs/mine --episodes 100                   # held-out seeds from 1,000,000
mpg eval runs/mine --episodes 100 --policy random    # the baseline on the same seeds
```

The other six rooms, the ladder over all 44, the protocol and the known records are in
[the challenge document](challenges.en.md).

## Install

On macOS or Linux, one line does it. It installs everything needed (uv, Python 3.12, dependencies, the robot model and walking policy, about 22 MB) and checks the example room to confirm the setup.

```bash
git clone https://github.com/Song-Seng-Hun/mujoco-puzzle-gym.git && cd mujoco-puzzle-gym && ./install.sh
```

Tested on macOS (Apple Silicon). Windows has not been tested yet.

## Try it

Make a video of the trained duck solving the example room (`demo.mp4`).

```bash
uv run mpg demo
```

Train your own. On a 12-core machine, 15 minutes of training solved 99% of unseen layouts ([training log](training-log.md)).

```bash
uv run mpg train examples/task_ppo.yaml --minutes 15 --id first
```

```bash
uv run mpg render runs/first
```

## Make your own puzzle room

Copy the example room [`examples/switch_door.yaml`](../examples/switch_door.yaml), change it, check that it can be solved, then train. In this room, stepping on the switch opens the door, and reaching the goal is success.

```yaml
name: switch_door
size: [2.4, 1.8]                  # interior size [m], centred at the origin
wall: {thickness: 0.06, height: 0.25}
walls:                            # divider at x = 0.3 with a 0.5 m opening in the middle
  - {from: [0.3, -0.9], to: [0.3, -0.28]}
  - {from: [0.3, 0.28], to: [0.3, 0.9]}
spawn: {pos: [-0.75, 0.0], yaw: 0.0, random: [0.2, 0.35, 3.14]}
parts:
  - {name: switch_a, type: switch, pos: [-0.25, 0.5], size: [0.3, 0.3], random: [0.15, 0.1]}
  - {name: door_b, type: door, pos: [0.3, 0.0], width: 0.5, axis: y}
  - {name: goal, type: goal, pos: [0.85, 0.0], radius: 0.15, random: [0.1, 0.45]}
links:
  - {from: switch_a, to: door_b}  # door_b opens when switch_a is on
```

```bash
uv run mpg check my_room.yaml
```

To train on this room, copy [`examples/task_ppo.yaml`](../examples/task_ppo.yaml) to `my_task.yaml`, set `room: my_room.yaml` and run `uv run mpg train my_task.yaml --minutes 15 --id my-room`. The room path is resolved from the folder of the training config.

All commands, training settings, adding parts and rewards, and swapping the body are in the [guide](guide.en.md).

## Progress

- [x] **Milestone 1**: the full loop of building, checking, training, diagnosing and evaluating a switch → door → goal puzzle. The built-in parts are switch, door and goal; other devices can be added as plugins.
- [x] **Milestone 2**: a part catalog
  - [x] Buttons, sensors, logic gates, counters, coins, warps, balls and goal nets, several kinds of door, checkpoints and curriculum training, ArUco markers and a gaze sensor, camera and distance-sensor observations. The part presets and the example rooms are in the [parts and presets guide](parts-guide.en.md).
  - [x] Moving devices (platforms, conveyors, springs), traps, robots that move on their own
  - [x] Cubes and beak grabbing, tow ropes (`puzzles/tow/`), more episode randomization (anywhere in a region, one of several candidates, the room's wiring and the room itself)
- [x] **Milestone 3**: a second body (a quadruped) and the body capability gate — a part declares what it needs from the body, and `mpg check` refuses a body that lacks it, with the reason
- [x] **Milestone 4**: a fixed-base arm and a humanoid, and puzzles whose answer is the difference between bodies — four bodies (duck, quadruped, arm, humanoid) share one boundary, the tabletop rooms are solvable only by a body with no legs, and the wide rooms only by a large one
- [x] **Milestone 5**: carrying a load — rooms where the arm picks something up and puts it in a box, on a tower or on a conveyor. What blocked it was the receiver's rim rather than the arm (the grab grid went 7/25 to 22-24/25)
- [x] **Milestone 6**: parts whose answer is weight or a budget — a lever (mass x distance) and a shared pool two doors draw from
- [x] **Milestone 7**: magnets and light (emitter, photocell, mirror, splitter, filter), a grasp held by two fingers pressing (20/20 with no weld in the scene), and **how close a policy gets to hardware**:
  - policies trained **on sensors alone** (proprioception, joint encoders and the wrist camera's gaze): reach 0.93, and 0.52 for carrying a block held by two fingers. With privileged observation a pick-and-place policy scores 1.00 - by cloning the room's own scripted solution rather than by reinforcement learning
  - actuator latency and detection error (position noise, dropout, calibration, lens distortion) in the model, and 0.94 trained under them
  - what is left is written as numbers too: the recipe for the finger-grasp carry lands one run in three, and five of ten arm rooms still fail at 40 ms of latency (the [training log](training-log.md))

## Limits

- By default the high-level policy uses privileged observations that read the puzzle state directly. Add `rangefinder` (head distance sensor) and `camera` (64×64 head camera) to `obs:` in task.yaml to train on real-sensor observations, and use `critic_obs:` to give privileged observations to the value function only. Policies that solve a room from sensors alone **do exist** - on the arm, reach 0.93 and 0.52 for a carry held by two fingers (with privileged observation a pick-and-place policy reaches 1.00). Camera training is still about 15× slower than privileged observation, so those policies read distances, joint encoders and marker gaze rather than the image itself.
- The high-level policy generalises to unseen layouts, and working the order out by itself has been tested (`order_cue` and `order_blind`: the same room with the cue on and off). The walking body's default rooms still always have the same order (switch → door → goal) with the reward pointing at it.
- The official walking policy is kept frozen. It cannot walk backward and walks forward at about 0.12–0.22 m/s. Room sizes and time limits have to fit this speed.
- Simulation only. Transfer to the real robot (sim2real) is out of scope.
- Tested on macOS (Apple Silicon, CPU MuJoCo). Rendering on Linux has not been checked yet.

## Built with AI

The planning, design documents, code, tests and training experiments in this repository were made by the author together with Claude (Anthropic). The design documents are in [specs/](specs/).

## Credits and license

- Code in this repository: [Apache-2.0](../LICENSE).
- Robot model and meshes: © Pollen Robotics, [microduck_rl](https://github.com/pollen-robotics/microduck_rl) pinned commit `0daab160`. The MJCF is Apache-2.0; the 3D meshes (`*.stl`) are [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) (non-commercial, share-alike). The meshes are not in this repository; `mpg fetch-assets` downloads them unmodified. The MJCF is edited only in memory.
- Walking policy: `velstand.onnx` from [pollen-robotics/microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies), Apache-2.0.
- Second body's model: `unitree_go2/` from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) pinned commit `370c4396`, BSD-3-Clause, model and meshes alike.
- The fixed-base arm's model: `robotstudio_so101/` from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) pinned commit `370c4396`, Apache-2.0. No usably licensed pretrained policy exists for it, so it is driven by inverse kinematics.
- The humanoid's model: `unitree_g1/` from the same commit, BSD-3-Clause. Its walking policy is `deploy/pre_train/g1/motion.pt` from [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) at a pinned commit, BSD-3-Clause — **this policy is not an official frozen release either**, but the checkpoint that repository ships with its deployment example.
- Second body's walking policy: [diasAiMaster/unitree-go2-velocity-flat](https://huggingface.co/diasAiMaster/unitree-go2-velocity-flat) pinned revision `9a723b7a`, BSD-3-Clause. **This policy is not an official Unitree release but a community checkpoint.** It was trained on flat ground (no `gait_phase`, action scale 0.5) with the upstream training code unmodified. The details are in [bodies and parts](bodies.en.md).
- The demo video and images show the CC BY-NC-SA 4.0 meshes.
- This is an unofficial project, not affiliated with Pollen Robotics.

## More

- [Guide](guide.en.md): the loop, commands, training settings, extending, body adapter
- [Agent guide](../AGENTS.md): what an AI agent working in this repository reads first
- [Sharing rooms and policies](community.en.md): submitting your rooms as PRs, sharing policies as ONNX, why we don't open other people's pickles
- [Parts and attributes](parts.en.md): the parts available now and their attributes
- [Training log](training-log.md): training results and diagnoses of broken settings
- [Speed measurements](bench.md)
- [Design documents](specs/)
