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
- **Randomization is supported.** Today, part positions and the start position and heading are randomized within set ranges every episode, so a policy solves the puzzle instead of memorising the layout. Placing anywhere inside a region, picking one of several candidates, and changing the room structure itself come in milestone 2.
- **Baseline environments are included.** The example room, a verified PPO training config, a pretrained policy (`mpg demo`), a random-policy comparison on the same seeds (`mpg eval --policy random`), and a set of deliberately broken configs ([`examples/broken/`](../examples/broken/)) give a starting point to compare new rooms or new algorithms against.
- **The body can be swapped.** The puzzle code does not know the body; a body adapter takes movement commands and drives the body. Build a new body adapter and another robot trains in the same rooms (how: [guide](guide.en.md#body-adapter)). The only body included today is Microduck.

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
- [ ] **Milestone 2**: a part catalog of buttons, sensors, logic gates, counters, coins, warps, traps, beak grabbing, checkpoints and curriculum training (in progress)
- [ ] **Milestone 3**: puzzle parts for other bodies, such as arms and wheeled robots

## Limits

- The high-level policy uses privileged observations that read the puzzle state directly. There is no policy yet that solves puzzles from real sensors (camera, distance sensor) alone.
- The official walking policy is kept frozen. It cannot walk backward and walks forward at about 0.12–0.22 m/s. Room sizes and time limits have to fit this speed.
- Simulation only. Transfer to the real robot (sim2real) is out of scope.
- Tested on macOS (Apple Silicon, CPU MuJoCo). Rendering on Linux has not been checked yet.

## Built with AI

The planning, design documents, code, tests and training experiments in this repository were made by the author together with Claude (Anthropic). The design documents are in [specs/](specs/).

## Credits and license

- Code in this repository: [Apache-2.0](../LICENSE).
- Robot model and meshes: © Pollen Robotics, [microduck_rl](https://github.com/pollen-robotics/microduck_rl) pinned commit `0daab160`. The MJCF is Apache-2.0; the 3D meshes (`*.stl`) are [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) (non-commercial, share-alike). The meshes are not in this repository; `mpg fetch-assets` downloads them unmodified. The MJCF is edited only in memory.
- Walking policy: `velstand.onnx` from [pollen-robotics/microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies), Apache-2.0.
- The demo video and images show the CC BY-NC-SA 4.0 meshes.
- This is an unofficial project, not affiliated with Pollen Robotics.

## More

- [Guide](guide.en.md): the loop, commands, training settings, extending, body adapter
- [Parts and attributes](parts.en.md): the parts available now and their attributes
- [Training log](training-log.md): training results and diagnoses of broken settings
- [Speed measurements](bench.md)
- [Design documents](specs/)
