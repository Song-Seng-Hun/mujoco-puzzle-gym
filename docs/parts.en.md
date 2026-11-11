# Parts and their attributes

[Back to README](README.en.md) · [한국어](parts.md) · [日本語](parts.ja.md)

A puzzle room is assembled from parts. One part is one line in the `parts:` list of `room.yaml`, made of a **type (`type`) and its attributes**. Parts pass signals to each other through `links:`. This document lists the parts available now and their attributes.

## Attributes every part has

| Attribute | Meaning | Default |
|---|---|---|
| `name` | Part name. `links` and observations refer to the part by it | required |
| `type` | Part type (list below) | required |
| `pos: [x, y]` | Position [m], origin at the room centre | required |
| `random: [dx, dy]` | Range by which the position is shaken each episode [m], so a trained policy does not memorise spots | `[0, 0]` |
| `require: all \| any` | With several linked signals, whether all must be on or any one is enough | `all` |

Each part has a role. The check (`mpg check`) and the environment use roles to work out the solution order.

| Role | Meaning |
|---|---|
| `switch` | Has an on/off state and sends a signal |
| `gate` | Blocks the way until its linked signals are on |
| `goal` | Reaching it ends the episode as a success |
| `prop` | Anything else: decoration or obstacles |

## Built-in parts

### `switch` — floor switch (role `switch`)
On when the body centre is over the plate. No collision.

| Attribute | Meaning | Default |
|---|---|---|
| `size: [w, h]` | Plate size [m] | `[0.25, 0.25]` |
| `latch` | `true`: stays on once stepped on; `false`: on only while pressed | `true` |

### `door` — sliding door (role `gate`)
A panel that closes a gap in a wall. It slides into the wall while its linked switches are on and closes when they are off.

| Attribute | Meaning | Default |
|---|---|---|
| `width` | Width of the gap it closes [m] | `0.5` |
| `axis: x \| y` | Direction the panel lies along | `y` |
| `thickness` | Panel thickness [m] | `0.04` |
| `height` | Panel height [m] | `0.25` |
| `open_dir: 1 \| -1` | Which way it slides when opening | `1` |

### `goal` — goal (role `goal`)
Success when the body centre is within the radius. No collision.

| Attribute | Meaning | Default |
|---|---|---|
| `radius` | Goal radius [m] | `0.15` |

## Room attributes

| Attribute | Meaning |
|---|---|
| `size: [w, h]` | Interior size [m], origin at the centre |
| `wall: {thickness, height}` | Outer wall thickness and height |
| `walls:` | Inner walls, each `{from: [x, y], to: [x, y]}` |
| `spawn: {pos, yaw, random}` | Start position and heading, and the per-episode range `[dx, dy, dyaw]` |
| `links:` | Signal links, each `{from: part, to: part}` |

## Making a new part

A part is one Python class that inherits `Part`. It reads its attributes from `cfg` and declares its role with `role`. Register it with `register_part` and put the module name in `plugins:` in `task.yaml`; then `room.yaml` can use it by `type`. See the example in the [guide](guide.en.md#adding-parts-observations-rewards-and-algorithms).

## Parts on the way (milestone 2, in design)

Buttons (wall and floor), pressure plates, zone and laser sensors, receptacles, targets, logic gates and counters, transceivers, signal sources and wires, warps and springs, traps, beak-grabbed objects and devices (lever, dial, drawer, dispenser), self-moving robots and an owner, tow ropes, a wheel add-on station, destroyers, checkpoints, and puzzle presets that bundle them. Each will be added here with its attributes as it lands.
