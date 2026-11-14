# Agent guide

For an AI agent working in this repository. Humans start at [README.md](README.md).

`mujoco-puzzle-gym` builds puzzle rooms from YAML, checks that they are solvable, and trains a
high-level policy on top of a frozen walking policy for the Microduck body. The agent-facing loop
is: **build a room → check it → train → diagnose → evaluate**.

Docs come in three languages and the Korean file is the source: `docs/<name>.md` (ko),
`docs/<name>.en.md`, `docs/<name>.ja.md`. Change all three together.

| Read | For |
|---|---|
| [docs/guide.md](docs/guide.md) ([en](docs/guide.en.md) · [ja](docs/guide.ja.md)) | the loop, every command, task.yaml, plugging in parts/obs/rewards/algos, body adapters |
| [docs/parts-guide.md](docs/parts-guide.md) ([en](docs/parts-guide.en.md) · [ja](docs/parts-guide.ja.md)) | every part and preset with its values, assembly presets, the signal bus, expression gates, the two check phases, staged training, episode randomization |
| [docs/parts.md](docs/parts.md) ([en](docs/parts.en.md) · [ja](docs/parts.ja.md)) | the short part list |
| [docs/community.md](docs/community.md) ([en](docs/community.en.md) · [ja](docs/community.ja.md)) | sharing a room by PR, exporting a policy to ONNX, why foreign pickles stay closed |
| [docs/specs/](docs/specs/) | intent, spec and plan per milestone: what is required (`R<n>`) and how it is proved |
| [docs/training-log.md](docs/training-log.md) | measured training results, including negative ones |

## Setup

```bash
./install.sh            # uv sync, Python 3.12, pinned assets (SHA256-checked), example check
uv run mpg version
```

`mpg fetch-assets` pins the robot model and walking policy to fixed upstream commits. Never edit
`src/mujoco_puzzle_gym/upstream.lock.json` to get a different asset.

## Building a room from presets

The full rules (merge order, assembly presets, value tables) are in
[parts-guide.md § 프리셋으로 방 짜기](docs/parts-guide.md#프리셋으로-방-짜기). The order:

1. `uv run mpg parts --markdown` — pick the presets to use. Do not invent part types or values;
   everything registered is in that output and in the guide's value tables.
2. Write the room: `size`, `wall`, `walls`, `spawn`, then one `parts` line per part
   (`{name: b, preset: input/floor_oneshot, pos: [x, y]}`) with only the values you change.
3. Wire the signals: the receiving part's `in:` lists the emitting part's channel names
   (`require: all | any`), or use `links:`. Expression gates take `expr:` — arithmetic is rejected,
   so spell conditions out with and/or/not.
4. `uv run mpg check <room>.yaml --static` for values, channels, overlap and solution order, then
   the full `uv run mpg check <room>.yaml` for stability and the scripted solver (needs 18/20 seeds).
   `--seeds N` changes the seed count. Fix the room, not the threshold.
5. Only then write `task.yaml` (room, `obs`, `reward`, `algo`, `randomize`) and train.

A room that the scripted solver cannot solve is not ready for training: `mpg check` failing is the
answer, not an obstacle. `examples/broken/` holds rooms broken on purpose, with the check message
each one is supposed to produce.

## Rules that will bite you

- **Pickles.** `train`, `eval`, `diagnose`, `render` and `export-onnx` refuse run folders and
  snapshots outside the current directory's `runs/`, because they are pickles. `--trust` is only
  for files you made yourself.
- **Value tables are generated.** The tables in `parts-guide.md` must equal
  `mpg parts --markdown` exactly, between the two marker lines. `tests/test_guide.py` fails
  otherwise, and it also fails if a registered part or preset name is missing from the guide. So
  adding a part means updating the guide in the same change.
- **Tests are split.** `uv run pytest` runs the fast set (`addopts = -m 'not slow'`). Long
  20-seed checks and video renders are `-m slow`; `-m ''` runs everything. `-m network`
  downloads the pinned assets when the cache is empty.
- **Shared rooms are limited.** `mpg check --community` applies the extra limits for rooms
  contributed to `community/rooms/`.
- **Measure, don't assert.** When a room's or policy's success rate matters, run the seeds and
  write the numbers (and the failure seeds) into the spec's plan record or
  `docs/training-log.md`. Negative results are recorded too.

## Spec documents

Each milestone has `docs/specs/<date>-<slug>/{intent,spec,plan}.md`. `spec.md` holds numbered
requirements `R<n>` with the check that proves each one; `plan.md` holds the build order and, under
each step, the implementation record with measured numbers and every place the build departed from
the plan. If you implement a step, write that record in the same change.
