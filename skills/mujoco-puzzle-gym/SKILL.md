---
name: mujoco-puzzle-gym
description: Build, check, train and evaluate puzzle rooms in a mujoco-puzzle-gym checkout (YAML rooms for the Microduck body over a frozen walking policy). Use when asked to make or fix a room (`examples/rooms/*.yaml`, `community/rooms/`), write a training task, run `mpg check|train|diagnose|eval|render`, add a part, observation, reward or algorithm, or read why a check or a training run failed.
---

# mujoco-puzzle-gym

Rooms are YAML. A room is ready when `mpg check` passes; a failing check is the answer, not an
obstacle. Fix the room, never the threshold.

## Orient first

Read [AGENTS.md](../../AGENTS.md) in the checkout: it is the agent entry point and maps every doc.
Then, for the task at hand:

- parts, presets, their values, the signal bus, `expr:` gates, the two check phases, staged
  training, episode randomization -> `docs/parts-guide.md`
- commands, `task.yaml`, plugging in parts/obs/rewards/algos, body adapters -> `docs/guide.md`
- what the current milestone requires -> `docs/specs/<latest>/spec.md` (numbered `R<n>` + its check)
  and `plan.md` (build order + implementation records)

Docs are Korean-source with `.en.md` and `.ja.md` beside them; change all three together.

## Build a room

```bash
./install.sh                              # once: uv sync + pinned assets
uv run mpg parts --markdown               # 1. the only parts and values that exist
uv run mpg build room.yaml                # 3. MJCF + top-down PNG, look at it
uv run mpg check room.yaml --static       # 4a. values, channels, demo, body, parts, overlap, order
uv run mpg check room.yaml                # 4b. + stability and the scripted solver, 18/20 seeds
```

2. between them: `size`, `wall`, `walls`, `spawn`, one `parts` line per part
(`{name: b, preset: input/floor_oneshot, pos: [x, y]}`, only the values you change), then wiring —
the receiver's `in:` lists emitter channel names (`require: all | any`). `expr:` rejects
arithmetic, so spell conditions out with and/or/not.

5. only then `task.yaml` (`room`, `obs`, `reward`, `algo`, `randomize`) and
`uv run mpg train task.yaml --minutes N --id <id>`, then `mpg diagnose` / `mpg eval runs/<id>`.

## Non-negotiable

- Never invent a part type, preset or field: if it is not in `mpg parts` output, it does not exist.
- Pickles outside `./runs/` are refused by `train|eval|diagnose|render|export-onnx`. `--trust` is
  only for files you made.
- `parts-guide.md` value tables must equal `mpg parts --markdown` between its marker lines, and
  every registered part and preset name must appear in it — `tests/test_guide.py` enforces both.
- `uv run pytest` is the fast set; long seed checks and renders need `-m slow` (`-m ''` for all).
- Measure before claiming: run the seeds, write the numbers and the failing seeds into the plan
  record or `docs/training-log.md`. Record negative results too.
