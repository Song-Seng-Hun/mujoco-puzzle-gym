# Sharing rooms and policies

Send puzzle rooms you build to this repository as a PR. Share trained policies as ONNX files through an external link. Discussion happens in GitHub Discussions.

## What we accept and what we don't

| Accepted | Not accepted |
|---|---|
| Room files `*.yaml` (64 KB or less each) | Python files, plugin parts |
| A folder description `README.md` | `*.pkl`, `*.zip`, checkpoints, run folders |
| External links to policies (ONNX) | Policy files committed to the repository |

A room file cannot carry code. YAML is read with `yaml.safe_load`, and logic expressions are evaluated by an interpreter that allows only a fixed set of operations. That is why submissions are limited to room files.

## Submitting a room

1. Fork the repository and create a `community/rooms/<GitHub username>/` folder. Don't use subfolders.
2. Add your room files and, if you like, a `README.md`. An example is in `community/rooms/Song-Seng-Hun/`.
3. Check the room yourself before you submit.

   ```bash
   uv run mpg check --community --seeds 20 --json community/rooms/<name>/<room>.yaml
   ```

   The result must be `"ok": true`.
4. Open a PR. Include only files inside `community/`. Fill in the checklist in the PR template.

When the PR opens, the CI job `community` runs the same check on every changed room (10 minutes per room, 30 minutes in total). If everything passes, a maintainer reads it and merges. Nothing is merged automatically.

## What the `--community` check rejects

| Rejected | Why |
|---|---|
| A `plugins` key in the room file | Plugin modules run code |
| A `stages:` path pointing outside the room file's folder | Keeps the check from reading files elsewhere |
| Part types that are not built in | Plugin parts need a code review |
| More than 64 parts | Caps CI time and training cost |
| A room side longer than 10 m | Same reason |
| A text map side longer than 40 cells | Same reason |

`--community` cannot be combined with `--task`, because `plugins:` in a task file can load code. The check uses the built-in task.

## Sharing a policy (ONNX)

Policies don't go into the repository. Instead, convert them to ONNX, upload them somewhere like Hugging Face, and post the link in the `Rooms` category of Discussions.

```bash
uv pip install 'mujoco-puzzle-gym[export]'
uv run mpg export-onnx runs/<id>
```

Two files are written to `runs/<id>/onnx/`.

- `policy.onnx`: the policy network that outputs deterministic actions
- `policy.json`: observation terms and sizes, action range, SHA256 of the training task, room name

Only vector-observation policies (PPO, A2C, SAC) can be exported. Camera-observation policies are not supported yet.

A policy you receive is evaluated without pickle. Put both files in the same folder and pass your own task file for the room the policy was trained on with `--task`:

```bash
uv run mpg eval --onnx policy.onnx --task task.yaml --episodes 100
```

If the observation size or action size differs from the task, evaluation is refused. If the sizes match but the observation terms differ, the result carries `warnings`.

## Don't open other people's run folders or pkl files

SB3 checkpoints (`checkpoint.zip`), checkpoint saves (`*.pkl`) and `task.resolved.yaml` in a run folder can run code when opened. So `mpg` reads these files only under `runs/` in the current folder and from the demos bundled in the repository. Any other path is refused with a reason.

Add `--trust` only when you moved files you made yourself to another place (`train`, `eval`, `diagnose`, `render`, `export-onnx`). `--trust` sets the environment variable `MPG_TRUST=1` while that command runs. Don't keep this variable set in your shell; that leaves the path check switched off.

## Discussions

- `Rooms`: show off rooms you built, policy links
- `Q&A`: questions
- `Ideas`: proposals for new parts or features
