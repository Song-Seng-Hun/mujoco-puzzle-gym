# R25 fresh-worker trial — result: PASS

Run 2026-11-14 on branch `feat/m2-docs-17b` (PR #42, not yet merged). Criterion fixed before the
run in `criterion.txt`. Worker: a fresh general-purpose agent, given only the repository path and
`skills/mujoco-puzzle-gym/SKILL.md`. It asked nothing and was told nothing further. 39 tool calls,
about 8 minutes, 165k tokens.

## Verified by the desk, not taken from the worker's report

- `mpg check <room> --seeds 20` -> `ok: True`, exit 0. Static: values, channels, demo, body, parts,
  overlap, order, circuit, actions all ok. Dynamic: stability ok; `solver` **20/20 seeds**
  (need 18), mean 32.78 s, no failures. Three checkpoint snapshots written
  (`runs/check/r25_order_hall/checkpoints/flag_{1,2,3}.pkl`).
- All three required features are really there, read off the YAML: `flag_1/2/3`
  (`input/checkpoint_flag`, `order: 1/2/3`), a `type: gate` `sequence: [btn_a, btn_b, btn_c]` with
  `latch: true` driving `device/sliding_door`, and `to: next_stage` warp with
  `stages: [r25_vault.yaml]`, the goal living in the vault.
- **Negative control (desk-added, not the worker's):** same room with the press order reversed in
  `demo:` -> `solver` **0/10**, every seed stuck at `demo_step 6` targeting `to_vault` at 40.0 s,
  i.e. all three buttons pressed and the door still shut. The order lock is real, not decorative.
- The repository working tree stayed clean (`git status` empty); only the gitignored
  `runs/check/...` was written.

## Documentation gaps the trial exposed (all four code claims checked at the source)

1. **`stages:` is undocumented.** `docs/guide.md` never mentions it; `docs/parts-guide.md` has two
   incidental mentions, both inside the randomization section as "the same mechanism as `stages:`".
   The rules only exist in the `room.py` module docstring ("Stages (R14)"), and the only worked
   example is `tests/test_warp.py::stage_rooms`. `examples/rooms/warp_hop.yaml` is a place warp, so
   the documented warp example does not show this at all.
2. **A `sequence:` gate's output is a one-step pulse unless `latch: true`.** `gate.py:118`
   (`self.done = self.latch`). The guide's sequence-lock line only mentions `<name>.wrong`; the
   value table says `latch` makes it "stay on once the order is complete", which does not say what
   happens without it. A `mode: follow` door cannot be held open by the pulse.
3. **The static `order` planner models a sequence gate as a plain AND** (`gate.py:152`
   `plan_values`: `all(is_on(...))`). So `order` passes a room whose press order the scripted
   solver will then get wrong, and the unstated rule is that an order lock must spell its press
   order out in `demo:`.
4. **Oneshot buttons make an order lock one-shot.** A latched button has no second rising edge, so
   one stray early press kills the lock for the whole episode, with no error anywhere — just
   `solver` 0/N. This dictated the worker's layout (flags on one edge, buttons on the other).
5. **Checkpoint flags are not in the solution order.** Confirmed in the desk run: `order` returned
   `['btn_a', 'btn_b', 'btn_c', 'vault_door', 'to_vault', 'vault_goal']`. A passing `order` stage is
   therefore no evidence that the flags are reachable in sequence.
6. **Nothing relates episode length to route length, and the failure reads like a broken room.** A
   route too long for the 40 s default fails `solver` with `successes: 0`; the tell is
   `demo_step` == the number of demo steps together with `seconds` == the episode length. The
   worker hit exactly this (8.3 m route) and first worked around it with a 90 s task before
   shrinking the room so the bare `mpg check` passes.

Minor, nothing to fix: `mpg build` writes into the gitignored `build/`.

## Files

`r25_order_hall.yaml` (room), `r25_vault.yaml` (the next stage), `task_r25_order_hall.yaml`
(60 s, optional), `neg_wrong_order.yaml` (desk negative control), `worklog.md` (the worker's own
log), `parts.md` (the catalog it saved), `criterion.txt`, this file.
