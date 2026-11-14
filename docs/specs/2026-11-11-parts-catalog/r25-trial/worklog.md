# r25-trial worklog

Goal: one room with (1) a button-order lock, (2) three checkpoint flags, (3) a warp to the
next room. `mpg check` must exit 0.

## Documents read, in order

1. `skills/mujoco-puzzle-gym/SKILL.md` — entry point, says read AGENTS.md then the docs.
2. `AGENTS.md` — the loop (build → check → train), doc map, "fix the room, not the threshold".
3. `examples/rooms/{order_cue,order_blind,four_flags,room_roulette,room_roulette_b,logic_lock}.yaml`
   — order lock with an `expr:` gate, checkpoint flags, zone-based room drawing.
4. `uv run mpg parts --markdown` (saved to `parts.md` in this dir) — part/preset catalog.
   Found `gate.sequence` (sequence lock), `checkpoint` (`input/checkpoint_flag`), `warp`
   (`to: {pos,yaw} | reset | next_stage`).
5. `docs/parts-guide.en.md` — signal bus, expression gates, two-phase check (static stages
   values/channels/demo/body/parts/overlap/order/actions; dynamic stability/solver 18/20),
   checkpoints & staged training, assembly example table.
6. `examples/rooms/warp_hop.yaml` — a plain place warp.
7. Source, because the docs did not cover `stages:` (see "Gaps" below):
   `src/mujoco_puzzle_gym/room.py` module docstring (stages = extra room files, each built as
   a zone to the +x side, part names unique across stages, goal requirement holds for the
   stages together), `src/mujoco_puzzle_gym/parts/device/warp.py`,
   `tests/test_warp.py` (`stage_rooms`, `test_next_stage_warp`,
   `test_order_goes_through_the_warp`) — the only working example of a `stages:` room.
8. `src/mujoco_puzzle_gym/parts/logic/gate.py` — sequence-lock semantics and, crucially,
   `plan_values`, which models a sequence gate as a plain AND of its inputs.

## Design decisions from that reading

- Order lock: `type: gate` with `sequence: [btn_a, btn_b, btn_c]`, `latch: true`. Without
  `latch` the gate writes 1 for a single body step only, which cannot hold a
  `mode: follow` door open.
- Buttons: `input/floor_oneshot` so each press is one rising edge.
- The planner treats the sequence gate as an AND, so the static `order` stage would let the
  solver press in any order. The press order therefore has to be given explicitly in `demo:`.
- Flags along the top edge, buttons along the bottom edge, so the solver's route to a flag
  never crosses a later button (an out-of-order press of a oneshot button would lock the
  room for good — no second rising edge).
- Warp: `to: next_stage`, behind the door; `stages: [r25_vault.yaml]` holds the goal.

## Attempts

### Attempt 1 — first draft, size [3.0, 2.0]

`mpg check r25_order_hall.yaml --static` → exit 0 on the first try. All static stages ok;
solution order `['btn_a', 'btn_b', 'btn_c', 'vault_door', 'to_vault', 'vault_goal']`
(note: the flags are not in the solution order — the order stage reasons about signals and
walls to the goal, and the checkpoints are handled by the solver itself).

`mpg check r25_order_hall.yaml` (dynamic) → **FAIL**:

```
solver: {'ok': False, 'successes': 0, 'seeds': 20, 'need': 18,
         'failures': [{'seed': 0, 'fallen': False, 'nan': False, 'target': 'to_vault',
                       'seconds': 40.0, 'demo_step': 6}, ... ]}
dynamic: {'ok': False, 'failed': ['solver']}
```

Reading it: `demo_step: 6` means every demo step was done — all three flags passed (the
three flag snapshots were written) and all three buttons pressed in order, so the lock,
the door and the flags all worked. `fallen: False`, `nan: False`, `seconds: 40.0`, and the
target is `to_vault` or `vault_goal`: it simply ran out of episode time on the way to the
warp or to the goal after warping. Pure route-length problem.

### Attempt 2 — longer episodes via a task file

`episode_seconds` comes from `task.yaml` (`task.py` DEFAULTS: 40 s), and `mpg check
--task` takes one. Wrote `task_r25_order_hall.yaml` with `episode_seconds: 90`.

`mpg check r25_order_hall.yaml --task task_r25_order_hall.yaml` → **PASS**,
`successes: 20 / 20`, `mean_success_seconds: 44.84`.

### Attempt 3 — make it pass the bare `mpg check` too

A room that only passes with a custom task is weaker, so instead of buying time I
shortened the route: `size` [3.0, 2.0] → [2.2, 1.6], divider x 0.75 → 0.45, flags and
buttons pulled in, vault [1.8, 1.4] → [1.2, 1.0]. Route ≈ 8.3 m → ≈ 5.8 m.

- `mpg check r25_order_hall.yaml` (built-in 40 s task) → **PASS**, 20/20,
  mean 32.78 s.
- `mpg check r25_order_hall.yaml --seeds 40` → **PASS**, 40/40, mean 32.88 s
  (checked the 7 s margin was not luck of 20 seeds).
- `mpg check r25_order_hall.yaml --task task_r25_order_hall.yaml` (now 60 s) → **PASS**,
  20/20.
- `mpg build` picture inspected: flags along the top, buttons along the bottom, the door
  in the divider, the warp pad behind it, the vault drawn as a separate zone to the +x
  side with the goal in it.

No other failures occurred. `build/` output was removed again afterwards; `git status` in
the repository is clean and only the gitignored `runs/check/r25_order_hall/` was written.

## Gaps — what the documentation did not tell me, and where I found it instead

1. **`stages:` is essentially undocumented.** The task needs "a warp that leads to the
   next room", and `mpg parts --markdown` says a warp's `to` can be `next_stage`. Nothing
   in `docs/guide.en.md` or `docs/parts-guide.en.md` says what a stage is or how to
   declare one — `grep -n stages docs/*.en.md` returns only two incidental mentions (a
   community-limit row, and a parenthetical in the `randomize: {rooms:}` section saying
   "the machinery `stages:` uses"). The assembly-example table lists `warp_hop.yaml`,
   which is a plain `to: {pos, yaw}` warp, not a stage warp. I had to read the module
   docstring of `src/mujoco_puzzle_gym/room.py` ("Stages (R14)") for: `stages:` is a list
   of room files, relative to the room file; each is built as its own zone to the +x side
   in the same model; part names must be unique across all stages while channels are
   shared; and — important for the layout — *the goal requirement holds for the stages
   together*, so the first room may have no goal. The only worked example anywhere is
   `tests/test_warp.py::stage_rooms`.
2. **A `sequence:` gate needs `latch: true` to hold a door open.** `parts-guide.en.md`
   mentions `sequence: [a, b, c]` in one line and only tells you about `<name>.wrong`.
   That the gate writes 1 for a *single body step* and then rearms is in the docstring of
   `src/mujoco_puzzle_gym/parts/logic/gate.py`; the `latch` field exists in the value
   table with the terse doc "sequence lock: stay on once the order is complete", but
   nothing says a `mode: follow` door needs it. Without it the door blips open for one
   step.
3. **The static `order` planner does not model the sequence at all.** `Gate.plan_values`
   in `gate.py` returns `all(inputs on)` for a sequence gate — a plain AND. So the
   `order` stage passes a room whose buttons the solver will then press in the wrong
   order, and the documented stage description ("whether the signals and walls allow a
   solution order") gives no hint of this. The consequence is a rule nothing states: an
   order lock must have its press order written out in `demo:`, or the solver stage will
   fail for a reason the static phase called fine. I found this only by reading the
   source before writing the room.
4. **Oneshot buttons make an order lock one-shot too, and no doc warns about it.** A
   oneshot button latches on, so it produces exactly one rising edge per episode. One
   stray early press — e.g. the solver's route to a flag crossing a later button — breaks
   the lock permanently, with no error anywhere, just a silent 0/20 at the solver stage.
   This is only inferrable from `_sequence()` in `gate.py` plus the button's `kind`
   semantics. It drove the whole layout (flags on one edge, buttons on the other).
5. **The solution order silently omits checkpoint flags.** `order:` came back as
   `['btn_a', 'btn_b', 'btn_c', 'vault_door', 'to_vault', 'vault_goal']` with no flag in
   it. The docs describe the stage as "whether the signals and walls allow a solution
   order that reaches the goal" and never say the flags are outside it, so a passing
   `order` is not evidence that the flags are reachable in their order; only the dynamic
   solver stage checks that.
6. **Nothing says how long an episode may be, or that the room must fit the default.**
   `episode_seconds` is documented as a `task.yaml` key, but the check's default task
   (40 s, `task.py` DEFAULTS) is not mentioned where rooms are built, and no doc gives a
   rule of thumb for route length vs. episode time. The only hints are comments inside
   `examples/task_laundry.yaml` and `examples/task_four_flags.yaml` ("the scripted solver
   needs 21.8 s on average"). The failure mode is also not described: a room that is
   merely too big fails the `solver` stage with `successes: 0`, which reads like a broken
   room rather than a long one. The tell is `demo_step` equal to the number of demo steps
   plus `seconds` equal to the episode length.
7. **Minor:** that `mpg build` writes into the repository's `build/` folder is not
   mentioned by the SKILL or AGENTS (it is gitignored, so harmless here, but it matters
   when told not to touch the tree).
