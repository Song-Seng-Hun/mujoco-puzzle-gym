"""Signal bus (R1): named channels carrying floats between parts.

- A channel holds a float. On/off signals use 0 and 1; a value above `ON`
  (0.5) reads as on. Counters put their integer count on the channel as is.
- Each part reads channels (`in`), writes channels (`out`) and may have a power
  channel (`enable`; none = always on). A part whose class sets `emits` writes
  the channel named after itself when `out` is not given (switch `switch_a`
  writes channel `switch_a`).
- Every body step the bus updates each part once: input parts, then logic
  parts, then device parts. Inside a stage, a part that reads another part's
  channel runs after it, so a change crosses input -> logic -> device in the
  same step. A read against the stage order (a logic part reading a device's
  channel) sees the previous step's value.
- A loop of channels through stateless parts only has no evaluation order and
  is rejected; a loop through a stateful part (delay, latch) is allowed, since
  the stateful part's output does not depend on this step's input.
- A part off stage (`Part.parked`: R35's storage shelf, a dispenser's stored copy)
  is not updated and its channels stay at 0.
- A part's `bind` sources (R13, `Part.watch`) are reads too: the bus applies
  them (`Part.apply_binds`) right before the part's update.
- M1 `links: [{from, to}]` are channels: the link adds the source's channel to
  the target's `in` (room.py).

`wire(parts)` checks the channels and returns the bus; it raises `SignalError`
(a ValueError) for unknown channels, two writers of one channel, or a stateless loop.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import mujoco

    from .bodies import BodyState
    from .parts.base import Part

STAGES = ("input", "logic", "device")
ON = 0.5


class SignalError(ValueError):
    """Invalid channel wiring; the message names the parts and channels involved."""


def is_on(value: float) -> bool:
    return value > ON


class SignalBus:
    """Channel values plus the order in which parts update every body step."""

    def __init__(self, order: list["Part"], writers: dict[str, str]):
        self.order = order                  # parts in evaluation order
        self.writers = writers              # channel -> name of the part writing it
        self.values: dict[str, float] = {ch: 0.0 for ch in writers}
        self.dt = 0.02                      # seconds per body step (the env sets it)

    def reset(self) -> None:
        for ch in self.values:
            self.values[ch] = 0.0

    def get(self, channel: str) -> float:
        return self.values[channel]

    def on(self, channel: str) -> bool:
        return is_on(self.values[channel])

    def set(self, channel: str, value: float) -> None:
        self.values[channel] = float(value)

    def step(self, model: "mujoco.MjModel", data: "mujoco.MjData", body: "BodyState") -> None:
        """One body step: update every part once in evaluation order. A part off stage
        (R35 storage shelf, R28 a dispenser's stored copy) is skipped, so it writes
        nothing while it is away."""
        for part in self.order:
            if part.parked:
                continue
            if part.binds:
                part.apply_binds(model, data)
            part.update(model, data, body)

    def snapshot(self) -> dict[str, float]:
        return dict(self.values)

    def load(self, values: dict[str, float]) -> None:
        self.values.update(values)


def _reads(part: "Part") -> list[str]:
    return list(part.inputs) + ([part.enable] if part.enable else []) + list(part.watch)


def _find_stateless_cycle(parts: dict[str, "Part"], edges: dict[str, list[str]]) -> list[str] | None:
    """A loop (as part names, first repeated at the end) through stateless parts only."""
    color = {n: 0 for n in parts}           # 0 new, 1 on stack, 2 done
    stack: list[str] = []

    def visit(n: str) -> list[str] | None:
        color[n] = 1
        stack.append(n)
        for m in edges[n]:
            if color[m] == 1:
                return stack[stack.index(m):] + [m]
            if color[m] == 0 and (found := visit(m)):
                return found
        stack.pop()
        color[n] = 2
        return None

    for n in parts:
        if color[n] == 0 and (found := visit(n)):
            return found
    return None


def _stage_order(names: list[str], edges: dict[str, list[str]], index: dict[str, int]) -> list[str]:
    """Topological order of `names` along `edges` restricted to them; ties by room order."""
    members = set(names)
    indeg = {n: 0 for n in names}
    for n in names:
        for m in edges[n]:
            if m in members:
                indeg[m] += 1
    ready = sorted((n for n in names if indeg[n] == 0), key=index.__getitem__)
    out: list[str] = []
    while ready:
        n = ready.pop(0)
        out.append(n)
        for m in edges[n]:
            if m in members:
                indeg[m] -= 1
                if indeg[m] == 0:
                    ready.append(m)
                    ready.sort(key=index.__getitem__)
    return out


def wire(parts: dict[str, "Part"], bus: SignalBus | None = None) -> SignalBus:
    """Check the channels of `parts` (room order) and give every part the shared bus.

    Also sets each part's `sources` to the parts writing its `in` channels (the
    room-grid planner reads them).

    With `bus`, the same bus is wired again in place (R35 `shuffle`: an episode
    draws which part feeds a reader, so the evaluation order and the sources are
    recomputed at every reset). Channel values of channels that are still there
    are kept; a wiring the checks refuse raises `SignalError` as at load time.
    """
    writers: dict[str, str] = {}
    for name, part in parts.items():
        if part.stage not in STAGES:
            raise SignalError(f"part {name!r}: stage {part.stage!r} is not one of {STAGES}")
        for ch in part.outputs:
            if ch in writers:
                raise SignalError(f"channel {ch!r} is written by both {writers[ch]!r} and {name!r}")
            writers[ch] = name
    for name, part in parts.items():
        for ch in _reads(part):
            if ch not in writers:
                raise SignalError(f"part {name!r} reads channel {ch!r} that no part writes "
                                  f"(channels: {sorted(writers)})")

    # writer -> reader edges; a stateful writer's output does not depend on this
    # step's input, so its outgoing edges impose no order and cannot close a loop.
    edges: dict[str, list[str]] = {n: [] for n in parts}
    for name, part in parts.items():
        for ch in _reads(part):
            src = writers[ch]
            if not parts[src].stateful and name not in edges[src]:
                edges[src].append(name)
    cycle = _find_stateless_cycle(parts, edges)
    if cycle:
        raise SignalError("signal loop through stateless parts only: " + " -> ".join(cycle)
                          + "; put a stateful part (delay, latch) in the loop")

    index = {n: i for i, n in enumerate(parts)}
    order: list[str] = []
    for stage in STAGES:
        order += _stage_order([n for n in parts if parts[n].stage == stage], edges, index)
    if bus is None:
        bus = SignalBus([parts[n] for n in order], writers)
    else:
        bus.order = [parts[n] for n in order]
        bus.writers = writers
        bus.values = {ch: bus.values.get(ch, 0.0) for ch in writers}
    for name, part in parts.items():
        part.bus = bus
        srcs: list[Any] = []
        for ch in part.inputs:
            if parts[writers[ch]] not in srcs:
                srcs.append(parts[writers[ch]])
        part.sources = srcs
    return bus
