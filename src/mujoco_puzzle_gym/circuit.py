"""Circuit graph behind the break checks (R34, `mpg check` stage `circuit`).

Breaking a part changes what other parts read, in one of two ways:

- a breakable part freezes its own value outputs at `broken_value`, so every
  part reading one of its channels is affected;
- a `wire` keeps the channel it carries but takes it away from the readers in
  its `to` (`Part.cut`), so only those parts are affected.

Both are edges from a part to the parts whose reading changes, so "what does
breaking this reach" is a walk over those edges (`affected`). `lost_channels`
names the channels that stop carrying their own value along the way: the
"channels that go off when it breaks" of the spec.

`check_circuit(env, order)` is the static stage. It fails on a `demo:` entry
that breaks a part the room cannot afford to lose (`on_break: fail`) or that
reaches no other part at all, and warns when a part the answer leans on can be
broken with no way to repair it: `shuts_gates` plays the break through the plan
and sees whether a door the answer walks through closes. The per-part sets are
in the result, so a room author can read what the graph thinks.
"""

from __future__ import annotations

from typing import Any

from .logic import gate_open, plan_channels
from .signals import _reads


def demo_breaks(room) -> frozenset[str]:
    """Parts the room's `demo:` breaks (R34). The plan (logic.py `solution_order`) takes
    these as broken, since a room may well be solvable only once something is cut."""
    return frozenset(e[1] for e in (getattr(room, "demo", None) or []) if e[0] == "break")


def readers_of(parts: dict[str, Any]) -> dict[str, list[str]]:
    """Channel -> names of the parts that read it (`in`, `enable`, `watch`)."""
    out: dict[str, list[str]] = {}
    for name, part in parts.items():
        for ch in _reads(part):
            out.setdefault(ch, []).append(name)
    return out


def break_edges(parts: dict[str, Any]) -> dict[str, list[str]]:
    """Part -> the parts whose reading changes when it breaks (see the module docstring)."""
    from .parts.logic.wire import Wire

    readers = readers_of(parts)
    out: dict[str, list[str]] = {}
    for name, part in parts.items():
        if isinstance(part, Wire):
            names = [p for p, _ in part.targets]                  # a wire: only its `to` readers
        else:
            names = [r for ch in part.outputs for r in readers.get(ch, [])]
        out[name] = sorted({n for n in names if n != name})
    return out


def affected(parts: dict[str, Any], name: str) -> list[str]:
    """Parts reached when `name` breaks, in name order (`name` itself is not in it)."""
    edges = break_edges(parts)
    seen: set[str] = set()
    stack = list(edges.get(name, []))
    while stack:
        n = stack.pop()
        if n in seen or n == name:
            continue
        seen.add(n)
        stack += edges.get(n, [])
    return sorted(seen)


def lost_channels(parts: dict[str, Any], name: str) -> list[str]:
    """Channels that stop carrying their own value when `name` breaks: the value
    outputs of the broken part and of every part it reaches."""
    out: set[str] = set()
    for n in [name, *affected(parts, name)]:
        part = parts[n]
        vals = part.value_outputs() if hasattr(part, "value_outputs") else list(part.outputs)
        out.update(vals)
    return sorted(out)


def breakables(parts: dict[str, Any]) -> list[str]:
    return [n for n, p in parts.items() if getattr(p, "breakable", False)]


def shuts_gates(parts: dict[str, Any], name: str, on: set[str]) -> list[str]:
    """Gates that are open in the plan but closed once `name` breaks: the doors the
    answer walks through and a break would take away."""
    whole = plan_channels(parts, on)
    cut = plan_channels(parts, on, {name})
    return sorted(p.name for p in parts.values()
                  if p.role == "gate" and gate_open(p, whole) and not gate_open(p, cut))


def check_circuit(env, order: list[str]) -> dict[str, Any]:
    """Static: the break graph against the room's `demo:` and solution order (R34)."""
    parts = env.parts
    names = breakables(parts)
    demo = list(getattr(env.room, "demo", []) or [])
    broken_in_demo = {e[1] for e in demo if e[0] == "break"}
    graph = {n: {"affected": affected(parts, n), "channels": lost_channels(parts, n)} for n in names}
    why: list[str] = []
    warn: list[str] = []
    for name in sorted(broken_in_demo):
        part = parts[name]
        if part.on_break == "fail":
            why.append(f"`demo:` breaks {name!r}, which has `on_break: fail`: breaking it ends "
                       f"the episode as a failure")
        if not graph[name]["affected"]:
            why.append(f"`demo:` breaks {name!r} but no part reads anything it writes, so "
                       f"breaking it changes nothing")
    on = {n for n in order if parts[n].role == "switch"}
    for name in sorted(names):
        part = parts[name]
        if name in broken_in_demo or part.has_repair():
            continue
        shuts = shuts_gates(parts, name, on)
        if shuts:
            warn.append(f"breaking {name!r} (`by: {part.by}`) shuts {shuts}, which the answer "
                        f"walks through, and there is no way to repair it (`repair` or "
                        f"`repair_with`)")
    out: dict[str, Any] = {"ok": not why, "breakable": sorted(names), "graph": graph}
    if why:
        out["reason"] = "; ".join(why)
    if warn:
        out["warnings"] = warn
    return out
