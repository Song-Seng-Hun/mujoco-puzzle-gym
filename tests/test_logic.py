"""R7 safe expression gates, R8 transceivers, and the check order stage through
logic parts (a door opened via a gate).

Time is counted in body steps of `bus.dt` = 0.02 s.
"""

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check_order, check_solver
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.expr import ExprError, compile_expr
from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.parts.logic import Gate, Receiver, Transmitter
from mujoco_puzzle_gym.registry import register_logic, register_part
from mujoco_puzzle_gym.room import Room, RoomError
from mujoco_puzzle_gym.signals import SignalError, wire

ROOM = "examples/switch_door.yaml"
DT = 0.02


class Src(Part):
    """Input double: writes `value` to its channel each step."""

    stage, emits, has_body = "input", True, False
    value = 0.0

    def update(self, model, data, body):
        self.write(self.value)


register_part("t_src", Src, replace=True)


def rig(srcs, **parts):
    """Sources named in `srcs` plus parts {name: (cls, cfg)}; returns (parts, bus)."""
    out = {n: Src(n, {"pos": [0, 0]}) for n in srcs}
    out.update({n: cls(n, cfg) for n, (cls, cfg) in parts.items()})
    return out, wire(out)


def run(parts, bus, out, inputs):
    """Step once per entry of `inputs` ({src: value}); return the `out` channel trace."""
    trace = []
    for frame in inputs:
        for n, v in frame.items():
            parts[n].value = v
        bus.step(None, None, None)
        trace.append(bus.get(out))
    return trace


def gate(expr, *srcs):
    parts, bus = rig(srcs, g=(Gate, {"expr": expr}))
    return parts, bus


# ---- expressions: boolean logic and comparisons ---------------------------------------
@pytest.mark.parametrize("expr, a, b, want", [
    ("a and b", 1, 1, 1), ("a and b", 1, 0, 0), ("a or b", 0, 1, 1), ("not a", 0, 0, 1),
    ("a xor b", 1, 1, 0), ("a xor b", 1, 0, 1), ("a > 0.2 and b <= 3", 0.3, 3, 1),
    ("0 < a < 1", 0.5, 0, 1), ("0 < a < 1", 1, 0, 0), ("a == -2", -2, 0, 1),
    ("a or b xor a", 1, 0, 0),           # xor has `or` precedence, folds left: (a or b) xor a
    ("not a and b", 0, 1, 1),
])
def test_boolean_and_compare(expr, a, b, want):
    parts, bus = gate(expr, "a", "b")
    assert run(parts, bus, "g", [{"a": a, "b": b}]) == [want]


def test_dotted_channel_names():
    parts, bus = rig(["laps.count"], g=(Gate, {"expr": "laps.count >= 3"}))
    assert parts["g"].inputs == ["laps.count"]
    assert run(parts, bus, "g", [{"laps.count": 2}, {"laps.count": 3}]) == [0, 1]


# ---- expressions: time functions --------------------------------------------------------
def test_delay():
    parts, bus = gate("delay(a, 0.1)", "a")          # 5 steps
    trace = run(parts, bus, "g", [{"a": 1}] * 8 + [{"a": 0}] * 7)
    assert trace.index(1.0) == 5 and trace[5:13] == [1.0] * 8 and trace[13:] == [0.0, 0.0]


def test_hold():
    parts, bus = gate("hold(a, 0.06)", "a")          # 3 steps after a goes off
    trace = run(parts, bus, "g", [{"a": 1}] * 2 + [{"a": 0}] * 5)
    assert trace == [1, 1, 1, 1, 1, 0, 0]


def test_pulse():
    parts, bus = gate("pulse(a)", "a")
    trace = run(parts, bus, "g", [{"a": 1}] * 3 + [{"a": 0}] + [{"a": 1}])
    assert trace == [1, 0, 0, 0, 1]


def test_latch_reset_wins():
    parts, bus = gate("latch(a, b)", "a", "b")
    frames = [{"a": 1, "b": 0}, {"a": 0, "b": 0}, {"a": 1, "b": 1}, {"a": 0, "b": 0}]
    assert run(parts, bus, "g", frames) == [1, 1, 0, 0]


def test_count_rising_edges_only():
    parts, bus = gate("count(a)", "a")
    trace = run(parts, bus, "g", [{"a": 1}, {"a": 1}, {"a": 0}, {"a": 1}, {"a": 0}, {"a": 1}])
    assert trace == [1, 1, 1, 2, 2, 3]


def test_timer():
    parts, bus = gate("timer(a) > 0.09", "a")        # on after 5 steps of a
    trace = run(parts, bus, "g", [{"a": 1}] * 6 + [{"a": 0}] + [{"a": 1}] * 2)
    assert trace == [0, 0, 0, 0, 1, 1, 0, 0, 0]


def test_functions_compose_and_snapshot():
    parts, bus = gate("latch(pulse(a), b) and not delay(b, 0.04)", "a", "b")
    g = parts["g"]
    assert g.stateful
    run(parts, bus, "g", [{"a": 1, "b": 0}])
    saved = g.save_state()
    after = run(parts, bus, "g", [{"a": 0, "b": 1}] * 3)
    g.load_state(None, None, saved)
    assert run(parts, bus, "g", [{"a": 0, "b": 1}] * 3) == after


# ---- expressions: rejected at build ------------------------------------------------------
BAD = [
    ("__import__('os')", "call to '__import__'"),
    ("a.__class__", "starts with '_'"),
    ("pulse(a).real", "attribute access"),
    ("(lambda: 1)()", "Lambda"),
    ("[x for x in a]", "ListComp"),
    ("{x: 1 for x in a}", "DictComp"),
    ("a[0]", "Subscript"),
    ("a + b", "BinOp"),
    ("'text'", "not a number"),
    ("open('f')", "call to 'open'"),
    ("os.system('ls')", "call to 'os.system'"),
    ("delay(a, b)", "must be a number of seconds"),
    ("delay(a)", "takes 2 argument"),
    ("delay", "is a function"),
    ("a if b else c", "IfExp"),
    ("a and", "cannot parse"),
]


@pytest.mark.parametrize("text, needle", BAD)
def test_rejected_expressions(text, needle):
    with pytest.raises(ExprError, match=needle.replace("(", r"\(").replace(".", r"\.")):
        compile_expr(text)


@pytest.mark.parametrize("text", ["__import__('os')", "a.__class__", "(lambda: 1)()",
                                  "[x for x in a]"])
def test_rejected_at_room_build(repo_root, tmp_path, text):
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    raw["parts"].append({"name": "g", "type": "gate", "expr": text})
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(RoomError, match="part 'g'"):
        Room.load(path)


def test_unknown_channel_in_expr_rejected():
    with pytest.raises(SignalError, match="reads channel 'nope'"):
        rig(["a"], g=(Gate, {"expr": "a and nope"}))


def test_gate_needs_exactly_one_mode():
    with pytest.raises(ValueError, match="exactly one of"):
        Gate("g", {"expr": "a", "op": "and", "in": ["a"]})
    with pytest.raises(ValueError, match="exactly one of"):
        Gate("g", {})


# ---- preset forms: op over `in` ------------------------------------------------------------
@pytest.mark.parametrize("op, frames, want", [
    ("and", [{"a": 1, "b": 1}], [1]), ("or", [{"a": 0, "b": 1}], [1]),
    ("xor", [{"a": 1, "b": 1}], [0]),
])
def test_op_two_inputs(op, frames, want):
    parts, bus = rig(["a", "b"], g=(Gate, {"op": op, "in": ["a", "b"]}))
    assert run(parts, bus, "g", frames) == want


def test_op_not_delay_timer():
    parts, bus = rig(["a"], n=(Gate, {"op": "not", "in": ["a"]}),
                     d=(Gate, {"op": "delay", "in": ["a"], "seconds": 0.04}),
                     t=(Gate, {"op": "timer", "in": ["a"]}))
    run(parts, bus, "n", [{"a": 1}] * 3)
    assert bus.get("n") == 0 and bus.get("d") == 1 and bus.get("t") == pytest.approx(3 * DT)
    assert parts["d"].stateful and not parts["n"].stateful


def test_op_arity_checked():
    with pytest.raises(ValueError, match="op not needs 1"):
        Gate("g", {"op": "not", "in": ["a", "b"]})


# ---- sequence lock ---------------------------------------------------------------------------
def seq_rig(**extra):
    return rig(["a", "b", "c"], s=(Gate, {"sequence": ["a", "b", "c"], **extra}))


def presses(*names):
    """Each name: one step pressed, one step released."""
    frames = []
    for n in names:
        frames += [{n: 1}, {n: 0}]
    return frames


def test_sequence_right_order_pulses_once():
    parts, bus = seq_rig()
    trace = run(parts, bus, "s", presses("a", "b", "c") + [{}] * 3)
    assert trace[4] == 1 and sum(trace) == 1            # one step on, then waits again
    trace = run(parts, bus, "s", presses("a", "b", "c"))
    assert sum(trace) == 1                              # works again


def test_sequence_wrong_order_pulses_wrong():
    parts, bus = seq_rig()
    out, wrong = [], []
    for frame in presses("a", "c", "b", "c"):
        run(parts, bus, "s", [frame])
        out.append(bus.get("s"))
        wrong.append(bus.get("s.wrong"))
    assert sum(out) == 0
    assert wrong == [0, 0, 1, 0, 1, 0, 1, 0]            # c out of order; then b, c without a


def test_sequence_restart_with_first_after_wrong():
    parts, bus = seq_rig()
    trace = run(parts, bus, "s", presses("a", "a", "b", "c"))
    assert sum(trace) == 1                              # the second a starts a new attempt


def test_sequence_latch():
    parts, bus = seq_rig(latch=True)
    trace = run(parts, bus, "s", presses("a", "b", "c") + presses("b"))
    assert trace[4:] == [1, 1, 1, 1]


# ---- python logic plugins --------------------------------------------------------------------
@register_logic("t_majority", replace=True)
def _majority(inputs, state, dt):
    state["calls"] = state.get("calls", 0) + 1
    return float(sum(v > 0.5 for v in inputs.values()) * 2 > len(inputs))


def test_registered_logic():
    parts, bus = rig(["a", "b", "c"], g=(Gate, {"logic": "t_majority", "in": ["a", "b", "c"]}))
    assert run(parts, bus, "g", [{"a": 1}, {"b": 1}]) == [0, 1]
    assert parts["g"].plugin_state["calls"] == 2


def test_unregistered_logic_rejected():
    with pytest.raises(ValueError, match="unknown logic 'nope'"):
        Gate("g", {"logic": "nope", "in": ["a"]})


# ---- loops -----------------------------------------------------------------------------------
def test_not_not_loop_rejected():
    with pytest.raises(SignalError, match="stateless"):
        rig([], n1=(Gate, {"op": "not", "in": ["n2"]}), n2=(Gate, {"op": "not", "in": ["n1"]}))


def test_loop_through_delay_gate_runs():
    parts, bus = rig([], n=(Gate, {"expr": "not d"}), d=(Gate, {"expr": "delay(n, 0.02)"}))
    assert run(parts, bus, "n", [{}] * 6) == [1, 1, 0, 0, 1, 1]


# ---- transceivers (R8) -------------------------------------------------------------------------
@pytest.mark.parametrize("tx_delay, rx_delay", [(0.3, 0.0), (0.0, 0.3), (0.1, 0.2)])
def test_transceiver_delay(tx_delay, rx_delay):
    parts, bus = rig(["a"], tx=(Transmitter, {"pos": [0, 0], "in": ["a"], "band": "blue",
                                              "delay": tx_delay}),
                     rx=(Receiver, {"pos": [1, 0], "band": "blue", "delay": rx_delay}))
    trace = run(parts, bus, "rx", [{"a": 1}] * 30)
    first = trace.index(1.0)                            # steps after the input turned on
    assert abs(first - round(0.3 / DT)) <= 1


def test_transceiver_same_step_without_delay_and_fan_out():
    parts, bus = rig(["a"], r1=(Receiver, {"pos": [1, 0], "band": "x"}),
                     tx=(Transmitter, {"pos": [0, 0], "in": ["a"], "band": "x"}),
                     r2=(Receiver, {"pos": [2, 0], "band": "x"}))
    run(parts, bus, "r1", [{"a": 1}])
    assert bus.get("r1") == 1 and bus.get("r2") == 1


def test_band_errors():
    with pytest.raises(SignalError, match="'band:x' that no part writes"):
        rig([], rx=(Receiver, {"pos": [0, 0], "band": "x"}))
    with pytest.raises(SignalError, match="written by both"):
        rig(["a"], t1=(Transmitter, {"pos": [0, 0], "in": ["a"], "band": "x"}),
            t2=(Transmitter, {"pos": [0, 0], "in": ["a"], "band": "x"}))


# ---- check order stage through logic parts ---------------------------------------------------
def gate_room(repo_root, tmp_path, extra_parts, door_in, second_switch=(-0.25, -0.5)):
    """switch_door with a second switch and the door fed through logic parts."""
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    raw["links"] = []
    raw["parts"].insert(1, {"name": "switch_c", "type": "switch", "pos": list(second_switch),
                            "size": [0.3, 0.3]})
    raw["parts"] += extra_parts
    raw["parts"][2]["in"] = door_in                     # door_b
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


ROUTES = {
    "gate": ([{"name": "both", "type": "gate", "expr": "switch_a and delay(switch_c, 0.5)"}],
             ["both"]),
    "counter": ([{"name": "c", "type": "counter", "up": "switch_a", "target": 1},
                 {"name": "g", "type": "gate", "op": "and", "in": ["c.reached", "switch_c"]}],
                ["g"]),
    "radio": ([{"name": "tx", "type": "transmitter", "pos": [-1.0, 0.8], "in": ["n"], "band": "b"},
               {"name": "n", "type": "gate", "expr": "switch_a and switch_c"},
               {"name": "rx", "type": "receiver", "pos": [0.25, 0.4], "band": "b", "delay": 0.2}],
              ["rx"]),
}


@pytest.mark.parametrize("route", sorted(ROUTES))
def test_order_through_logic(asset_dir, repo_root, tmp_path, route):
    extra, door_in = ROUTES[route]
    env = PuzzleEnv(room=gate_room(repo_root, tmp_path, extra, door_in))
    res = check_order(env)
    assert res["ok"], res
    assert res["order"] == ["switch_a", "switch_c", "door_b", "goal"]
    env.close()


def test_order_through_gate_fails_when_input_unreachable(asset_dir, repo_root, tmp_path):
    # switch_c sits behind the door it has to open.
    extra, door_in = ROUTES["gate"]
    env = PuzzleEnv(room=gate_room(repo_root, tmp_path, extra, door_in, second_switch=(0.85, 0.6)))
    res = check_order(env)
    assert not res["ok"]
    assert "door_b" in res["reason"] and "unreachable: ['switch_c']" in res["reason"]
    env.close()


def test_order_through_not_gate(asset_dir, repo_root, tmp_path):
    # Door open while switch_c is off: the goal is reachable with no switch at all.
    env = PuzzleEnv(room=gate_room(repo_root, tmp_path,
                                   [{"name": "inv", "type": "gate", "op": "not", "in": ["switch_c"]}],
                                   ["inv"]))
    res = check_order(env)
    assert res["ok"] and res["order"] == ["goal"]
    env.close()


def test_door_opens_through_gate_in_one_body_step(asset_dir, repo_root, tmp_path):
    extra = [{"name": "g", "type": "gate", "expr": "switch_a and not switch_c"}]
    env = PuzzleEnv(room=gate_room(repo_root, tmp_path, extra, ["g"]))
    lay = env.sample_layout(None)
    lay["spawn"] = (-0.25, 0.5, 0.0)                    # on switch_a
    env.reset(options={"layout": lay})
    door = env.parts["door_b"]
    env.signals.step(env.model, env.data, env.body.state(env.data))
    assert env.signals.get("g") == 1.0 and env.data.ctrl[door._act] == pytest.approx(door.travel)
    env.close()


@pytest.mark.slow
def test_gate_room_passes_solver(asset_dir, repo_root, tmp_path):
    extra, door_in = ROUTES["gate"]
    env = PuzzleEnv(room=gate_room(repo_root, tmp_path, extra, door_in))
    res = check_solver(env, seeds=20, need=18)
    assert res["ok"], res
    env.close()
