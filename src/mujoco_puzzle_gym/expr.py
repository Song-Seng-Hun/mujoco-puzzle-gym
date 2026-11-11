"""Safe signal expressions for gates (R7).

An expression reads channels and numbers and gives a float:

    a and not b
    (lever.angle > 0.5) xor door_open
    delay(btn, 1.5)
    latch(pulse(a), reset_btn)
    count(lap) >= 3 or timer(plate) > 2

Allowed: channel names (dotted names such as `laps.count` are one channel),
numbers, `and or not xor`, comparisons (`< <= > >= == !=`, chains too), unary
`-`/`+`, parentheses and the functions below. Everything else (attribute access
on anything but a channel name, calls to other names, `__import__`, lambdas,
comprehensions, subscripts, strings, arithmetic) is rejected with `ExprError`
when the room is built. Nothing is passed to `eval`/`exec`: the text is parsed
with `ast`, checked against the allow-list, and computed by this module.

Values: on/off results are 1.0 / 0.0; a value above 0.5 reads as on (R1).
`xor` has the precedence of `or`; mixed `or`/`xor` chains fold left to right.

Functions (each call keeps its own state; `s` must be a number literal):
- `delay(x, s)`  x as it was s seconds ago (at least one body step).
- `hold(x, s)`   1 while x is on and for s seconds after it goes off.
- `pulse(x)`     1 on the step x turns on, else 0.
- `latch(set, reset)`  1 after set turns on until reset is on (reset wins).
- `count(x)`     number of times x has turned on.
- `timer(x)`     seconds x has been on without a break (0 while off).
An expression using any of them is stateful: its gate may sit in a channel loop.
"""

from __future__ import annotations

import ast
import io
import math
import operator
import tokenize
from collections import deque
from typing import Any, Callable

ON = 0.5
MAX_LEN = 1000
MAX_SECONDS = 3600.0

Getter = Callable[[str], float]


class ExprError(ValueError):
    """An expression that is not allowed or malformed; the message says why."""


def _on(v: float) -> bool:
    return v > ON


def _b(flag: bool) -> float:
    return 1.0 if flag else 0.0


# ---- stateful functions --------------------------------------------------------------
class _Fn:
    n_args = 1
    takes_seconds = False

    def __init__(self, seconds: float = 0.0):
        self.seconds = seconds
        self.reset()

    def reset(self) -> None: ...

    def get_state(self) -> Any: ...

    def set_state(self, st: Any) -> None: ...

    @staticmethod
    def steps(seconds: float, dt: float) -> int:
        return int(round(seconds / dt))


class _Delay(_Fn):
    n_args, takes_seconds = 2, True

    def reset(self):
        self.buf: deque[float] | None = None

    def __call__(self, args, dt):
        if self.buf is None:
            self.buf = deque([0.0] * max(1, self.steps(self.seconds, dt)))
        out = self.buf.popleft()
        self.buf.append(args[0])
        return out

    def get_state(self):
        return None if self.buf is None else list(self.buf)

    def set_state(self, st):
        self.buf = None if st is None else deque(st)


class _Hold(_Fn):
    n_args, takes_seconds = 2, True

    def reset(self):
        self.left = 0

    def __call__(self, args, dt):
        if _on(args[0]):
            self.left = self.steps(self.seconds, dt)
            return 1.0
        if self.left > 0:
            self.left -= 1
            return 1.0
        return 0.0

    def get_state(self):
        return self.left

    def set_state(self, st):
        self.left = st


class _Edge(_Fn):
    """Base for functions that watch the rising edge of their first argument."""

    def reset(self):
        self.prev = False
        self.value = 0.0

    def rising(self, x: float) -> bool:
        now = _on(x)
        up = now and not self.prev
        self.prev = now
        return up

    def get_state(self):
        return (self.prev, self.value)

    def set_state(self, st):
        self.prev, self.value = st


class _Pulse(_Edge):
    def __call__(self, args, dt):
        return _b(self.rising(args[0]))


class _Count(_Edge):
    def __call__(self, args, dt):
        if self.rising(args[0]):
            self.value += 1.0
        return self.value


class _Latch(_Fn):
    n_args = 2

    def reset(self):
        self.q = False

    def __call__(self, args, dt):
        if _on(args[1]):
            self.q = False
        elif _on(args[0]):
            self.q = True
        return _b(self.q)

    def get_state(self):
        return self.q

    def set_state(self, st):
        self.q = st


class _Timer(_Fn):
    def reset(self):
        self.t = 0.0

    def __call__(self, args, dt):
        self.t = self.t + dt if _on(args[0]) else 0.0
        return self.t

    def get_state(self):
        return self.t

    def set_state(self, st):
        self.t = st


FUNCTIONS: dict[str, type[_Fn]] = {
    "delay": _Delay, "hold": _Hold, "pulse": _Pulse,
    "latch": _Latch, "count": _Count, "timer": _Timer,
}

# ---- nodes ------------------------------------------------------------------------------
_CMP = {ast.Lt: operator.lt, ast.LtE: operator.le, ast.Gt: operator.gt, ast.GtE: operator.ge,
        ast.Eq: operator.eq, ast.NotEq: operator.ne}


class _Node:
    def eval(self, get: Getter, dt: float) -> float: ...

    def plan(self, get: Getter) -> float:
        """Value once inputs settle, time functions taken as 'eventually' (planning)."""
        return self.eval(get, 0.0)


class _Const(_Node):
    def __init__(self, v):
        self.v = float(v)

    def eval(self, get, dt):
        return self.v


class _Chan(_Node):
    def __init__(self, name):
        self.name = name

    def eval(self, get, dt):
        return float(get(self.name))


class _Not(_Node):
    def __init__(self, a):
        self.a = a

    def eval(self, get, dt):
        return _b(not _on(self.a.eval(get, dt)))

    def plan(self, get):
        return _b(not _on(self.a.plan(get)))


class _Neg(_Node):
    def __init__(self, a, sign):
        self.a, self.sign = a, sign

    def eval(self, get, dt):
        return self.sign * self.a.eval(get, dt)

    def plan(self, get):
        return self.sign * self.a.plan(get)


class _Chain(_Node):
    """and / or / xor over values; ops[i] joins values[i] and values[i + 1]."""

    def __init__(self, values, ops):
        self.values, self.ops = values, ops

    def _fold(self, vals: list[float]) -> float:
        acc = _on(vals[0])
        for op, v in zip(self.ops, vals[1:]):
            b = _on(v)
            acc = (acc and b) if op == "and" else (acc or b) if op == "or" else (acc != b)
        return _b(acc)

    def eval(self, get, dt):
        # Every operand is evaluated each step (no short-circuit) so the state
        # of functions inside stays in step with time.
        return self._fold([v.eval(get, dt) for v in self.values])

    def plan(self, get):
        return self._fold([v.plan(get) for v in self.values])


class _Compare(_Node):
    def __init__(self, left, ops, rights):
        self.left, self.ops, self.rights = left, ops, rights

    def _cmp(self, vals):
        return _b(all(op(a, b) for op, a, b in zip(self.ops, vals, vals[1:])))

    def eval(self, get, dt):
        return self._cmp([self.left.eval(get, dt)] + [r.eval(get, dt) for r in self.rights])

    def plan(self, get):
        return self._cmp([self.left.plan(get)] + [r.plan(get) for r in self.rights])


class _Call(_Node):
    def __init__(self, name, fn: _Fn, args):
        self.name, self.fn, self.args = name, fn, args

    def eval(self, get, dt):
        return float(self.fn([a.eval(get, dt) for a in self.args], dt))

    def plan(self, get):
        x = [a.plan(get) for a in self.args]
        # Eventually: a delayed / held / pulsed signal reaches its input's value;
        # a count or timer of an input that can stay on grows without bound.
        if self.name in ("delay", "hold", "pulse"):
            return _b(_on(x[0]))
        if self.name == "latch":
            return _b(_on(x[0]) and not _on(x[1]))
        return math.inf if _on(x[0]) else 0.0


# ---- parsing ----------------------------------------------------------------------------
def _xor_positions(text: str) -> tuple[str, set[tuple[int, int]]]:
    """Replace the word `xor` by ` or` (same length) and return where it stood."""
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(text).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError) as exc:
        raise ExprError(f"cannot read expression {text!r}: {exc}") from None
    lines = text.split("\n")
    where: set[tuple[int, int]] = set()
    for t in toks:
        if t.type == tokenize.NAME and t.string == "xor":
            (r, c) = t.start
            lines[r - 1] = lines[r - 1][:c] + " or" + lines[r - 1][c + 3:]
            where.add((r, c))
    return "\n".join(lines), where


def _dotted(node: ast.AST) -> str | None:
    """`a.b.c` built only from names -> 'a.b.c'; anything else -> None."""
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if not isinstance(node, ast.Name):
        return None
    parts.append(node.id)
    return ".".join(reversed(parts))


class Expr:
    """A checked expression. `channels` lists the channels it reads (first use order)."""

    def __init__(self, text: str):
        if not isinstance(text, str) or not text.strip():
            raise ExprError("expression must be a non-empty string")
        if len(text) > MAX_LEN:
            raise ExprError(f"expression longer than {MAX_LEN} characters")
        self.text = text
        src, self._xors = _xor_positions(text.strip())
        try:
            tree = ast.parse(src, mode="eval")
        except SyntaxError as exc:
            raise ExprError(f"cannot parse expression {text!r}: {exc.msg}") from None
        self.channels: list[str] = []
        self.calls: list[_Call] = []
        self.root = self._build(tree.body)

    @property
    def stateful(self) -> bool:
        return bool(self.calls)

    def _reject(self, node: ast.AST, why: str = "") -> ExprError:
        what = type(node).__name__
        return ExprError(f"expression {self.text!r}: {why or what + ' is not allowed'}; allowed are "
                         "channel names, numbers, and/or/not/xor, comparisons and "
                         + ", ".join(f"{f}()" for f in FUNCTIONS))

    def _channel(self, node: ast.AST) -> _Chan:
        name = _dotted(node)
        if name is None:
            raise self._reject(node, "attribute access is only allowed inside a channel name")
        if any(p.startswith("_") for p in name.split(".")):
            raise self._reject(node, f"name {name!r} is not allowed (starts with '_')")
        if name in FUNCTIONS:
            raise self._reject(node, f"{name!r} is a function; call it as {name}(...)")
        if name not in self.channels:
            self.channels.append(name)
        return _Chan(name)

    def _seconds(self, node: ast.AST, fname: str) -> float:
        sign = 1.0
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            sign = -1.0 if isinstance(node.op, ast.USub) else 1.0
            node = node.operand
        if not (isinstance(node, ast.Constant) and type(node.value) in (int, float)):
            raise self._reject(node, f"{fname}(x, s): s must be a number of seconds")
        s = sign * float(node.value)
        if not 0.0 <= s <= MAX_SECONDS:
            raise self._reject(node, f"{fname}(x, s): s = {s:g} is out of range [0, {MAX_SECONDS:g}]")
        return s

    def _build(self, node: ast.AST) -> _Node:
        if isinstance(node, ast.Constant):
            if type(node.value) in (int, float, bool):
                return _Const(node.value)
            raise self._reject(node, f"constant {node.value!r} is not a number")
        if isinstance(node, (ast.Name, ast.Attribute)):
            return self._channel(node)
        if isinstance(node, ast.UnaryOp):
            if isinstance(node.op, ast.Not):
                return _Not(self._build(node.operand))
            if isinstance(node.op, (ast.USub, ast.UAdd)):
                return _Neg(self._build(node.operand), -1.0 if isinstance(node.op, ast.USub) else 1.0)
            raise self._reject(node.op)
        if isinstance(node, ast.BoolOp):
            values = [self._build(v) for v in node.values]
            if isinstance(node.op, ast.And):
                ops = ["and"] * (len(values) - 1)
            else:
                ops = []
                for a, b in zip(node.values, node.values[1:]):
                    between = {p for p in self._xors
                               if (a.end_lineno, a.end_col_offset) <= p < (b.lineno, b.col_offset)}
                    ops.append("xor" if between else "or")
            return _Chain(values, ops)
        if isinstance(node, ast.Compare):
            ops = []
            for op in node.ops:
                if type(op) not in _CMP:
                    raise self._reject(op)
                ops.append(_CMP[type(op)])
            return _Compare(self._build(node.left), ops, [self._build(c) for c in node.comparators])
        if isinstance(node, ast.Call):
            fname = node.func.id if isinstance(node.func, ast.Name) else None
            if fname not in FUNCTIONS:
                shown = fname or (_dotted(node.func) or type(node.func).__name__)
                raise self._reject(node, f"call to {shown!r} is not allowed")
            if node.keywords:
                raise self._reject(node, f"{fname}() takes no keyword arguments")
            cls = FUNCTIONS[fname]
            if len(node.args) != cls.n_args:
                raise self._reject(node, f"{fname}() takes {cls.n_args} argument(s)")
            if cls.takes_seconds:
                fn = cls(self._seconds(node.args[1], fname))
                args = [self._build(node.args[0])]
            else:
                fn = cls()
                args = [self._build(a) for a in node.args]
            call = _Call(fname, fn, args)
            self.calls.append(call)
            return call
        raise self._reject(node)

    # ---- running ------------------------------------------------------------------
    def reset(self) -> None:
        for c in self.calls:
            c.fn.reset()

    def eval(self, get: Getter, dt: float) -> float:
        return self.root.eval(get, dt)

    def plan(self, get: Getter) -> float:
        return self.root.plan(get)

    def get_state(self) -> list[Any]:
        return [c.fn.get_state() for c in self.calls]

    def set_state(self, st: list[Any]) -> None:
        for c, s in zip(self.calls, st):
            c.fn.set_state(s)


def compile_expr(text: str) -> Expr:
    return Expr(text)


OPS = ("and", "or", "xor", "not", "delay", "hold", "pulse", "timer", "count", "latch")
_ONE_INPUT = ("not", "delay", "hold", "pulse", "timer", "count")


def op_expr(op: str, channels: list[str], seconds: float = 0.0) -> Expr:
    """The expression for a preset-style gate `op` over `channels` (any channel
    names, not only identifiers): and/or/xor over all inputs, `not` and the time
    functions over one input, `latch` over [set, reset]."""
    if op not in OPS:
        raise ExprError(f"op {op!r} is not one of {OPS}")
    need = 1 if op in _ONE_INPUT else 2 if op == "latch" else None
    if (need and len(channels) != need) or not channels:
        raise ExprError(f"op {op} needs {need or 'one or more'} input channel(s) in `in`, "
                        f"got {channels}")
    e = Expr.__new__(Expr)
    e.text = f"{op}({', '.join(channels)})"
    e._xors = set()
    e.channels = list(dict.fromkeys(channels))
    e.calls = []
    chans = [_Chan(c) for c in channels]
    if op in ("and", "or", "xor"):
        e.root = _Chain(chans, [op] * (len(chans) - 1))
    elif op == "not":
        e.root = _Not(chans[0])
    else:
        cls = FUNCTIONS[op]
        fn = cls(seconds) if cls.takes_seconds else cls()
        e.root = _Call(op, fn, chans)
        e.calls.append(e.root)
    return e
