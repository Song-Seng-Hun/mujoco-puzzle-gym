"""Value schemas for part settings (R3).

Every value a part reads from room.yaml (size, force, speed, mass, time) is a
`Field`: name, unit, default, min/max, and whether it may change at run time.
`Part` collects its class's fields (base classes first) and `resolve` checks a
room entry against them, so an out-of-range value fails `mpg build` /
`mpg check` with the reason and the allowed range:

    part 'door_b': width = 5.0 m is out of range [0.1, 2.0]

Keys not in the schema pass through unchanged (plugins may read extra keys).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Sequence

REQUIRED = object()   # default for fields that have no default

KINDS = ("float", "int", "bool", "str", "enum", "vec", "list", "any")


class SchemaError(ValueError):
    """A value outside its schema; the message names the part, the value and the allowed range."""


@dataclass(frozen=True)
class Field:
    """One setting of a part.

    kind: float | int | bool | str | enum (one of `choices`) | vec (`length`
    numbers, each in [min, max]) | list (of str) | any (structured value the part
    checks itself, e.g. a warp's `to` / `routes`).
    runtime: may be changed while running (R13 setters); size and shape may not.
    episode: may be changed at episode start only (R21 difficulty steps): a value
    a part reads only when an episode starts; run-time values may be changed then too.
    """

    name: str
    kind: str = "float"
    default: Any = REQUIRED
    unit: str = ""
    min: float | None = None
    max: float | None = None
    choices: tuple[Any, ...] = ()
    length: int | None = None
    runtime: bool = False
    episode: bool = False
    doc: str = ""

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"field {self.name!r}: kind must be one of {KINDS}")

    @property
    def required(self) -> bool:
        return self.default is REQUIRED

    def range_text(self) -> str:
        if self.kind == "enum":
            return "one of " + ", ".join(map(repr, self.choices))
        lo = "-inf" if self.min is None else f"{self.min:g}"
        hi = "inf" if self.max is None else f"{self.max:g}"
        return f"[{lo}, {hi}]"

    def describe(self) -> dict[str, Any]:
        """JSON-friendly description (for `mpg parts --json`, R3)."""
        d = asdict(self)
        d["default"] = None if self.required else self.default
        d["required"] = self.required
        d["choices"] = list(self.choices)
        return d

    # ---- checking -----------------------------------------------------------
    def _in_range(self, v: float) -> bool:
        return (self.min is None or v >= self.min) and (self.max is None or v <= self.max)

    def _number(self, owner: str, value: Any, integer: bool = False) -> float | int:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise SchemaError(f"{owner}: {self.name} must be a number, got {value!r}")
        if integer and float(value) != int(value):
            raise SchemaError(f"{owner}: {self.name} must be a whole number, got {value!r}")
        return int(value) if integer else float(value)

    def _out_of_range(self, owner: str, value: Any) -> SchemaError:
        unit = f" {self.unit}" if self.unit else ""
        return SchemaError(f"{owner}: {self.name} = {value!r}{unit} is out of range "
                           f"{self.range_text()}")

    def check(self, owner: str, value: Any) -> Any:
        """Return `value` converted to the field's kind, or raise SchemaError."""
        k = self.kind
        if k in ("float", "int"):
            v = self._number(owner, value, integer=(k == "int"))
            if not self._in_range(v):
                raise self._out_of_range(owner, v)
            return v
        if k == "bool":
            if not isinstance(value, bool):
                raise SchemaError(f"{owner}: {self.name} must be true or false, got {value!r}")
            return value
        if k == "str":
            if not isinstance(value, str) or not value:
                raise SchemaError(f"{owner}: {self.name} must be a non-empty string, got {value!r}")
            return value
        if k == "enum":
            if value not in self.choices:
                raise SchemaError(f"{owner}: {self.name} = {value!r} is not allowed; "
                                  f"{self.range_text()}")
            return value
        if k == "vec":
            if not isinstance(value, (list, tuple)) or (self.length and len(value) != self.length):
                n = self.length or "n"
                raise SchemaError(f"{owner}: {self.name} must be a list of {n} numbers, got {value!r}")
            out = [self._number(owner, v) for v in value]
            if not all(self._in_range(v) for v in out):
                raise self._out_of_range(owner, out)
            return out
        if k == "any":
            return value
        # list of str; a single string is accepted as a one-element list
        if isinstance(value, str):
            value = [value]
        if not isinstance(value, (list, tuple)) or not all(isinstance(v, str) and v for v in value):
            raise SchemaError(f"{owner}: {self.name} must be a list of names, got {value!r}")
        return list(value)


Schema = dict[str, Field]


def make_schema(fields: Sequence[Field]) -> Schema:
    return {f.name: f for f in fields}


def resolve(owner: str, schema: Schema, cfg: dict[str, Any]) -> dict[str, Any]:
    """`cfg` with schema defaults filled in and every schema value checked.

    Missing required values and out-of-range values raise SchemaError; keys
    outside the schema are kept as they are.
    """
    out = dict(cfg)
    for name, f in schema.items():
        if name not in cfg or cfg[name] is None:
            if f.required:
                raise SchemaError(f"{owner}: `{name}` is required ({f.kind}"
                                  f"{', ' + f.unit if f.unit else ''})")
            out[name] = list(f.default) if isinstance(f.default, (list, tuple)) else f.default
            continue
        out[name] = f.check(owner, cfg[name])
    return out
