"""M1 `switch` (R2): an alias of the floor button (`parts/input/button.py`).

`latch: true` (default) is the button kind `oneshot` (on from the first step
on it, as in M1); `latch: false` is `momentary`. Only the walking body presses
it (`trigger: body`). The plate is on while the body centre is over it; with
`enable` off it keeps its state (R1). Other button values (`kind`, `trigger`)
may be given too; `kind` then wins over `latch`.
"""

from __future__ import annotations

from ..schema import Field
from .input.button import Button


class Switch(Button):
    fields = (
        Field("latch", "bool", True, doc="M1: stay on after the body leaves (oneshot); "
                                         "false: momentary"),
    )

    def __init__(self, name, cfg):
        cfg = dict(cfg)
        latch = cfg.get("latch", True)
        if "kind" not in cfg and isinstance(latch, bool):
            cfg["kind"] = "oneshot" if latch else "momentary"
        cfg.setdefault("trigger", "body")
        super().__init__(name, cfg)
        self.latch = self.cfg["latch"]
