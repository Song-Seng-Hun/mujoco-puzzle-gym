"""Transceivers (R8): a transmitter sends its `in` channel on a `band`; every
receiver on that band writes the value to its own channel (its name).

Both stand in the room as small posts (no collision) so a picture or a guide
can show where a signal goes. `delay` (seconds, on either end; they add up)
holds the value back by that many body steps. A band is the channel
`band:<band>`; two transmitters on one band are rejected like two writers of
one channel.
"""

from __future__ import annotations

from collections import deque

import mujoco

from ...schema import Field, SchemaError
from ..base import Part

BAND_PREFIX = "band:"
TX_RGBA = (0.85, 0.3, 0.85, 1.0)
RX_RGBA = (0.4, 0.3, 0.9, 1.0)


class _Delayed(Part):
    stage = "logic"
    fields = (
        Field("band", "str", doc="band name shared by a transmitter and its receivers"),
        Field("delay", "float", 0.0, unit="s", min=0.0, max=3600.0, doc="hold the value back"),
    )
    rgba = TX_RGBA

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.band = self.cfg["band"]
        self.delay = float(self.cfg["delay"])
        self.stateful = self.delay > 0
        self.buf: deque[float] | None = None
        self.value = 0.0

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/post", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[0.02, 0.06], pos=[0, 0, 0.06], rgba=self.rgba,
                      contype=0, conaffinity=0)
        return body

    def reset(self, model, data):
        self.buf = None
        self.value = 0.0

    def _pass(self, v: float) -> float:
        if self.delay <= 0:
            return v
        if self.buf is None:
            self.buf = deque([0.0] * max(1, int(round(self.delay / self.bus.dt))))
        out = self.buf.popleft()
        self.buf.append(v)
        return out

    def update(self, model, data, body):
        if not self.enabled():
            return
        self.value = self._pass(self.read(self.inputs[0]))
        self.write(self.value)

    def state(self):
        return float(self.value > 0.5)

    def plan_values(self, on, get):
        return {self.outputs[0]: get(self.inputs[0])}

    def save_state(self):
        return {"value": self.value, "buf": None if self.buf is None else list(self.buf)}

    def load_state(self, model, data, st):
        self.value = st["value"]
        self.buf = None if st["buf"] is None else deque(st["buf"])


class Transmitter(_Delayed):
    rgba = TX_RGBA

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        if len(self.inputs) != 1:
            raise SchemaError(f"part {name!r}: a transmitter needs exactly one channel in `in`, "
                              f"got {self.inputs}")
        self.outputs = [BAND_PREFIX + self.band]


class Receiver(_Delayed):
    rgba = RX_RGBA
    emits = True

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        if self.inputs:
            raise SchemaError(f"part {name!r}: a receiver reads its band, not `in`")
        self.inputs = [BAND_PREFIX + self.band]
