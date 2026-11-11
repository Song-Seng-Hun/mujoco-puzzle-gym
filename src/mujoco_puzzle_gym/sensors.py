"""Real-sensor observation items (R39): `rangefinder` and `camera`.

Both are opt-in task.yaml `obs:` items; without them the observation is the
privileged M1 vector.

    obs:
      - gravity
      - {item: rangefinder, rays: 9, fov: 120, max_dist: 2.0, noise: 0.01}
      - {item: camera, mode: rgbd, size: [64, 64], max_dist: 2.0, noise: 0.05}

- `rangefinder`: N MuJoCo rangefinder sensors the body adapter mounts on the
  head (`BodyAdapter.add_rangefinders`), a level fan of `fov` degrees around
  the heading, ordered from right (-fov/2) to left (+fov/2). Distance / `max_dist` in [0, 1]; no hit reads 1. `noise` is a
  distance standard deviation [m] (R35 `body` layer).
- `camera`: the adapter's eye camera (`BodyAdapter.view_camera`) rendered with
  one offscreen renderer per env, once per env step (the high-level decision),
  never per body step. uint8 image (C, H, W): `rgb` 3 channels, `depth` 1
  (clipped at `max_dist`, scaled to 0-255), `rgbd` 4. `size` is [height,
  width]. `noise` is a per-frame brightness offset standard deviation as a
  fraction of full scale on the colour channels (R35 `body` layer). Shadows and
  reflections are off (they double the render time, OQ11).

Item classes take `(env, spec, **params)` before the model is compiled and are
called with the env afterwards (see `obs.make_items`).
"""

from __future__ import annotations

import mujoco
import numpy as np

from .registry import register_obs

CAMERA_MODES = {"rgb": 3, "depth": 1, "rgbd": 4}
CAMERA_SIZE_RANGE = (36, 256)      # SB3's NatureCNN needs >= 36 px


class ObsParamError(ValueError):
    pass


def _positive(name: str, value, integer: bool = False) -> float:
    try:
        v = int(value) if integer else float(value)
    except (TypeError, ValueError):
        raise ObsParamError(f"{name} must be a {'whole ' if integer else ''}number, got {value!r}") from None
    if integer and v != value:
        raise ObsParamError(f"{name} must be a whole number, got {value!r}")
    if v <= 0:
        raise ObsParamError(f"{name} must be > 0, got {value!r}")
    return v


def _noise(value) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ObsParamError(f"noise must be a number, got {value!r}") from None
    if v < 0:
        raise ObsParamError(f"noise must be >= 0, got {value!r}")
    return v


@register_obs("rangefinder", replace=True)
class Rangefinder:
    """Head rangefinder fan (R39): `rays` distances / `max_dist`, right to left."""

    sensor = True

    def __init__(self, env, spec, rays: int = 9, fov: float = 120.0, max_dist: float = 2.0,
                 noise: float = 0.0):
        self.rays = int(_positive("rangefinder rays", rays, integer=True))
        self.fov = _positive("rangefinder fov", fov)
        if self.fov > 360:
            raise ObsParamError(f"rangefinder fov must be <= 360 degrees, got {fov!r}")
        self.max_dist = _positive("rangefinder max_dist", max_dist)
        self.noise = _noise(noise)
        if env is None:          # validation only
            return
        half = np.deg2rad(self.fov) / 2
        yaws = np.linspace(-half, half, self.rays) if self.rays > 1 else np.zeros(1)
        self.names = env.body.add_rangefinders(spec, yaws, prefix=f"{env.obs_prefix}rf")
        self._adr: np.ndarray | None = None

    def __call__(self, env) -> np.ndarray:
        if self._adr is None:
            self._adr = np.array([int(env.model.sensor(n).adr[0]) for n in self.names])
        d = env.data.sensordata[self._adr].astype(np.float64)
        d[(d < 0) | ~np.isfinite(d)] = self.max_dist           # -1: no hit
        std = self.noise * env.noise_scale
        if std > 0:
            d = d + env.np_random.normal(0.0, std, d.shape)
        return np.clip(d, 0.0, self.max_dist) / self.max_dist


@register_obs("camera", replace=True)
class Camera:
    """Eye camera image (R39), uint8 (C, H, W)."""

    sensor = True
    image = True

    def __init__(self, env, spec, mode: str = "rgb", size=(64, 64), max_dist: float = 2.0,
                 noise: float = 0.0):
        if mode not in CAMERA_MODES:
            raise ObsParamError(f"camera mode must be one of {sorted(CAMERA_MODES)}, got {mode!r}")
        if (not isinstance(size, (list, tuple)) or len(size) != 2
                or not all(isinstance(s, int) and not isinstance(s, bool) for s in size)):
            raise ObsParamError(f"camera size must be [height, width] in pixels, got {size!r}")
        lo, hi = CAMERA_SIZE_RANGE
        if not all(lo <= s <= hi for s in size):
            raise ObsParamError(f"camera size must be within {lo}..{hi} px, got {size!r}")
        self.mode = mode
        self.height, self.width = int(size[0]), int(size[1])
        self.max_dist = _positive("camera max_dist", max_dist)
        self.noise = _noise(noise)
        self.shape = (CAMERA_MODES[mode], self.height, self.width)
        self._renderer: mujoco.Renderer | None = None

    def _setup(self, env) -> None:
        cam = env.body.view_camera(env.model)
        if cam is None:
            raise ObsParamError(f"body {type(env.body).__name__} has no eye camera (view_camera)")
        self._cam = cam
        self._renderer = mujoco.Renderer(env.model, height=self.height, width=self.width)
        for flag in (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION):
            self._renderer.scene.flags[flag] = False

    def __call__(self, env) -> np.ndarray:
        if self._renderer is None:
            self._setup(env)
        r = self._renderer
        r.update_scene(env.data, camera=self._cam)
        planes = []
        if self.mode in ("rgb", "rgbd"):
            rgb = r.render().astype(np.float64)
            std = self.noise * env.noise_scale
            if std > 0:
                rgb = rgb + env.np_random.normal(0.0, std) * 255.0
            planes.append(np.clip(np.rint(rgb), 0, 255).astype(np.uint8).transpose(2, 0, 1))
        if self.mode in ("depth", "rgbd"):
            r.enable_depth_rendering()
            try:
                depth = r.render()
            finally:
                r.disable_depth_rendering()
            depth = np.clip(depth, 0.0, self.max_dist) / self.max_dist
            planes.append(np.rint(depth * 255.0).astype(np.uint8)[None])
        return np.concatenate(planes, axis=0)

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
