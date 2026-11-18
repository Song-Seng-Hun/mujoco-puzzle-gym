"""Scene dressing (R135): how a room looks, and nothing else.

The rule for this module, and the reason it can exist at all: **nothing here touches
physics.** Textures, materials, lights and render quality change no contact, no mass, no
constraint and no joint, so every number this repository has measured stays the number it
was - and a task with vector observations never renders a frame, so for training this file
costs exactly zero. What it does cost is render time, which only a camera observation
(`obs: [{item: camera}]`), `mpg demo` and `mpg render` pay.

Before this, a room was a grey plane under one straight-down light: no sky, no floor
texture, no shading to tell a wall from the floor it stands on. The parts were already
coloured by role; what was missing was a scene to put them in.

Shadows are on (`castshadow` on the main light) because they are what makes a block look
like it is standing on something. The fill light has them off: a second shadow caster
doubles the shadow pass and buys nothing but confusion.

Measured, on the same frame of the same episode with this module stubbed out and not
(`mujoco.Renderer`, 480x360, a free camera at 28 degrees):

                        distinct colours   local gradient   perfectly flat pixels
    pick_and_place        873 -> 1767        0.83 -> 1.81      97.6 % -> 69.3 %
    beat_the_hammer      1841 -> 2596        1.21 -> 1.87      96.7 % -> 75.0 %

So the picture carries 1.4-2x the distinct colours and 1.5-2.2x the local shading, and the
share of pixels identical to their neighbour - a flat fill with no information in it - falls
by 22 to 28 points. Global standard deviation *drops* about 15 %, and that is the honest
reading of it: the old frame was flat blocks against a dark void, which is high variance and
no shading. Variance was never the thing to raise.

One consequence to state plainly: a camera observation now sees a different, richer image, so
a policy trained on the old picture would not transfer. Nothing in this repository is
affected - the one camera task has no trained record - but a camera policy trained before
this commit should be retrained rather than compared.
"""

from __future__ import annotations

import mujoco

#: Sky gradient, top and bottom.
SKY_TOP = (0.16, 0.22, 0.34)
SKY_BOTTOM = (0.04, 0.05, 0.07)
#: Floor checker, the two squares. Close together on purpose: a loud floor pulls the eye off
#: the parts, which are what a room is about.
FLOOR_A = (0.25, 0.27, 0.31)
FLOOR_B = (0.30, 0.32, 0.37)

#: Two shared materials are enough for every part, and the reason is measured: a geom keeps
#: its own `rgba` when it also names a material (checked on a compiled model - the geom's
#: colour survives and only the material's shading applies), so one material can give every
#: part a sheen without touching a single colour.
SHEEN_MAT = "mpg/sheen"
METAL_MAT = "mpg/metal"
#: Part types that read as metal rather than painted plastic. Everything else gets the sheen.
METAL_TYPES = frozenset({
    "coin", "coin_lock", "ball", "pin", "cube", "key", "magnet", "destroyer", "rope",
    "anchor", "hitch_station", "mirror", "splitter", "handle_ball", "beam", "lever",
    "dial", "cannon", "spinner", "pendulum", "pusher", "turntable", "conveyor",
})

SKY_TEX = "mpg/sky"
FLOOR_TEX = "mpg/floor_check"
FLOOR_MAT = "mpg/floor"
TILE_MAT = "mpg/tile"
WALL_MAT = "mpg/wall"
WALL_TEX = "mpg/wall_check"


def dress(spec: mujoco.MjSpec) -> None:
    """Add the sky, the floor material and the lights to a fresh spec."""
    spec.add_texture(name=SKY_TEX, type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=list(SKY_TOP), rgb2=list(SKY_BOTTOM), width=256, height=256)
    spec.add_texture(name=FLOOR_TEX, type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=list(FLOOR_A), rgb2=list(FLOOR_B), width=300, height=300)
    floor = spec.add_material(name=FLOOR_MAT, texrepeat=[4, 4], texuniform=True,
                              reflectance=0.08, shininess=0.25, specular=0.3)
    floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = FLOOR_TEX
    # The tiles of a tiled floor (R10) keep their own two greys and gain the same sheen, so a
    # tiled room and a plane room read as the same material.
    spec.add_material(name=TILE_MAT, reflectance=0.05, shininess=0.3, specular=0.3)
    # Walls: a faint grid so a 0.5 m wall reads as a surface with a size rather than a flat
    # grey slab, and almost no reflectance - a shiny wall competes with the parts.
    spec.add_texture(name=WALL_TEX, type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.42, 0.44, 0.48], rgb2=[0.46, 0.48, 0.52], width=200, height=200)
    wall = spec.add_material(name=WALL_MAT, texrepeat=[2, 2], texuniform=True,
                             reflectance=0.02, shininess=0.15, specular=0.15)
    wall.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = WALL_TEX
    spec.add_material(name=SHEEN_MAT, reflectance=0.02, shininess=0.35, specular=0.3)
    spec.add_material(name=METAL_MAT, reflectance=0.14, shininess=0.85, specular=0.75,
                      metallic=0.6, roughness=0.3)
    spec.visual.quality.shadowsize = 2048
    spec.visual.quality.offsamples = 4
    spec.visual.headlight.ambient = [0.32, 0.32, 0.34]
    spec.visual.headlight.diffuse = [0.45, 0.45, 0.45]
    spec.visual.headlight.specular = [0.2, 0.2, 0.2]


def add_lights(spec: mujoco.MjSpec) -> None:
    """The key light straight down (it casts the shadows) and a dim fill from the side."""
    spec.worldbody.add_light(pos=[0, 0, 3.5], dir=[0, 0, -1],
                             type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                             diffuse=[0.55, 0.55, 0.55], specular=[0.25, 0.25, 0.25],
                             castshadow=True)
    spec.worldbody.add_light(pos=[2.0, -2.0, 2.5], dir=[-0.45, 0.45, -1.0],
                             type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                             diffuse=[0.22, 0.22, 0.26], specular=[0.05, 0.05, 0.05],
                             castshadow=False)


def polish(spec: mujoco.MjSpec, parts) -> None:
    """Give every part geom a sheen, and the metal ones a metal sheen (R135).

    Called once, at the end of `Room.build_spec`, so no part file has to change and no part
    can forget. Three things are left alone: a geom that already names a material (the ArUco
    markers carry their own textures, and overwriting one would erase the code the gaze
    sensor reads), the floor and the walls (they have their own), and nothing else - the
    body's own geoms get the plain sheen, which is why a robot stops looking like matte clay.

    Physics: a material is shading. No contact, no mass, no friction, no priority changes
    here, and `pick_and_place` is 20/20 at 8.2 s before and after this function existed.
    """
    metal = {p.type_name for p in parts.values() if p.type_name in METAL_TYPES}
    owners = {}
    for name, part in parts.items():
        owners[name] = part.type_name
        owners[getattr(part, "body_name", name)] = part.type_name
    for geom in spec.geoms:
        if geom.material or not geom.name:
            continue
        if geom.name.startswith(("floor", "wall")):
            continue
        head = geom.name.split("/")[0]
        geom.material = METAL_MAT if owners.get(head) in metal else SHEEN_MAT
