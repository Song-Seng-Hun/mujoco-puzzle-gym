"""R15: spring pad launches objects (free joint velocity) and the body (`BodyAdapter.kick`)."""

from __future__ import annotations

import numpy as np
import pytest

from rooms import body_steps, open_room, tick
from test_doors import Source  # noqa: F401  (registers t_signal)

G = 9.81
HALF = 0.03


def spring_env(tmp_path, name="sp", extra=(), **spring):
    s = {"name": "sp", "preset": "device/spring_pad", "pos": [-0.8, 0.0], **spring}
    box = {"name": "box", "type": "t_box", "pos": [1.0, 1.0], "half": HALF}
    return open_room(tmp_path, [s, box, *extra], spawn=(-1.0, -1.0), name=name)


@pytest.mark.parametrize("vel", [[1.0, 0.0, 2.0], [0.6, 0.4, 1.5]])
def test_cube_launch_speed_and_landing(tmp_path, vel):
    env = spring_env(tmp_path, velocity=vel)
    box = env.parts["box"]
    box.put(env.data, [-0.8, 0.0, HALF])
    tick(env)
    assert env.world.requests and env.world.requests[0][0] == "kick"
    env._run_requests()
    v0 = env.data.qvel[box._v:box._v + 3].copy()
    assert np.allclose(v0, vel, rtol=0.05), v0                       # launch velocity +-5 %
    start = box.pos(env.data)
    track = []
    body_steps(env, 1.5, every=lambda e: track.append(box.pos(e.data)))
    track = np.array(track)
    land = next(i for i in range(1, len(track)) if track[i, 2] <= start[2] + 0.002 and track[i - 1, 2] > track[i, 2])
    dist = np.linalg.norm(track[land, :2] - start[:2])
    vh, vz = np.hypot(vel[0], vel[1]), vel[2]
    assert dist == pytest.approx(2 * vh * vz / G, rel=0.15)          # ballistic range +-15 %
    assert env.parts["sp"].uses == 1


def test_signal_and_cooldown(tmp_path):
    env = spring_env(tmp_path, name="sig", velocity=[0.0, 0.0, 1.0], cooldown=2.0,
                     extra=[{"name": "s", "type": "t_signal", "pos": [1.0, -1.0]}], **{"in": ["s"]})
    box = env.parts["box"]
    box.put(env.data, [-0.8, 0.0, HALF])
    body_steps(env, 0.5)
    assert env.parts["sp"].uses == 0 and box.pos(env.data)[2] < HALF + 0.002   # signal 0: nothing
    env.parts["s"].value = 1.0
    body_steps(env, 0.04)
    assert env.parts["sp"].uses == 1
    body_steps(env, 1.0)                                  # lands back on the pad within the cooldown
    assert env.parts["sp"].uses == 1
    body_steps(env, 1.5)
    assert env.parts["sp"].uses == 2


def test_body_kick(tmp_path):
    env = spring_env(tmp_path, name="kick", pos=[-1.0, -1.0], trigger="body", velocity=[0.3, 0.0, 0.5])
    tick(env)
    reqs = [r for r in env.world.requests if r[2] == "body"]
    assert reqs
    pose = env.read_body().pos.copy()
    env._run_requests()
    st = env.read_body()
    assert np.allclose(st.pos, pose)
    m = env.model
    va = m.jnt_dofadr[m.body_jntadr[env.body.root_body_id(m)]]
    v = env.data.qvel[va:va + 3]
    assert np.allclose(v, [0.3, 0.0, 0.5])
