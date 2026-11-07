"""Numerical checks of the WebXR -> ROS frame conversion.

Verifies the claim that remapping the quaternion vector part like a position
equals R_C_W * R * R_C_W^T, and that the browser implementation
(web/js/core.js) produces the same numbers as the Python reference.
"""

import json
import os
import shutil
import subprocess

import numpy as np
import pytest

from quest_webxr_bridge.frames import (
    R_C_W, axis_angle_quaternion, quaternion_to_matrix,
    webxr_to_ros_quaternion, webxr_to_ros_vector)

CORE_JS = os.path.join(os.path.dirname(__file__), '..', 'web', 'js', 'core.js')


def random_unit_quaternions(n, seed=0):
    rng = np.random.default_rng(seed)
    q = rng.normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def test_r_c_w_is_proper_rotation():
    assert np.allclose(R_C_W @ R_C_W.T, np.eye(3))
    assert np.isclose(np.linalg.det(R_C_W), 1.0)


def test_axis_semantics():
    # WebXR forward (-Z) -> ROS +X, WebXR right (+X) -> ROS -Y, WebXR up (+Y) -> ROS +Z
    assert webxr_to_ros_vector((0, 0, -1)) == (1, 0, 0)
    assert webxr_to_ros_vector((1, 0, 0)) == (0, -1, 0)
    assert webxr_to_ros_vector((0, 1, 0)) == (0, 0, 1)


def test_vector_mapping_matches_matrix():
    rng = np.random.default_rng(1)
    for v in rng.normal(size=(50, 3)):
        assert np.allclose(webxr_to_ros_vector(v), R_C_W @ v)


def test_quaternion_mapping_matches_similarity_transform():
    for q in random_unit_quaternions(200):
        r_w = quaternion_to_matrix(q)
        r_r = quaternion_to_matrix(webxr_to_ros_quaternion(q))
        assert np.allclose(r_r, R_C_W @ r_w @ R_C_W.T, atol=1e-12)


def test_yaw_about_webxr_up_is_yaw_about_ros_z():
    q_r = webxr_to_ros_quaternion(axis_angle_quaternion((0, 1, 0), 0.7))
    assert np.allclose(q_r, axis_angle_quaternion((0, 0, 1), 0.7))


@pytest.mark.skipif(shutil.which('node') is None, reason='node not installed')
def test_browser_implementation_matches_python():
    qs = random_unit_quaternions(20, seed=3).tolist()
    vs = np.random.default_rng(4).normal(size=(20, 3)).tolist()
    script = (
        'const c = require(process.argv[1]);'
        'const inp = JSON.parse(process.argv[2]);'
        'const out = {'
        ' q: inp.q.map(([x, y, z, w]) => c.webxrToRosQuaternion({x, y, z, w})),'
        ' v: inp.v.map(([x, y, z]) => c.webxrToRosVector({x, y, z}))};'
        'console.log(JSON.stringify(out));'
    )
    res = subprocess.run(
        ['node', '-e', script, os.path.abspath(CORE_JS), json.dumps({'q': qs, 'v': vs})],
        check=True, capture_output=True, text=True)
    out = json.loads(res.stdout)
    for q, js in zip(qs, out['q']):
        assert np.allclose(webxr_to_ros_quaternion(q), [js['x'], js['y'], js['z'], js['w']])
    for v, js in zip(vs, out['v']):
        assert np.allclose(webxr_to_ros_vector(v), [js['x'], js['y'], js['z']])
