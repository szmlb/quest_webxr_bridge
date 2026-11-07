"""Python reference of the WebXR to ROS frame conversion.

The runtime conversion is done only in the browser (web/js/core.js). This
module mirrors it for the fake publisher and for tests, which check that both
implementations agree and that the quaternion remapping equals
R_C_W * R * R_C_W^T.
"""

import numpy as np

# R_C_W: (x, y, z)_R = (-z, -x, y)_W
R_C_W = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def webxr_to_ros_vector(v):
    x, y, z = v
    return (-z, -x, y)


def webxr_to_ros_quaternion(q):
    """q = (x, y, z, w) in WebXR -> (x, y, z, w) in ROS."""
    x, y, z, w = q
    return (-z, -x, y, w)


def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def axis_angle_quaternion(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = np.sin(angle / 2.0)
    return (axis[0] * s, axis[1] * s, axis[2] * s, np.cos(angle / 2.0))
