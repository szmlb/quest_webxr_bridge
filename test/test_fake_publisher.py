"""Frame convention checks on the output of the fake publisher."""

import math

import numpy as np
import pytest
import rclpy

from quest_webxr_bridge.fake_webxr_publisher import FakeWebXRPublisher, webxr_hand_state
from quest_webxr_bridge.frames import quaternion_to_matrix


@pytest.fixture(scope='module')
def fake():
    rclpy.init()
    node = FakeWebXRPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_hands_are_in_front_and_above_floor(fake):
    from builtin_interfaces.msg import Time
    for side, y_sign in (('left', 1.0), ('right', -1.0)):
        p = fake.make_pose(side, 0.0, Time()).pose.position
        assert p.x > 0.2          # in front of the user (WebXR -Z)
        assert p.z > 1.0          # above the floor (WebXR +Y)
        assert math.copysign(1.0, p.y) == y_sign  # left hand has +Y in ROS


def test_twist_matches_pose_derivative(fake):
    from builtin_interfaces.msg import Time
    t, dt = 1.3, 1e-5

    def pos(tt):
        p = fake.make_pose('right', tt, Time()).pose.position
        return np.array([p.x, p.y, p.z])

    lin = fake.make_twist('right', t).linear
    assert np.allclose((pos(t + dt) - pos(t - dt)) / (2 * dt), [lin.x, lin.y, lin.z], atol=1e-6)


def test_yaw_rate_is_about_ros_z(fake):
    from builtin_interfaces.msg import Time
    t, dt = 0.4, 1e-5

    def rot(tt):
        o = fake.make_pose('left', tt, Time()).pose.orientation
        return quaternion_to_matrix((o.x, o.y, o.z, o.w))

    # Spatial angular velocity: skew(omega) = dR/dt R^T
    dr = (rot(t + dt) - rot(t - dt)) / (2 * dt)
    w = dr @ rot(t).T
    omega = np.array([w[2, 1], w[0, 2], w[1, 0]])
    ang = fake.make_twist('left', t).angular
    assert np.allclose(omega, [ang.x, ang.y, ang.z], atol=1e-6)
    assert abs(ang.x) < 1e-12 and abs(ang.y) < 1e-12


def test_joy_layout(fake):
    from builtin_interfaces.msg import Time
    msg = fake.make_joy(1.05, Time())  # lower button pulse starts at t = 1.0
    assert len(msg.axes) == 4 + len(msg.buttons)
    assert msg.buttons[4] == 1 and msg.buttons[5] == 0


def test_webxr_state_is_consistent():
    p0, _, lin, _ = webxr_hand_state('right', 0.0, 0.1, 6.0)
    assert p0[2] < 0  # forward in WebXR is -Z
    assert lin[1] == 0.0
