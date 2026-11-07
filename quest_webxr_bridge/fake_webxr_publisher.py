"""Headset-free test publisher for /webxr/* topics.

Publishes the same topics and message types as web/app.js. The motion is
generated in the WebXR frame (+X right, +Y up, -Z forward) and converted with
the same mapping as the browser, so the output exercises the frame convention:

* Each hand moves on a horizontal circle (radius ``radius_m``) in front of the
  user and yaws back and forth about the vertical axis.
* Upper button (B / Y) is pressed for 0.2 s every ``upper_period_s``, which
  toggles the gripper in Quest2ROS2.
* Lower button (A / X) is pressed for 0.2 s every ``lower_period_s``, which
  toggles arm enable and resets the anchor in Quest2ROS2 (0 disables it).
* The trigger and squeeze values follow slow sine waves; the thumbstick
  traces a circle.

Usage:
    ros2 run quest_webxr_bridge fake_webxr_publisher
    ros2 run quest_webxr_bridge fake_webxr_publisher --ros-args -p side:=right
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from quest_webxr_bridge.frames import webxr_to_ros_quaternion, webxr_to_ros_vector

PRESS_DURATION_S = 0.2
NUM_BUTTONS = 7  # trigger, squeeze, touchpad, thumbstick, A/X, B/Y, thumbrest


def webxr_hand_state(side, t, radius, period):
    """Pose and velocity of one hand in the WebXR frame at time t."""
    sign = -1.0 if side == 'left' else 1.0
    center = (0.2 * sign, 1.1, -0.4)
    w = 2.0 * math.pi / period
    position = (
        center[0] + radius * math.cos(w * t),
        center[1],
        center[2] + radius * math.sin(w * t),
    )
    linear = (-radius * w * math.sin(w * t), 0.0, radius * w * math.cos(w * t))

    # Yaw about +Y (up): angle(t) = A sin(w t)
    amp = 0.3
    yaw = amp * math.sin(w * t)
    yaw_rate = amp * w * math.cos(w * t)
    orientation = (0.0, math.sin(yaw / 2.0), 0.0, math.cos(yaw / 2.0))
    angular = (0.0, yaw_rate, 0.0)
    return position, orientation, linear, angular


def pulse(t, period, offset):
    if period <= 0.0:
        return False
    return ((t - offset) % period) < PRESS_DURATION_S and t >= offset


class FakeWebXRPublisher(Node):

    def __init__(self):
        super().__init__('fake_webxr_publisher')
        self.side = self.declare_parameter('side', 'both').value
        self.rate_hz = self.declare_parameter('rate_hz', 72.0).value
        self.radius = self.declare_parameter('radius_m', 0.1).value
        self.period = self.declare_parameter('period_s', 6.0).value
        self.upper_period = self.declare_parameter('upper_period_s', 5.0).value
        self.lower_period = self.declare_parameter('lower_period_s', 10.0).value

        sides = ('left', 'right') if self.side == 'both' else (self.side,)
        self.pubs = {
            side: (
                self.create_publisher(PoseStamped, f'/webxr/{side}/pose', 10),
                self.create_publisher(Twist, f'/webxr/{side}/twist', 10),
                self.create_publisher(Joy, f'/webxr/{side}/joy', 10),
            )
            for side in sides
        }
        self.t0 = time.monotonic()
        self.create_timer(1.0 / self.rate_hz, self._on_timer)
        self.get_logger().info(
            f'fake_webxr_publisher: sides {sides}, {self.rate_hz:.0f} Hz, '
            f'upper press every {self.upper_period:.1f} s, lower press every {self.lower_period:.1f} s')

    def _on_timer(self):
        t = time.monotonic() - self.t0
        stamp = self.get_clock().now().to_msg()
        for side, (pose_pub, twist_pub, joy_pub) in self.pubs.items():
            pose_pub.publish(self.make_pose(side, t, stamp))
            twist_pub.publish(self.make_twist(side, t))
            joy_pub.publish(self.make_joy(t, stamp))

    def make_pose(self, side, t, stamp):
        p, q, _, _ = webxr_hand_state(side, t, self.radius, self.period)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'webxr'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = webxr_to_ros_vector(p)
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = webxr_to_ros_quaternion(q)
        return msg

    def make_twist(self, side, t):
        _, _, lin, ang = webxr_hand_state(side, t, self.radius, self.period)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = webxr_to_ros_vector(lin)
        msg.angular.x, msg.angular.y, msg.angular.z = webxr_to_ros_vector(ang)
        return msg

    def make_joy(self, t, stamp):
        trigger = 0.5 + 0.5 * math.sin(0.5 * t)
        squeeze = 0.5 + 0.5 * math.cos(0.5 * t)
        values = [0.0] * NUM_BUTTONS
        values[0] = trigger
        values[1] = squeeze
        values[4] = 1.0 if pulse(t, self.lower_period, 1.0) else 0.0
        values[5] = 1.0 if pulse(t, self.upper_period, 2.5) else 0.0
        msg = Joy()
        msg.header.stamp = stamp
        msg.header.frame_id = 'webxr'
        msg.buttons = [1 if v > 0.5 else 0 for v in values]
        msg.axes = [0.0, 0.0, math.cos(t), math.sin(t)] + values
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FakeWebXRPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
