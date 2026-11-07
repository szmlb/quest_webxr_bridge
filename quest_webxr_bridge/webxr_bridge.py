"""Convert /webxr/* topics from the browser into Quest2ROS2-compatible topics.

Input (published by web/app.js through rosbridge, already in the ROS frame):
    /webxr/{left,right}/pose   geometry_msgs/PoseStamped
    /webxr/{left,right}/twist  geometry_msgs/Twist
    /webxr/{left,right}/joy    sensor_msgs/Joy

Output (same names and types as the Quest2ROS app):
    /q2r_{left,right}_hand_pose    geometry_msgs/PoseStamped
    /q2r_{left,right}_hand_twist   geometry_msgs/Twist
    /q2r_{left,right}_hand_inputs  quest2ros/OVR2ROSInputs

The latest sample of each topic is republished at a fixed rate. Samples older
than ``stale_timeout_s`` are never republished, because Quest2ROS2 interprets
pose changes as robot motion.
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from quest2ros.msg import OVR2ROSInputs
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from quest_webxr_bridge.mapping import JoyMapping, PressLatch, joy_to_inputs

SIDES = ('left', 'right')


class _Latest:
    """Most recent message on one topic and its arrival time (monotonic)."""

    def __init__(self):
        self.msg = None
        self.t = None

    def set(self, msg, now):
        self.msg = msg
        self.t = now

    def fresh(self, now, timeout):
        return self.t is not None and now - self.t <= timeout


class _Hand:
    def __init__(self):
        self.pose = _Latest()
        self.twist = _Latest()
        self.joy = _Latest()
        self.latch = PressLatch()
        self.streaming = False


class WebXRBridge(Node):

    def __init__(self):
        super().__init__('webxr_bridge')
        self.publish_rate_hz = self.declare_parameter('publish_rate_hz', 30.0).value
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        self.stale_timeout_s = self.declare_parameter('stale_timeout_s', 0.5).value
        self.mapping = JoyMapping(
            button_upper_index=self.declare_parameter('button_upper_index', 5).value,
            button_lower_index=self.declare_parameter('button_lower_index', 4).value,
            index_trigger_index=self.declare_parameter('index_trigger_index', 0).value,
            middle_trigger_index=self.declare_parameter('middle_trigger_index', 1).value,
            thumbstick_x_axis=self.declare_parameter('thumbstick_x_axis', 2).value,
            thumbstick_y_axis=self.declare_parameter('thumbstick_y_axis', 3).value,
            invert_thumbstick_y=self.declare_parameter('invert_thumbstick_y', True).value,
        )

        self.hands = {}
        self.pubs = {}
        for side in SIDES:
            hand = _Hand()
            self.hands[side] = hand
            self.create_subscription(
                PoseStamped, f'/webxr/{side}/pose',
                lambda msg, h=hand: h.pose.set(msg, time.monotonic()), 10)
            self.create_subscription(
                Twist, f'/webxr/{side}/twist',
                lambda msg, h=hand: h.twist.set(msg, time.monotonic()), 10)
            self.create_subscription(
                Joy, f'/webxr/{side}/joy',
                lambda msg, h=hand: self._on_joy(h, msg), 10)
            self.pubs[side] = {
                'pose': self.create_publisher(PoseStamped, f'/q2r_{side}_hand_pose', 10),
                'twist': self.create_publisher(Twist, f'/q2r_{side}_hand_twist', 10),
                'inputs': self.create_publisher(OVR2ROSInputs, f'/q2r_{side}_hand_inputs', 10),
            }

        self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            f'webxr_bridge started: {self.publish_rate_hz:.1f} Hz, frame_id "{self.frame_id}", '
            f'stale timeout {self.stale_timeout_s:.2f} s, mapping {self.mapping}')

    def _on_joy(self, hand, msg):
        hand.joy.set(msg, time.monotonic())
        hand.latch.observe(joy_to_inputs(msg.buttons, msg.axes, self.mapping))

    def _on_timer(self):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        for side in SIDES:
            self._publish_side(side, now, stamp)

    def _publish_side(self, side, now, stamp):
        hand = self.hands[side]
        pubs = self.pubs[side]
        timeout = self.stale_timeout_s
        any_fresh = False

        if hand.pose.fresh(now, timeout):
            any_fresh = True
            out = PoseStamped()
            out.header.stamp = stamp
            out.header.frame_id = self.frame_id
            out.pose = hand.pose.msg.pose
            pubs['pose'].publish(out)

        if hand.twist.fresh(now, timeout):
            any_fresh = True
            pubs['twist'].publish(hand.twist.msg)

        if hand.joy.fresh(now, timeout):
            any_fresh = True
            joy = hand.joy.msg
            values = hand.latch.apply(joy_to_inputs(joy.buttons, joy.axes, self.mapping))
            pubs['inputs'].publish(OVR2ROSInputs(**values))

        if any_fresh and not hand.streaming:
            hand.streaming = True
            self.get_logger().info(f'{side} hand: receiving /webxr/{side}/* data, publishing')
        elif not any_fresh and hand.streaming:
            hand.streaming = False
            # Drop a press that was latched but never published.
            hand.latch = PressLatch()
            self.get_logger().warn(
                f'{side} hand: no /webxr/{side}/* data for {timeout:.2f} s, '
                'stopped publishing /q2r_* topics')


def main(args=None):
    rclpy.init(args=args)
    node = WebXRBridge()
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
