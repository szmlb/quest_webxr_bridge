"""End-to-end test: fake publisher -> webxr_bridge -> /q2r_* topics.

Requires the quest2ros message package (Quest2ROS2 README step 5).
"""

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

quest2ros_msg = pytest.importorskip('quest2ros.msg')

from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402

from quest_webxr_bridge.fake_webxr_publisher import FakeWebXRPublisher  # noqa: E402
from quest_webxr_bridge.webxr_bridge import WebXRBridge  # noqa: E402

TOPICS = {
    f'/q2r_{side}_hand_{kind}': msg_type
    for side in ('left', 'right')
    for kind, msg_type in (('pose', PoseStamped), ('twist', Twist),
                           ('inputs', quest2ros_msg.OVR2ROSInputs))
}


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


def test_fake_to_q2r_topics():
    rclpy.init()
    try:
        fake = FakeWebXRPublisher()
        bridge = WebXRBridge()
        probe = rclpy.create_node('q2r_probe')
        received = {name: [] for name in TOPICS}
        for name, msg_type in TOPICS.items():
            probe.create_subscription(
                msg_type, name, lambda m, n=name: received[n].append((time.monotonic(), m)), 50)

        executor = SingleThreadedExecutor()
        for node in (fake, bridge, probe):
            executor.add_node(node)

        spin_for(executor, 2.0)

        for name, msgs in received.items():
            assert len(msgs) > 20, f'{name}: only {len(msgs)} messages'
        # Fixed output rate (30 Hz by default) regardless of the 72 Hz input.
        n = len(received['/q2r_right_hand_pose'])
        assert 40 <= n <= 75, n

        pose = received['/q2r_right_hand_pose'][-1][1]
        assert pose.header.frame_id == 'world'
        assert pose.pose.position.x > 0.2 and pose.pose.position.z > 1.0
        assert pose.pose.position.y < 0.0  # right hand is on -Y in ROS

        # Lower button pulse (t = 1.0 .. 1.2 s) must reach the output.
        assert any(m.button_lower for _, m in received['/q2r_left_hand_inputs'])
        # Triggers are analog values in [0, 1].
        assert all(0.0 <= m.press_index <= 1.0 for _, m in received['/q2r_left_hand_inputs'])

        # Stop the input: output must stop after stale_timeout_s (0.5 s).
        executor.remove_node(fake)
        fake.destroy_node()
        t_stop = time.monotonic()
        spin_for(executor, 1.5)
        late = [t for t, _ in received['/q2r_right_hand_pose'] if t > t_stop + 0.6]
        assert late == [], f'{len(late)} stale poses published after timeout'

        executor.shutdown()
        bridge.destroy_node()
        probe.destroy_node()
    finally:
        rclpy.shutdown()
