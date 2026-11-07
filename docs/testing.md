# Testing

## Testing without a headset

```bash
cd ~/quest_ws && source install/setup.bash

# Unit and integration tests (pytest + Node.js tests for web/js/core.js)
colcon test --packages-select quest_webxr_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Tests:

| File | Checks |
|---|---|
| `test/test_frames.py` | `C` is a proper rotation, quaternion remap equals $`C R C^\top`$, JS and Python conversions agree |
| `test/test_mapping.py` | Joy to OVR2ROSInputs mapping, thumbstick sign, short-press latch |
| `test/test_fake_publisher.py` | Fake publisher output is in the ROS frame; twist equals the pose derivative |
| `test/test_bridge_integration.py` | Fake publisher to `/q2r_*`: all six topics, 30 Hz output, `frame_id`, button edges, output stops after the stale timeout |
| `test/js/core.test.js` (via `test/test_js.py`) | Reconnection with backoff (max 5 s), dropping messages while disconnected, message formatting, velocity estimation, and advertise/publish through the vendored roslib |

Manual check with Quest2ROS2:

```bash
# terminal 1: bridge only (rosbridge and HTTP server are not needed here)
ros2 run quest_webxr_bridge webxr_bridge
# terminal 2: synthetic controller data
ros2 run quest_webxr_bridge fake_webxr_publisher
# terminal 3: all six /q2r_* topics should be listed without timeout
ros2 run q2r2_bringup CheckTCPconnection
```

To see the controllers produce relative motion without a robot, provide the end
effector TF that they look up, then watch the target pose:

```bash
ros2 run tf2_ros static_transform_publisher --frame-id bh_robot_base \
  --child-frame-id right_arm_link_ee --x 0.4 --z 0.6
ros2 run q2r2_bringup right_arm_controller
ros2 topic echo /bh_robot/right_arm_clik_controller/target_frame
```

The target moves on a circle around the anchored end-effector position. Every
5 s the controller logs "Upper button pressed. Toggling gripper..." (and
"Gripper action server not available" without a robot), and every 10 s
"Lower button pressed: pose streaming is now ENABLED/DISABLED".

## Manual verification with the headset

1. Enable developer mode for the Quest in the Meta Horizon mobile app, connect
   the headset by USB, and accept "Allow USB debugging" in the headset.
   `adb devices` must show the device as `device` (not `unauthorized`).
2. Run `scripts/setup_adb.sh` (installed copy:
   `$(ros2 pkg prefix quest_webxr_bridge)/share/quest_webxr_bridge/scripts/setup_adb.sh`).
3. Run `ros2 launch quest_webxr_bridge bridge.launch.py`.
4. Open `http://localhost:8000` in the Quest browser. "WebXR immersive-vr" must
   show OK and "rosbridge" must show CONNECTED.
5. Press "Enter VR". The background is dark green while publishing (dark red:
   rosbridge disconnected, dark yellow: no controller pose). Check the rate with
   `ros2 topic hz /webxr/right/pose` (close to the headset frame rate, about
   72 Hz on Quest 1, 72-120 Hz on later models) and `ros2 topic hz /q2r_right_hand_pose` (30 Hz).
6. Check the buttons with `ros2 topic echo /q2r_right_hand_inputs`: A sets
   `button_lower`, B sets `button_upper`, the trigger and grip change
   `press_index` and `press_middle`. Repeat for the left controller (X, Y).
7. Start the Quest2ROS2 controllers and operate the robot or its simulation.

Things to confirm on your headset (not verifiable without it):

* `gripSpace` is present for both controllers ("gripSpace: true" on the page).
* The button order matches the [button mapping table](interface.md#button-mapping). Press each button once, exit VR,
  and read "pressed this session" on the page, or watch
  `ros2 topic echo /webxr/diagnostics` while in VR (JSON with raw button values,
  published once per second).
* `ros2 topic echo /q2r_right_hand_pose`: moving the controller forward
  increases x, to the left increases y, up increases z.
