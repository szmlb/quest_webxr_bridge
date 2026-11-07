# Interface specification

Determined from Quest2ROS2 at commit `07aaf65` (`Files_for_msg_pkg/`,
`q2r2_bringup/SimulationInput.py`, `CheckTCPconnection.py`,
`robot_arm_controller_base.py`, `left_arm_controller.py`, `right_arm_controller.py`).

## Message `quest2ros/OVR2ROSInputs`

```
bool    button_upper
bool    button_lower
float32 thumb_stick_horizontal
float32 thumb_stick_vertical
float32 press_index
float32 press_middle
```

(`quest2ros/OVR2ROSHapticFeedback` with `frequency`, `amplitude` also exists; it is
the ROS-to-headset direction and is not supported by this bridge.)

## Topics

| Topic | Type | Used by the Quest2ROS2 controllers |
|---|---|---|
| `/q2r_left_hand_pose`, `/q2r_right_hand_pose` | `geometry_msgs/PoseStamped` | yes |
| `/q2r_left_hand_inputs`, `/q2r_right_hand_inputs` | `quest2ros/OVR2ROSInputs` | yes |
| `/q2r_left_hand_twist`, `/q2r_right_hand_twist` | `geometry_msgs/Twist` | no (published for completeness and `CheckTCPconnection`) |

## Semantics in Quest2ROS2

* `SimulationInput.py` publishes at 30 Hz with `header.frame_id = "world"`,
  position in a ROS-style frame (x forward, y left, z up; e.g. right hand at
  y = -0.2, z = 0.5) and identity orientation.
* The controllers use the pose **relatively** (see [Relative motion](#relative-motion)
  below). The offset is added directly in the robot base frame and `frame_id` is
  ignored, so the incoming pose must have ROS axes (x forward, y left, z up)
  aligned with the robot base. A 20-sample moving average is applied to the pose.
* `button_upper` rising edge: toggle the gripper (GripperCommand action).
* `button_lower` rising edge: toggle pose streaming (arm enable) and reset the
  anchor (robot and controller).
* Pose updates are ignored if the TF lookup of the end effector fails.

### Relative motion

On the first (or re-anchored) sample, the controllers store the controller pose
$`(\mathbf{p}_{\mathrm{quest},0}, q_{\mathrm{quest},0})`$ and the robot
end-effector pose $`(\mathbf{p}_{\mathrm{robot},0}, q_{\mathrm{robot},0})`$ from TF
(`base_frame_id` to `end_effector_link_name`). After that the target is

```math
\mathbf{p}_{\mathrm{target}} = \mathbf{p}_{\mathrm{robot},0} + \left(\mathbf{p}_{\mathrm{quest}} - \mathbf{p}_{\mathrm{quest},0}\right)
```

```math
q_{\mathrm{target}} = \left(q_{\mathrm{quest}} \otimes q_{\mathrm{quest},0}^{-1}\right) \otimes q_{\mathrm{robot},0}
```

where $`\otimes`$ is the quaternion product.

## Button mapping

The browser publishes `sensor_msgs/Joy` in xr-standard order. `webxr_bridge`
maps it as follows (indices are ROS parameters, see [configuration](configuration.md#webxr_bridge)):

| OVR2ROSInputs field | Joy source (default) | Right controller | Left controller | Quest2ROS2 action |
|---|---|---|---|---|
| `button_upper` | `buttons[5]` | B | Y | toggle gripper |
| `button_lower` | `buttons[4]` | A | X | toggle arm enable + anchor reset |
| `press_index` | analog value of button 0 | index trigger | index trigger | - |
| `press_middle` | analog value of button 1 | grip (squeeze) | grip (squeeze) | - |
| `thumb_stick_horizontal` | `axes[2]` | thumbstick right = +1 | same | - |
| `thumb_stick_vertical` | `-axes[3]` | thumbstick forward = +1 | same | - |

xr-standard button order for Touch controllers: 0 trigger, 1 squeeze, 2 touchpad
(not present), 3 thumbstick press, 4 A/X, 5 B/Y, 6 thumbrest (if reported).
WebXR reports thumbstick forward as negative y, hence the sign flip.

Joy layout used between browser and bridge:

* `buttons[i]`: 1 if button `i` is pressed, else 0.
* `axes[0..3]`: touchpad x, touchpad y, thumbstick x, thumbstick y. Browsers that
  report only two axes (thumbstick) are normalized into slots 2 and 3.
* `axes[4 + i]`: analog value (0 to 1) of button `i`.

Short presses are latched so that a press shorter than one output period
(33 ms at 30 Hz) still produces one rising edge on `/q2r_*_hand_inputs`.

## Notes on Quest2ROS2 (upstream, not modified)

* The bringup package is `q2r2_bringup` (`ros2 run q2r2_bringup ...`).
* The message package name is inconsistent upstream: the README, the message
  package, `SimulationInput.py`, `CheckTCPconnection.py`, and the controllers use
  `quest2ros`, while `q2r2_bringup/package.xml` declares `quest2ros2_msg` and
  `ros2quest.py` imports `quest2ros2_msg`. This bridge uses `quest2ros`.
  `ros2quest` (demo node) does not run unless a `quest2ros2_msg` package exists.
* `CheckTCPconnection` prints fields that do not exist in `OVR2ROSInputs`
  (`press_grip`, `thumbstick_x`) and therefore falls back to printing the whole
  message for the inputs topics. The topic monitoring works.
* `SimulationInput.py` holds `button_lower = True` permanently, which the
  controllers see as a single rising edge (arm streaming is toggled to disabled
  once). This bridge reports real button states, so each A/X press is one toggle.
