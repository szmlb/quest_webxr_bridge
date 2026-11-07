# quest_webxr_bridge

Use a Meta Quest headset (Quest 1, 2, 3, or Pro) as a teleoperation input
for [Quest2ROS2](https://github.com/Taokt/Quest2ROS2) without the Quest2ROS app.
The Quest browser runs a WebXR page that reads the Touch controllers and sends
them to ROS2 through rosbridge. The `webxr_bridge` node turns them into the
same six `/q2r_*` topics that the Quest2ROS app publishes, so the Quest2ROS2
`left_arm_controller` / `right_arm_controller` work unchanged.

No Windows PC, Unity, or APK build is needed. Everything runs on Ubuntu + ROS2,
and the page is served locally (no CDN or internet access at runtime).

```
Quest browser (web/index.html + app.js, WebXR)
    | WebSocket ws://localhost:9090 (USB, adb reverse)
    v
rosbridge_server
    | /webxr/{left,right}/{pose,twist,joy}, /webxr/diagnostics
    v
webxr_bridge (this package)
    | fixed-rate republish, Joy -> OVR2ROSInputs, stale-data cut-off
    v
/q2r_{left,right}_hand_{pose,inputs,twist}
    v
Quest2ROS2 left_arm_controller / right_arm_controller
```

The browser publishes only standard message types. The custom
`quest2ros/OVR2ROSInputs` message is built on the ROS side, so rosbridge does
not need to know about custom messages.

## Contents

| Path | Purpose |
|---|---|
| `web/index.html`, `web/app.js` | WebXR page: session, pose/button extraction, publishing, diagnostics |
| `web/js/core.js` | Headset-independent logic (frame conversion, velocity filter, message builders, reconnecting rosbridge client); unit tested with Node.js |
| `web/vendor/roslib.min.js` | roslib 1.4.1, vendored (BSD-2-Clause) |
| `quest_webxr_bridge/webxr_bridge.py` | `/webxr/*` to `/q2r_*` converter node |
| `quest_webxr_bridge/fake_webxr_publisher.py` | Headset-free test publisher (72 Hz) |
| `quest_webxr_bridge/mapping.py` | Joy to OVR2ROSInputs mapping (pure Python) |
| `quest_webxr_bridge/frames.py` | Python reference of the frame conversion (fake publisher and tests only) |
| `launch/bridge.launch.py` | rosbridge + HTTP server (port 8000) + webxr_bridge |
| `scripts/setup_adb.sh` | `adb reverse` for ports 8000 and 9090 |

## Installation

Target: Ubuntu 22.04 + ROS2 Humble (also tested on Ubuntu 24.04 + ROS2 Jazzy;
replace `humble` with `jazzy` in package names).

```bash
sudo apt install ros-humble-rosbridge-server adb \
                 ros-humble-tf-transformations ros-humble-control-msgs python3-numpy
# On older Ubuntu releases the adb package is called android-tools-adb.
```

`ros-humble-tf-transformations` and `ros-humble-control-msgs` are needed by the
Quest2ROS2 controllers, not by this package.

Workspace layout (follows the Quest2ROS2 README, steps 3 and 5):

```bash
mkdir -p ~/quest_ws/src && cd ~/quest_ws/src
git clone https://github.com/Taokt/Quest2ROS2.git
# Quest2ROS2 step 5: message package named "quest2ros"
mkdir quest2ros && cp -r Quest2ROS2/Files_for_msg_pkg/* quest2ros/
# this package
git clone https://github.com/szmlb/quest_webxr_bridge.git
cd ~/quest_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`quest_webxr_bridge` depends on the `quest2ros` package (it imports
`quest2ros.msg.OVR2ROSInputs`). The `ros_tcp_communication` / ROS-TCP-Endpoint
package from the Quest2ROS2 README is not needed: rosbridge replaces it.

## Interface Specification

Determined from Quest2ROS2 at commit `07aaf65` (`Files_for_msg_pkg/`,
`q2r2_bringup/SimulationInput.py`, `CheckTCPconnection.py`,
`robot_arm_controller_base.py`, `left_arm_controller.py`, `right_arm_controller.py`).

### Message `quest2ros/OVR2ROSInputs`

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

### Topics

| Topic | Type | Used by the Quest2ROS2 controllers |
|---|---|---|
| `/q2r_left_hand_pose`, `/q2r_right_hand_pose` | `geometry_msgs/PoseStamped` | yes |
| `/q2r_left_hand_inputs`, `/q2r_right_hand_inputs` | `quest2ros/OVR2ROSInputs` | yes |
| `/q2r_left_hand_twist`, `/q2r_right_hand_twist` | `geometry_msgs/Twist` | no (published for completeness and `CheckTCPconnection`) |

### Semantics in Quest2ROS2

* `SimulationInput.py` publishes at 30 Hz with `header.frame_id = "world"`,
  position in a ROS-style frame (x forward, y left, z up; e.g. right hand at
  y = -0.2, z = 0.5) and identity orientation.
* The controllers use the pose **relatively**: on the first (or re-anchored)
  sample they store the controller pose and the robot end-effector pose from TF
  (`base_frame_id` to `end_effector_link_name`). After that the target is

  ```math
  \begin{aligned}
  \mathbf{p}_\mathrm{target} &= \mathbf{p}_\mathrm{robot,0} + \left(\mathbf{p}_\mathrm{quest} - \mathbf{p}_\mathrm{quest,0}\right) \\
  q_\mathrm{target} &= \left(q_\mathrm{quest} \otimes q_\mathrm{quest,0}^{-1}\right) \otimes q_\mathrm{robot,0}
  \end{aligned}
  ```

  where the subscript $0$ denotes the pose stored at the anchor. The offset is
  added directly in the robot base frame and `frame_id` is ignored, so the
  incoming pose must have ROS axes (x forward, y left, z up) aligned with the
  robot base. A 20-sample moving average is applied to the pose.
* `button_upper` rising edge: toggle the gripper (GripperCommand action).
* `button_lower` rising edge: toggle pose streaming (arm enable) and reset the
  anchor (robot and controller).
* Pose updates are ignored if the TF lookup of the end effector fails.

### Button mapping

The browser publishes `sensor_msgs/Joy` in xr-standard order. `webxr_bridge`
maps it as follows (indices are ROS parameters, see below):

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

### Coordinate conversion

WebXR: +X right, +Y up, -Z forward. ROS: +X forward, +Y left, +Z up.

```math
\mathbf{p}_R = C\,\mathbf{p}_W, \qquad
C = \begin{bmatrix} 0 & 0 & -1 \\ -1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}, \qquad
\det C = 1
```

```math
\begin{aligned}
(x,\ y,\ z)_R &= (-z,\ -x,\ y)_W \\
R_R = C\,R_W\,C^\top \quad &\Longleftrightarrow \quad
(q_x,\ q_y,\ q_z,\ q_w)_R = (-q_z,\ -q_x,\ q_y,\ q_w)_W
\end{aligned}
```

Linear and angular velocities use the same mapping as positions. The
conversion is implemented only in the browser (`web/js/core.js`).
`test/test_frames.py` verifies numerically that the quaternion remapping equals
$C R C^\top$ (200 random rotations) and that the JavaScript implementation gives
the same numbers as the Python reference.

The reference space is `local-floor` (fallback `local`): its origin is on the
floor where the session started and its forward direction is where the
headset faced at that moment. ROS +X therefore points to where you looked when
pressing "Enter VR". Because Quest2ROS2 uses relative motion, only the axis
directions matter: face the robot's forward (+X) direction when entering VR, or
re-anchor after turning (lower button twice). The output `frame_id` is `world`
as in `SimulationInput.py`.

The Quest2ROS app itself is closed source, so its exact frame convention could
not be inspected. This bridge follows the ROS convention implied by
`SimulationInput.py` and by how the controllers add the offset in the robot
base frame.

### Velocities

`XRPose.linearVelocity` / `angularVelocity` are used when the browser provides
them. Otherwise velocities are estimated by finite differences with a
first-order low-pass filter (time constant 0.05 s); the estimate is reset after
tracking gaps longer than 0.25 s.

## Nodes and parameters

### `webxr_bridge`

Subscribes `/webxr/{left,right}/{pose,twist,joy}` and republishes the latest
sample of each at a fixed rate. The output header stamp is the ROS time of
publication (the browser clock is not synchronized with the PC).

| Parameter | Default | Description |
|---|---|---|
| `publish_rate_hz` | 30.0 | Output rate (same as `SimulationInput.py`) |
| `frame_id` | `world` | Output `header.frame_id` |
| `stale_timeout_s` | 0.5 | A topic is not republished when its last input is older than this. A warning is logged once per hand when all its inputs go stale |
| `button_upper_index` | 5 | Joy button for `button_upper` |
| `button_lower_index` | 4 | Joy button for `button_lower` |
| `index_trigger_index` | 0 | Joy button for `press_index` |
| `middle_trigger_index` | 1 | Joy button for `press_middle` |
| `thumbstick_x_axis` | 2 | Joy axis for `thumb_stick_horizontal` |
| `thumbstick_y_axis` | 3 | Joy axis for `thumb_stick_vertical` |
| `invert_thumbstick_y` | true | Make thumbstick forward positive |

If the diagnostics show that your headset's browser uses a different button order,
change the indices without touching code, for example
`ros2 run quest_webxr_bridge webxr_bridge --ros-args -p button_upper_index:=3`.

### Web page URL parameters

| Parameter | Default | Description |
|---|---|---|
| `host`, `port` | `localhost`, `9090` | rosbridge address |
| `wss=1` | off | Use `wss://` (optional HTTPS mode) |
| `rate` | 0 | Max publish rate per hand in Hz (0 = every XR frame) |
| `space` | `grip` | `target` uses `targetRaySpace` if `gripSpace` is unavailable |

Example: `http://localhost:8000/?rate=60`.

### `bridge.launch.py`

Starts `rosbridge_websocket` (port `rosbridge_port`, default 9090),
`python3 -m http.server` for `web/` (port `http_port`, default 8000, bound to
0.0.0.0) and `webxr_bridge`. Launch arguments: `http_port`, `rosbridge_port`,
`publish_rate_hz`, `frame_id`, `stale_timeout_s`.

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
| `test/test_frames.py` | `C` is a proper rotation, quaternion remap equals $C R C^\top$, JS and Python conversions agree |
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
* The button order matches the table above. Press each button once, exit VR,
  and read "pressed this session" on the page, or watch
  `ros2 topic echo /webxr/diagnostics` while in VR (JSON with raw button values,
  published once per second).
* `ros2 topic echo /q2r_right_hand_pose`: moving the controller forward
  increases x, to the left increases y, up increases z.

## Optional: HTTPS over Wi-Fi

Use this only when `adb reverse` is not available. WebXR requires a secure
context, so a page loaded from `http://<pc-ip>:8000` will not offer VR.

```bash
openssl req -x509 -newkey rsa:2048 -nodes -days 365 -subj "/CN=quest-bridge" \
  -keyout ~/quest_key.pem -out ~/quest_cert.pem
# rosbridge with TLS
ros2 run rosbridge_server rosbridge_websocket --ros-args \
  -p port:=9090 -p ssl:=true -p certfile:=$HOME/quest_cert.pem -p keyfile:=$HOME/quest_key.pem
# HTTPS static server for web/
cd $(ros2 pkg prefix quest_webxr_bridge)/share/quest_webxr_bridge/web
python3 -c "import http.server, ssl; s = http.server.ThreadingHTTPServer(('0.0.0.0', 8443), http.server.SimpleHTTPRequestHandler); c = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER); c.load_cert_chain('$HOME/quest_cert.pem', '$HOME/quest_key.pem'); s.socket = c.wrap_socket(s.socket, server_side=True); s.serve_forever()"
# bridge node
ros2 run quest_webxr_bridge webxr_bridge
```

In the Quest browser, first open `https://<pc-ip>:9090` and accept the
certificate warning (the page itself shows an error, which is fine), then open
`https://<pc-ip>:8443/?host=<pc-ip>&wss=1` and accept the warning again.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| "WebXR immersive-vr: NOT A SECURE CONTEXT" | The page was opened via an IP address over plain HTTP. Use `http://localhost:8000` with `scripts/setup_adb.sh`, or the HTTPS mode |
| "navigator.xr not available" / "immersive-vr not supported" | Update the Quest browser; make sure no other immersive app is running; reload the page. If your headset's browser has lost WebXR support, use the HTTPS mode from another WebXR-capable browser (e.g. Wolvic) |
| `adb devices` shows `unauthorized` | Put on the headset and accept "Allow USB debugging" (check "Always allow"). If no prompt appears: `adb kill-server && adb start-server`, replug the cable, and check developer mode in the Meta Horizon app |
| `adb devices` shows nothing | Use a data-capable USB cable; on Linux check `lsusb` and add a udev rule for vendor `2833` if needed |
| rosbridge: "Address already in use" | Another rosbridge (or process) uses 9090: `ss -ltnp \| grep 9090`, stop it, or launch with `rosbridge_port:=9091` and open the page with `?port=9091` (and `adb reverse tcp:9091 tcp:9091`) |
| HTTP server: "Address already in use" | Use `http_port:=8001` and `HTTP_PORT=8001 scripts/setup_adb.sh` |
| rosbridge status stays CONNECTING | `adb reverse --list` must list 9090; the launch must be running; after reconnecting the USB cable, run `setup_adb.sh` again |
| Page changes are not visible | The browser cached the old files; reload or clear the site data |
| Diagnostics show `gripSpace: false` | Open the page with `?space=target` to use the target ray pose instead |
| Buttons map to the wrong fields | Read the indices from the diagnostics and set the `*_index` parameters of `webxr_bridge` |
| Robot jumps when publishing resumes | After a pause (stale timeout, session restart) the next pose is compared with the old anchor. Press the lower button twice to re-anchor |
| `/q2r_*` topics stop | No browser data for `stale_timeout_s`: session ended, headset asleep, or rosbridge disconnected. This is intentional |

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
