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
| `docs/` | Detailed documentation (see below) |

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

## Quick start

1. Enable developer mode for the Quest in the Meta Horizon mobile app, connect
   the headset by USB, and accept "Allow USB debugging" in the headset.
2. Forward the ports over USB: `scripts/setup_adb.sh`
3. Start rosbridge, the web server, and the bridge:
   `ros2 launch quest_webxr_bridge bridge.launch.py`
4. Open `http://localhost:8000` in the Quest browser and press "Enter VR".
5. Start the Quest2ROS2 controllers (e.g. `ros2 run q2r2_bringup right_arm_controller`).

No headset at hand? `ros2 run quest_webxr_bridge fake_webxr_publisher` publishes
synthetic controller data; see [Testing](docs/testing.md).

## Documentation

| Document | Contents |
|---|---|
| [Interface specification](docs/interface.md) | Message and topics, Quest2ROS2 semantics, button mapping, upstream notes |
| [Coordinate frames](docs/coordinate-frames.md) | WebXR to ROS frame conversion, reference space, velocity estimation |
| [Nodes and parameters](docs/configuration.md) | `webxr_bridge` parameters, web page URL parameters, launch arguments |
| [Testing](docs/testing.md) | Automated tests, headset-free checks, manual verification with the headset |
| [HTTPS over Wi-Fi](docs/https-wifi.md) | Optional setup when `adb reverse` is not available |
| [Troubleshooting](docs/troubleshooting.md) | Common problems and fixes |

## License

Apache License 2.0 (see [LICENSE](LICENSE)). The vendored `web/vendor/roslib.min.js`
is BSD-2-Clause (see [web/vendor/LICENSE.roslib](web/vendor/LICENSE.roslib)).
