# Nodes and parameters

## `webxr_bridge`

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

## Web page URL parameters

| Parameter | Default | Description |
|---|---|---|
| `host`, `port` | `localhost`, `9090` | rosbridge address |
| `wss=1` | off | Use `wss://` ([optional HTTPS mode](https-wifi.md)) |
| `rate` | 0 | Max publish rate per hand in Hz (0 = every XR frame) |
| `space` | `grip` | `target` uses `targetRaySpace` if `gripSpace` is unavailable |

Example: `http://localhost:8000/?rate=60`.

## `bridge.launch.py`

Starts `rosbridge_websocket` (port `rosbridge_port`, default 9090),
`python3 -m http.server` for `web/` (port `http_port`, default 8000, bound to
0.0.0.0) and `webxr_bridge`. Launch arguments: `http_port`, `rosbridge_port`,
`publish_rate_hz`, `frame_id`, `stale_timeout_s`.
