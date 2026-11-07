# Troubleshooting

| Symptom | Cause and fix |
|---|---|
| "WebXR immersive-vr: NOT A SECURE CONTEXT" | The page was opened via an IP address over plain HTTP. Use `http://localhost:8000` with `scripts/setup_adb.sh`, or the [HTTPS mode](https-wifi.md) |
| "navigator.xr not available" / "immersive-vr not supported" | Update the Quest browser; make sure no other immersive app is running; reload the page. If your headset's browser has lost WebXR support, use the [HTTPS mode](https-wifi.md) from another WebXR-capable browser (e.g. Wolvic) |
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
