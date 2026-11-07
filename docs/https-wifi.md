# Optional: HTTPS over Wi-Fi

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
