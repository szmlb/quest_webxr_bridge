#!/usr/bin/env bash
# Forward the Quest's localhost ports to this PC over USB (adb reverse), so the
# headset browser can open the page as http://localhost (a secure context).
set -euo pipefail

HTTP_PORT="${HTTP_PORT:-8000}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"

if ! command -v adb >/dev/null 2>&1; then
  echo "adb not found. Install it with: sudo apt install adb" >&2
  exit 1
fi

devices="$(adb devices | awk 'NR > 1 && NF >= 2 {print $1 " " $2}')"
if [ -z "$devices" ]; then
  echo "No adb device found. Connect the Quest via USB and enable developer mode." >&2
  exit 1
fi
if echo "$devices" | grep -q "unauthorized"; then
  echo "Device is unauthorized. Put on the headset and accept 'Allow USB debugging'." >&2
  exit 1
fi
if [ "$(echo "$devices" | grep -c " device$")" -gt 1 ]; then
  echo "More than one adb device found. Set ANDROID_SERIAL to choose one:" >&2
  echo "$devices" >&2
  exit 1
fi

adb reverse "tcp:${HTTP_PORT}" "tcp:${HTTP_PORT}"
adb reverse "tcp:${ROSBRIDGE_PORT}" "tcp:${ROSBRIDGE_PORT}"

echo "Active reverse forwards:"
adb reverse --list
echo
echo "Open this URL in the Quest browser:"
echo "  http://localhost:${HTTP_PORT}"
