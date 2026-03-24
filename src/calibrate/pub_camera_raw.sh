#!/bin/bash
set -e

[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

RTSP_URL="${RTSP_URL:-rtsp://192.168.234.1:8554/test}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/image_raw}"
FPS="${FPS:-15}"

python3 publish_go2_camera.py \
  --rtsp-url "$RTSP_URL" \
  --topic "$CAMERA_TOPIC" \
  --fps "$FPS"
