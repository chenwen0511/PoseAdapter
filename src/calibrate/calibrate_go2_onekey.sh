#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

RTSP_URL="${RTSP_URL:-rtsp://192.168.234.1:8554/test}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/image_raw}"
CHESSBOARD_SIZE="${CHESSBOARD_SIZE:-8x5}"
SQUARE_SIZE="${SQUARE_SIZE:-0.025}"
CAMERA_NAME="${CAMERA_NAME:-rtsp_camera}"
SAVE_PATH="${SAVE_PATH:-$SCRIPT_DIR}"
OUTPUT_YAML="${OUTPUT_YAML:-$SAVE_PATH/rtsp_camera_calib.yaml}"
FPS="${FPS:-15}"

mkdir -p "$SAVE_PATH"

PYTHON_CMD="${PYTHON_CMD:-python3}"

echo "[1/3] 启动 RTSP 拉流发布节点..."
"$PYTHON_CMD" publish_go2_camera.py \
  --rtsp-url "$RTSP_URL" \
  --topic "$CAMERA_TOPIC" \
  --fps "$FPS" &
CAM_PID=$!
trap 'kill $CAM_PID >/dev/null 2>&1 || true' EXIT

sleep 2
echo "[2/3] 检查 $CAMERA_TOPIC 是否有数据..."
if ! timeout 5 ros2 topic echo "$CAMERA_TOPIC" --once >/dev/null 2>&1; then
  echo "错误：未收到图像消息，请检查 RTSP 地址是否可达：$RTSP_URL"
  exit 1
fi

echo "[3/3] 开始无头标定..."
"$PYTHON_CMD" calibrate_opencv.py \
  --topic "$CAMERA_TOPIC" \
  --size "$CHESSBOARD_SIZE" \
  --square "$SQUARE_SIZE" \
  --camera-name "$CAMERA_NAME" \
  --out "$OUTPUT_YAML" \
  --headless

if [ -f "$OUTPUT_YAML" ]; then
  echo "标定完成: $OUTPUT_YAML"
  "$PYTHON_CMD" read_calib_params.py --file "$OUTPUT_YAML" || true
else
  echo "标定失败：未生成输出文件。"
  exit 1
fi
