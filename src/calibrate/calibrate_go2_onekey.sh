#!/bin/bash
# 宇树 Go2 一键标定（无头模式，不依赖 GUI）
# 用法：先启动相机节点发布 /camera/image_raw，将棋盘格对准相机后执行 ./calibrate_go2_onekey.sh
# 约 40 秒内多移动棋盘格（左右、上下、远近、倾斜），脚本自动采集约 20 张并标定，结果保存到 SAVE_PATH。

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ===================== 参数（与 calibrate_go2_camera.sh 一致） =====================
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/image_raw}"
CHESSBOARD_SIZE="${CHESSBOARD_SIZE:-8x5}"
SQUARE_SIZE="${SQUARE_SIZE:-0.025}"
SAVE_PATH="${SAVE_PATH:-$HOME/calibration_results}"
OUTPUT_YAML="$SAVE_PATH/calib_result.yaml"
# ==================================================================================

mkdir -p "$SAVE_PATH"

# 检查 ROS2
if ! command -v ros2 &> /dev/null; then
    echo "正在 source ROS2 环境..."
    [ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash || true
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash || true
fi
if ! command -v ros2 &> /dev/null; then
    echo "错误：未找到 ros2，请先执行: source /opt/ros/foxy/setup.bash"
    exit 1
fi

# 检查话题是否有数据
echo "检查话题 $CAMERA_TOPIC 是否在发布..."
if ! timeout 3 ros2 topic hz "$CAMERA_TOPIC" &>/dev/null; then
    echo "警告：未检测到 $CAMERA_TOPIC 或无数据，请先启动相机节点再运行本脚本。"
    read -p "是否继续？(y/N) " -n 1 -r; echo
    [[ ! $REPLY =~ ^[Yy]$ ]] && exit 1
fi

echo "====================================="
echo "  Go2 一键标定（无头模式）"
echo "  话题: $CAMERA_TOPIC"
echo "  棋盘格: $CHESSBOARD_SIZE, 方格 $SQUARE_SIZE m"
echo "  结果将保存到: $OUTPUT_YAML"
echo "====================================="
echo "请将棋盘格对准相机，并在约 40 秒内多移动棋盘格（左右、上下、远近、倾斜）。"
echo "====================================="

python3 calibrate_opencv.py \
    --topic "$CAMERA_TOPIC" \
    --size "$CHESSBOARD_SIZE" \
    --square "$SQUARE_SIZE" \
    --out "$OUTPUT_YAML" \
    --headless

if [ -f "$OUTPUT_YAML" ]; then
    echo ""
    echo "====================================="
    echo "  标定完成，内参预览："
    echo "====================================="
    python3 read_calib_params.py -f "$OUTPUT_YAML" 2>/dev/null || true
    echo "====================================="
    echo "  yaml 已保存: $OUTPUT_YAML"
    echo "  pose_adapter 使用示例: calib_file:=$OUTPUT_YAML"
    echo "====================================="
else
    echo "标定未生成 $OUTPUT_YAML，请检查是否采集到足够图像（棋盘格需在画面内）。"
    exit 1
fi
