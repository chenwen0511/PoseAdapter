#!/bin/bash
# 宇树 Go2 一键标定（无头模式，不依赖 GUI）
# 用法：先启动相机节点发布 /camera/image_raw，将棋盘格对准相机后执行 ./calibrate_go2_onekey.sh
# 约 40 秒内多移动棋盘格（左右、上下、远近、倾斜），脚本自动采集约 20 张并标定，结果保存到 SAVE_PATH。

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 先加载 ROS2 环境（否则 rclpy 会报错；若在 conda/venv 下请保持本脚本用系统 python3.8）
[ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
# 与 publish_go2_camera.py 一致，使用 domain 1（相机节点占 domain 0）
export ROS_DOMAIN_ID=1

# ===================== 参数（与 calibrate_go2_camera.sh 一致） =====================
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/image_raw}"
CHESSBOARD_SIZE="${CHESSBOARD_SIZE:-8x5}"
SQUARE_SIZE="${SQUARE_SIZE:-0.025}"
SAVE_PATH="${SAVE_PATH:-$HOME/calibration_results}"
OUTPUT_YAML="$SAVE_PATH/calib_result.yaml"
# ==================================================================================

mkdir -p "$SAVE_PATH"

# 检查 ROS2 并 source（避免 venv/conda 下 python 与 ROS 不兼容）
if ! command -v ros2 &> /dev/null; then
    echo "正在 source ROS2 环境..."
    [ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash || true
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash || true
fi
if ! command -v ros2 &> /dev/null; then
    echo "错误：未找到 ros2，请先执行: source /opt/ros/foxy/setup.bash"
    exit 1
fi

# 使用与 ROS2 一致的 Python（Foxy 为 3.8），避免 venv/conda 的 python3 导致 rclpy 报错
if command -v python3.8 &> /dev/null; then
    PYTHON_CMD=python3.8
else
    PYTHON_CMD=python3
fi
echo "使用: $PYTHON_CMD"

# 检查话题是否有数据
echo "检查话题 $CAMERA_TOPIC 是否在发布..."
if ! timeout 3 ros2 topic hz "$CAMERA_TOPIC" &>/dev/null; then
    echo "未检测到 $CAMERA_TOPIC。请先在「另一终端」运行相机发布节点："
    echo "  source /opt/ros/foxy/setup.bash"
    echo "  cd $SCRIPT_DIR"
    echo "  /usr/bin/python3.8 publish_go2_camera.py --topic $CAMERA_TOPIC"
    echo "保持该节点运行，再回到本终端重新执行 ./calibrate_go2_onekey.sh"
    exit 1
fi

echo "====================================="
echo "  Go2 一键标定（无头模式）"
echo "  话题: $CAMERA_TOPIC"
echo "  棋盘格: $CHESSBOARD_SIZE, 方格 $SQUARE_SIZE m"
echo "  结果将保存到: $OUTPUT_YAML"
echo "====================================="
echo "请将棋盘格对准相机，并在约 40 秒内多移动棋盘格（左右、上下、远近、倾斜）。"
echo "====================================="

$PYTHON_CMD calibrate_opencv.py \
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
    $PYTHON_CMD read_calib_params.py -f "$OUTPUT_YAML" 2>/dev/null || true
    echo "====================================="
    echo "  yaml 已保存: $OUTPUT_YAML"
    echo "  pose_adapter 使用示例: calib_file:=$OUTPUT_YAML"
    echo "====================================="
else
    echo "标定未生成 $OUTPUT_YAML，请检查是否采集到足够图像（棋盘格需在画面内）。"
    exit 1
fi
