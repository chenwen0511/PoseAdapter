#!/bin/bash
# 宇树Go2相机标定脚本
# 作者：编程导师
# 使用说明：修改下方棋盘格参数后，直接执行 ./calibrate_go2_camera.sh

# ===================== 请修改以下参数 =====================
CAMERA_TOPIC="/camera/image_raw"  # Go2相机图像话题
CHESSBOARD_SIZE="8x5"             # 棋盘格内角点数量（如9×6棋盘格填8x5）
SQUARE_SIZE="0.025"               # 棋盘格方格边长（米），建议25mm填0.025
CAMERA_NAME="/camera"             # 相机命名空间
SAVE_PATH="$HOME/calibration_results"  # 建议将最终 yaml 复制到此路径，便于 read_calib_params.py 使用
# ==========================================================

# 创建保存目录（用于后续存放从 /tmp 复制或转换得到的 yaml）
mkdir -p $SAVE_PATH

# 检查依赖
if ! command -v ros2 &> /dev/null; then
    echo "错误：未找到ROS2，请先source ROS2环境！"
    exit 1
fi

# ROS2 版本（优先用环境变量，默认 foxy）
ROS_DISTRO="${ROS_DISTRO:-foxy}"
CALIB_PKG="ros-${ROS_DISTRO}-camera-calibration"
if ! dpkg -l | grep -q "$CALIB_PKG"; then
    echo "正在安装相机标定依赖: $CALIB_PKG"
    sudo apt update && sudo apt install -y "$CALIB_PKG"
fi

# 启动标定工具
echo "====================================="
echo "  宇树Go2相机标定工具启动中..."
echo "  棋盘格参数：$CHESSBOARD_SIZE, 方格边长：$SQUARE_SIZE m"
echo "  图像话题：$CAMERA_TOPIC"
echo "  保存路径：$SAVE_PATH"
echo "====================================="
echo "操作说明："
echo "1. 移动棋盘格覆盖画面四角、边缘、中心"
echo "2. 直到CALIBRATE按钮亮起，点击标定"
echo "3. 标定完成后点击SAVE保存结果"
echo "4. 最后点击COMMIT退出"
echo "====================================="

ros2 run camera_calibration cameracalibrator \
    --size $CHESSBOARD_SIZE \
    --square $SQUARE_SIZE \
    --ros-args \
    -r image:=${CAMERA_TOPIC} \
    -p camera:=${CAMERA_NAME}

# 提示结果位置（ROS2 默认将标定数据写入 /tmp/calibrationdata.tar.gz）
echo "====================================="
echo "标定完成后，数据默认在: /tmp/calibrationdata.tar.gz"
echo "解压后可用 camera_calibration_parsers 转为 yaml，或从终端输出的 K/D 复制。"
echo "将生成的 calib_params.yaml 放到 $SAVE_PATH 后，运行："
echo "  python3 read_calib_params.py -f $SAVE_PATH/calib_params.yaml"
echo "====================================="