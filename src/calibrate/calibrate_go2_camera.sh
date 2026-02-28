#!/bin/bash
# 宇树Go2相机标定脚本
# 使用说明：
#   1. 标定工具带 GUI，需在有显示器的环境运行（接显示器的 Go2 或台式机/笔记本均可）。
#   2. 若在 Go2 上接显示器运行：建议在本地桌面终端执行；若通过 SSH，先执行 export DISPLAY=:0 再运行本脚本。
#   3. 若在另一台电脑运行：该机需能订阅到 /camera/image_raw（与狗同网、ROS_DOMAIN_ID 一致等）。
#   4. 修改下方参数后执行 ./calibrate_go2_camera.sh

# ===================== 请修改以下参数 =====================
CAMERA_TOPIC="/camera/image_raw"  # Go2 相机图像话题（与狗上发布的一致）
CHESSBOARD_SIZE="8x5"             # 棋盘格内角点数量（如 9×6 棋盘填 8x5）
SQUARE_SIZE="0.025"               # 棋盘格方格边长（米），如 25mm 填 0.025
CAMERA_NAME="/camera"             # 相机命名空间（单目仅一个 /camera）
SAVE_PATH="$HOME/calibration_results"  # 标定结果 yaml 建议放此路径
# ==========================================================

# 创建保存目录（用于后续存放从 /tmp 复制或转换得到的 yaml）
mkdir -p $SAVE_PATH

# 标定工具需要 GUI。若通过 SSH 运行且接了显示器，需先设置 DISPLAY=:0
if [ -z "${DISPLAY}" ]; then
    echo "提示：未检测到 DISPLAY。若本机已接显示器，请先执行: export DISPLAY=:0"
    echo "然后再运行本脚本；否则请在有显示器的电脑上运行并订阅 $CAMERA_TOPIC"
    export DISPLAY=:0
fi

# 检查依赖
if ! command -v ros2 &> /dev/null; then
    echo "错误：未找到ROS2，请先 source /opt/ros/foxy/setup.bash"
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

# 强制使用 X11 显示，避免 Qt 不弹窗（Go2/ARM 上常见）
export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM=xcb
# 若仍无窗口，可先手动执行：export QT_QPA_PLATFORM=xcb 和 export DISPLAY=:0 再运行
echo "DISPLAY=$DISPLAY  QT_QPA_PLATFORM=$QT_QPA_PLATFORM"

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
echo ""
echo "若本窗口始终不弹标定 GUI，请改用 OpenCV 标定脚本（无需 Qt 窗口或支持无头模式）："
echo "  python3 calibrate_opencv.py -t $CAMERA_TOPIC --size $CHESSBOARD_SIZE --square $SQUARE_SIZE -o $SAVE_PATH/calib_result.yaml"
echo "  无显示器时加 --headless 自动采集约 20 张后标定并保存。"
echo "====================================="