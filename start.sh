#!/bin/bash
# PoseAdapter 电表巡检启动/停止/重启脚本
# 使用方法: ./start.sh {start|stop|restart|status}

# 获取脚本所在目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
WORK_DIR="$(pwd)"

# PID 与日志
PID_FILE="$WORK_DIR/pose_adapter.pid"
LOG_FILE="$WORK_DIR/data/logs/pose_adapter.log"

# Conda 环境（unitree_sdk2py 在 task 中，可覆盖）
CONDA_ENV="${CONDA_ENV:-task}"

# 标定文件（仅当设置 CALIB_FILE 时覆盖 launch 默认值）

# 初始化 conda 环境
_init_conda() {
    if [ -n "$CONDA_ENV" ]; then
        if [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
            source "$HOME/anaconda3/etc/profile.d/conda.sh"
        elif [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
            source "$HOME/miniconda3/etc/profile.d/conda.sh"
        elif [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
            source "/opt/conda/etc/profile.d/conda.sh"
        fi
        if command -v conda &> /dev/null; then
            if conda activate "$CONDA_ENV" 2>/dev/null; then
                echo "📦 使用 Conda 环境: $CONDA_ENV"
            else
                echo "⚠️  无法激活 conda 环境 '$CONDA_ENV'"
            fi
        fi
    fi
}

# 检查进程是否运行
_is_running() {
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if ps -p "$PID" > /dev/null 2>&1; then
            return 0
        else
            rm -f "$PID_FILE"
            return 1
        fi
    fi
    return 1
}

# 启动
_start() {
    if _is_running; then
        echo "⚠️  PoseAdapter 已在运行中 (PID: $(cat "$PID_FILE"))"
        return 1
    fi

    # 检查工作空间
    if [ ! -f "$WORK_DIR/devel/setup.bash" ] && [ ! -f "$WORK_DIR/install/setup.bash" ]; then
        echo "❌ 错误: 未找到 devel/setup.bash 或 install/setup.bash"
        echo "   请先构建工作空间: cd $WORK_DIR && catkin_make && source devel/setup.bash"
        exit 1
    fi

    mkdir -p "$(dirname "$LOG_FILE")"

    echo "🚀 启动 PoseAdapter 电表巡检节点..."
    echo "📁 工作目录: $WORK_DIR"

    _init_conda

    # ROS 环境
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash
    else
        echo "❌ 错误: 未找到 ROS 安装"
        exit 1
    fi

    if [ -f "$WORK_DIR/devel/setup.bash" ]; then
        source "$WORK_DIR/devel/setup.bash"
    elif [ -f "$WORK_DIR/install/setup.bash" ]; then
        source "$WORK_DIR/install/setup.bash"
    fi

    # 构建 roslaunch 参数
    # 注意：start.sh 为后台模式，强制关闭 image_view，因 nohup 子进程无法可靠获取 X 显示
    LAUNCH_ARGS="show_debug_image:=false"
    if [ -n "${CALIB_FILE:-}" ] && [ -f "$CALIB_FILE" ]; then
        LAUNCH_ARGS="$LAUNCH_ARGS calib_file:=$CALIB_FILE"
        echo "📷 使用标定文件: $CALIB_FILE"
    fi
    if [ -n "${NETWORK_INTERFACE:-}" ]; then
        LAUNCH_ARGS="$LAUNCH_ARGS network_interface:=$NETWORK_INTERFACE"
        echo "🔌 网络接口: $NETWORK_INTERFACE"
    fi
    if [ -n "${LOOP_HZ:-}" ]; then
        LAUNCH_ARGS="$LAUNCH_ARGS loop_hz:=$LOOP_HZ"
        echo "⏱️  主循环: $LOOP_HZ Hz"
    fi

    cd "$WORK_DIR"
    # 每次启动覆盖日志，便于查看当前运行输出
    {
        echo "===== PoseAdapter 启动于 $(date '+%Y-%m-%d %H:%M:%S') ====="
        echo "参数: $LAUNCH_ARGS"
        echo ""
    } > "$LOG_FILE"
    # 禁用 Python 输出缓冲，确保日志实时写入
    export PYTHONUNBUFFERED=1
    # 使用 stdbuf 强制行缓冲，避免输出被缓存
    nohup stdbuf -oL -eL roslaunch pose_adapter pose_adapter.launch $LAUNCH_ARGS >> "$LOG_FILE" 2>&1 &
    PID=$!
    echo $PID > "$PID_FILE"

    sleep 2
    if _is_running; then
        echo "✅ PoseAdapter 已启动 (PID: $PID)"
        echo "📝 日志: $LOG_FILE"
        echo "   查看: tail -f $LOG_FILE"
        return 0
    else
        echo "❌ 启动失败，请检查日志: $LOG_FILE"
        rm -f "$PID_FILE"
        return 1
    fi
}

# 停止
_stop() {
    if _is_running; then
        PID=$(cat "$PID_FILE")
        echo "🛑 正在停止 PoseAdapter (PID: $PID)..."
        kill -TERM "$PID" 2>/dev/null

        for i in {1..10}; do
            if ! ps -p "$PID" > /dev/null 2>&1; then
                break
            fi
            sleep 1
        done

        if ps -p "$PID" > /dev/null 2>&1; then
            echo "⚠️  强制终止..."
            kill -KILL "$PID" 2>/dev/null
        fi
        rm -f "$PID_FILE"
        echo "✅ 已停止"
    else
        echo "⚠️  PoseAdapter 未运行"
    fi
    # 清理可能残留的 pose_adapter_node 子进程（restart 时易产生僵尸进程）
    pkill -9 -f "pose_adapter_node.py" 2>/dev/null && echo "🧹 已清理残留进程" || true
    return 0
}

# 重启
_restart() {
    echo "🔄 重启 PoseAdapter..."
    _stop
    sleep 2
    _start
}

# 状态
_status() {
    if _is_running; then
        PID=$(cat "$PID_FILE")
        echo "✅ PoseAdapter 正在运行 (PID: $PID)"
        if [ -f "$LOG_FILE" ]; then
            echo "📝 日志: $LOG_FILE ($(du -h "$LOG_FILE" 2>/dev/null | cut -f1))"
        fi
        return 0
    else
        echo "❌ PoseAdapter 未运行"
        return 1
    fi
}

# 主逻辑
case "${1:-}" in
    start)
        _start
        ;;
    stop)
        _stop
        ;;
    restart)
        _restart
        ;;
    status)
        _status
        ;;
    *)
        echo "用法: $0 {start|stop|restart|status}"
        echo ""
        echo "命令:"
        echo "  start   - 启动 PoseAdapter 电表巡检节点"
        echo "  stop    - 停止节点"
        echo "  restart - 重启节点"
        echo "  status  - 查看运行状态"
        echo ""
        echo "环境变量:"
        echo "  CALIB_FILE        - 相机标定 yaml 路径"
        echo "  CONDA_ENV        - Conda 环境名称，默认 task（含 unitree_sdk2py）"
        echo "  NETWORK_INTERFACE - Unitree SDK 网卡（如 eth1），留空自动检测"
        echo "  LOOP_HZ          - 主循环频率（默认 5），低算力可设 3"
        echo ""
        echo "说明: start.sh 为后台模式，不启动 image_view。如需看调试图像，请在前台运行:"
        echo "  roslaunch pose_adapter pose_adapter.launch show_debug_image:=true"
        exit 1
        ;;
esac
exit $?
