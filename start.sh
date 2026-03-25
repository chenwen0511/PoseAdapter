#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

ACTION="${1:-start}"
shift || true

PID_FILE="$SCRIPT_DIR/data/logs/pose_adapter.pid"
LOG_DIR="$SCRIPT_DIR/data/logs"
LOG_FILE="$LOG_DIR/pose_adapter.log"
DEFAULT_CALIB_FILE="$SCRIPT_DIR/src/calibrate/calibration_results/rtsp_camera_calib.yaml"
CALIB_FILE="${CALIB_FILE:-$DEFAULT_CALIB_FILE}"
DEFAULT_MODEL_FILE="$SCRIPT_DIR/model/best.pt"
MODEL_FILE="${MODEL_FILE:-$DEFAULT_MODEL_FILE}"

export BODY="${BODY:-ZSI-1}"
DEFAULT_ZSI_SDK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)/zsibot_sdk"
export ZSI_SDK_ROOT="${ZSI_SDK_ROOT:-$DEFAULT_ZSI_SDK_ROOT}"
export ZSI_LOCAL_IP="${ZSI_LOCAL_IP:-192.168.234.15}"
export ZSI_LOCAL_PORT="${ZSI_LOCAL_PORT:-43988}"
export ZSI_DOG_IP="${ZSI_DOG_IP:-192.168.234.1}"

mkdir -p "$LOG_DIR"

# 本地 RTSP 流媒体服务
MEDIAMTX_DIR="$SCRIPT_DIR/mediamtx"
MEDIAMTX_BIN="$MEDIAMTX_DIR/mediamtx"
MEDIAMTX_YAML="$MEDIAMTX_DIR/mediamtx.yml"

download_mediamtx() {
    if [ ! -f "$MEDIAMTX_BIN" ]; then
        echo "首次使用，正在下载 mediamtx..."
        mkdir -p "$MEDIAMTX_DIR"
        
        ARCH=$(uname -m)
        case "$ARCH" in
            x86_64)
                URL="https://github.com/bluenviron/mediamtx/releases/download/v1.12.1/mediamtx_v1.12.1_linux_amd64.tar.gz"
                ;;
            aarch64|arm64)
                URL="https://github.com/bluenviron/mediamtx/releases/download/v1.12.1/mediamtx_v1.12.1_linux_arm64.tar.gz"
                ;;
            *)
                echo "不支持的架构: $ARCH"
                return 1
                ;;
        esac
        
        cd "$MEDIAMTX_DIR"
        curl -L -o mediamtx.tar.gz "$URL" 2>/dev/null || {
            echo "下载 mediamtx 失败，请检查网络"
            return 1
        }
        tar -xzf mediamtx.tar.gz
        rm -f mediamtx.tar.gz
        chmod +x mediamtx
        echo "mediamtx 下载完成"
    fi
}

ensure_mediamtx() {
    # 如果配置了本地 RTMP，推流地址包含 127.0.0.1，则自动启动 mediamtx
    local rtmp_url="$(grep -E '^\s*rtmp_url:' "$SCRIPT_DIR/config/params.yaml" 2>/dev/null | head -1 | sed 's/.*rtmp_url:*[[:space:]]*//' | tr -d '"' | tr -d "'")"
    
    if [[ "$rtmp_url" == "127.0.0.1"* ]] || [[ "$rtmp_url" == "localhost"* ]]; then
        # 检查 mediamtx 是否已运行
        if ! pgrep -f "mediamtx" > /dev/null 2>&1; then
            download_mediamtx || return
            
            # 创建默认配置
            if [ ! -f "$MEDIAMTX_YAML" ]; then
                cat > "$MEDIAMTX_YAML" << 'EOF'
rtspAddress: :8554
protocols: [tcp, udp]
encryption: "no"
rtmpAddress: :8554
webrtcAddress: :8888
srtAddress: :8890
logLevel: info
logDestinations: [console]
EOF
            fi
            
            echo "启动本地 RTSP 流媒体服务..."
            cd "$MEDIAMTX_DIR"
            nohup ./mediamtx "$MEDIAMTX_YAML" >> "$LOG_DIR/mediamtx.log" 2>&1 &
            sleep 2
            echo "mediamtx 已启动"
        fi
        
        echo "=========================================="
        echo "本地流服务已启动"
        echo "RTSP 拉流地址: rtsp://127.0.0.1:8554/pose"
        echo "WebRTC 拉流地址: webrtc://127.0.0.1:8888/pose"
        echo "=========================================="
    fi
}

is_running() {
  if [ -f "$PID_FILE" ]; then
    pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      return 0
    fi
  fi
  return 1
}

cleanup_stale_processes() {
  stale_pids="$(ps -eo pid,args | awk '/pose_adapter_node\.py/ && $0 !~ /awk/ {print $1}')"
  if [ -n "$stale_pids" ]; then
    echo "检测到残留 pose_adapter_node.py 进程: $stale_pids"
    echo "正在清理残留进程..."
    for p in $stale_pids; do
      kill "$p" 2>/dev/null || true
    done
    sleep 1
    for p in $stale_pids; do
      kill -9 "$p" 2>/dev/null || true
    done
  fi
}

check_port_conflict() {
  conflict_line="$(ss -lupn 2>/dev/null | awk '/:43988/ {print}')"
  if [ -n "$conflict_line" ]; then
    echo "警告: 端口 43988 已被占用："
    echo "$conflict_line"
    echo "请确认无其他控制程序占用后再启动。"
  fi
}

ensure_env() {
  source /opt/ros/humble/setup.bash
  source "$SCRIPT_DIR/install/setup.bash"
}

ensure_zsi_path() {
  if [ "$BODY" = "ZSI-1" ] && [ ! -d "$ZSI_SDK_ROOT" ]; then
    echo "错误: ZSI SDK 目录不存在: $ZSI_SDK_ROOT"
    echo "请在与 PoseAdapter 同层目录克隆:"
    echo "  cd $(cd "$SCRIPT_DIR/.." && pwd)"
    echo "  git clone git@github.com:zsibot/zsibot_sdk.git"
    exit 1
  fi
}

ensure_calib_file() {
  if [ ! -f "$CALIB_FILE" ]; then
    echo "错误: 标定文件不存在: $CALIB_FILE"
    echo "请先完成标定，或通过环境变量指定:"
    echo "  CALIB_FILE=/path/to/rtsp_camera_calib.yaml ./start.sh start"
    exit 1
  fi
}

ensure_model_file() {
  if [ ! -f "$MODEL_FILE" ]; then
    echo "错误: 模型文件不存在: $MODEL_FILE"
    echo "请确认模型路径，或通过环境变量指定:"
    echo "  MODEL_FILE=/home/nvidia/stephen/PoseAdapter/model/best.pt ./start.sh start"
    exit 1
  fi
}

start_service() {
  if is_running; then
    echo "pose_adapter 已在运行 (pid=$(cat "$PID_FILE"))"
    exit 0
  fi

  cleanup_stale_processes
  ensure_zsi_path
  ensure_calib_file
  ensure_model_file
  ensure_env
  check_port_conflict
  
  # 启动本地流媒体服务
  ensure_mediamtx

  echo "启动 pose_adapter..."
  echo "BODY=$BODY, ZSI_SDK_ROOT=$ZSI_SDK_ROOT"
  echo "标定文件: $CALIB_FILE"
  echo "模型文件: $MODEL_FILE"
  echo "日志目录: $LOG_DIR"
  echo "日志文件: $LOG_FILE"

  nohup bash -lc "cd \"$SCRIPT_DIR\" && source /opt/ros/humble/setup.bash && source \"$SCRIPT_DIR/install/setup.bash\" && BODY=\"$BODY\" ZSI_SDK_ROOT=\"$ZSI_SDK_ROOT\" ZSI_LOCAL_IP=\"$ZSI_LOCAL_IP\" ZSI_LOCAL_PORT=\"$ZSI_LOCAL_PORT\" ZSI_DOG_IP=\"$ZSI_DOG_IP\" CALIB_FILE=\"$CALIB_FILE\" MODEL_FILE=\"$MODEL_FILE\" ros2 launch pose_adapter pose_adapter.launch body_type:=\"$BODY\" calib_file:=\"$CALIB_FILE\" yolo_model_path:=\"$MODEL_FILE\" $*" >> "$LOG_FILE" 2>&1 &
  echo $! > "$PID_FILE"
  echo "已启动 (pid=$!)"
}

stop_service() {
  target_pids=""
  if [ -f "$PID_FILE" ]; then
    pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [ -n "$pid" ]; then
      target_pids="$target_pids $pid"
    fi
  fi

  related_pids="$(ps -eo pid,args | awk '
    /pose_adapter_node\.py/ && $0 !~ /awk/ {print $1}
    /ros2 launch pose_adapter pose_adapter\.launch/ && $0 !~ /awk/ {print $1}
  ')"
  if [ -n "$related_pids" ]; then
    target_pids="$target_pids $related_pids"
  fi
  target_pids="$(echo "$target_pids" | tr ' ' '\n' | awk 'NF{print $1}' | awk '!seen[$0]++')"

  if [ -z "$target_pids" ]; then
    echo "pose_adapter 未运行（未发现相关进程）"
    rm -f "$PID_FILE"
  else
    echo "停止以下相关进程: $target_pids"
    for p in $target_pids; do
      kill "$p" 2>/dev/null || true
    done
    sleep 1
    for p in $target_pids; do
      if kill -0 "$p" 2>/dev/null; then
        kill -9 "$p" 2>/dev/null || true
      fi
    done
    echo "相关进程已清理"
  fi

  # 如仍有 43988 占用，给出提示（可能被外部程序占用）。
  remaining="$(ss -lupn 2>/dev/null | awk '/:43988/ {print}')"
  if [ -n "$remaining" ]; then
    echo "警告: 端口 43988 仍被占用（可能不是 pose_adapter 进程）:"
    echo "$remaining"
  else
    echo "端口 43988 已释放"
  fi
  
  # 停止 mediamtx（可选，如果需要持续运行则注释掉下面这行）
  # if pgrep -f "mediamtx" > /dev/null 2>&1; then
  #   pkill -f "mediamtx" 2>/dev/null || true
  #   echo "mediamtx 已停止"
  # fi

  rm -f "$PID_FILE"
  echo "stop 完成"
}

status_service() {
  if is_running; then
    echo "pose_adapter 运行中 (pid=$(cat "$PID_FILE"))"
    echo "日志文件: $LOG_FILE"
  else
    echo "pose_adapter 未运行"
    echo "日志文件: $LOG_FILE"
  fi
}

show_logs() {
  touch "$LOG_FILE"
  tail -n 100 -f "$LOG_FILE"
}

case "$ACTION" in
  start)
    start_service
    ;;
  stop)
    stop_service
    ;;
  restart)
    stop_service || true
    start_service
    ;;
  status)
    status_service
    ;;
  logs)
    show_logs
    ;;
  *)
    echo "用法: $0 {start|stop|restart|status|logs} [ros2 launch 额外参数]"
    exit 1
    ;;
esac
