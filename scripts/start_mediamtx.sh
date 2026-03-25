#!/bin/bash
# 本地 RTSP 流媒体服务启动脚本
# 使用 mediamtx (原名 rtsp-simple-server)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MEDIAMTX_DIR="$SCRIPT_DIR/mediamtx"
MEDIAMTX_BIN="$MEDIAMTX_DIR/mediamtx"
MEDIAMTX_YAML="$MEDIAMTX_DIR/mediamtx.yml"

# 下载 mediamtx（如果不存在）
download_mediamtx() {
    if [ ! -f "$MEDIAMTX_BIN" ]; then
        echo "正在下载 mediamtx..."
        mkdir -p "$MEDIAMTX_DIR"
        
        # 检测架构
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
                exit 1
                ;;
        esac
        
        cd "$MEDIAMTX_DIR"
        curl -L -o mediamtx.tar.gz "$URL"
        tar -xzf mediamtx.tar.gz
        rm mediamtx.tar.gz
        chmod +x mediamtx
        echo "mediamtx 下载完成"
    fi
}

# 检查是否已运行
check_running() {
    if pgrep -f "mediamtx" > /dev/null 2>&1; then
        echo "mediamtx 已在运行"
        exit 0
    fi
}

# 启动 mediamtx
start_mediamtx() {
    download_mediamtx
    
    # 创建默认配置（如果不存在）
    if [ ! -f "$MEDIAMTX_YAML" ]; then
        cat > "$MEDIAMTX_YAML" << 'EOF'
# mediamtx 配置文件
# RTSP 地址: rtsp://127.0.0.1:8554/pose

rtspAddress: :8554
protocols: [tcp, udp]
encryption: "no"
rtspAddress: :8554
webrtcAddress: :8888
srtAddress: :8890

# RTMP 配置
# mediamtx 默认 RTMP 通常监听在 1935（而 RTSP 是 8554）
rtmpAddress: :1935

# 日志
logLevel: info
logDestinations: [console]
EOF
    fi
    
    echo "启动 mediamtx..."
    cd "$MEDIAMTX_DIR"
    ./mediamtx "$MEDIAMTX_YAML" &
    echo "mediamtx 已启动"
    echo "RTMP 推流地址: rtmp://127.0.0.1:1935/pose"
    echo "RTSP 拉流地址: rtsp://127.0.0.1:8554/pose"
    echo "WebRTC 地址: webrtc://127.0.0.1:8888/pose"
}

# 停止 mediamtx
stop_mediamtx() {
    echo "停止 mediamtx..."
    pkill -f "mediamtx" 2>/dev/null
    echo "mediamtx 已停止"
}

case "${1:-start}" in
    start)
        check_running
        start_mediamtx
        ;;
    stop)
        stop_mediamtx
        ;;
    restart)
        stop_mediamtx
        sleep 1
        start_mediamtx
        ;;
    *)
        echo "用法: $0 {start|stop|restart}"
        exit 1
        ;;
esac
