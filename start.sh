#!/bin/bash
# PoseAdapter 启动脚本
# 使用 ZSI-1 机器狗时设置 BODY=ZSI-1

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 设置机器狗类型
export BODY=${BODY:-ZSI-1}

# ZSI-1 SDK：默认与 PoseAdapter 同层的 zsibot_sdk
DEFAULT_ZSI_SDK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)/zsibot_sdk"
export ZSI_SDK_ROOT=${ZSI_SDK_ROOT:-$DEFAULT_ZSI_SDK_ROOT}

# ZSI-1 SDK 配置 (可选，根据实际情况修改)
export ZSI_LOCAL_IP=${ZSI_LOCAL_IP:-192.168.234.15}
export ZSI_LOCAL_PORT=${ZSI_LOCAL_PORT:-43988}
export ZSI_DOG_IP=${ZSI_DOG_IP:-192.168.234.1}

if [ "$BODY" = "ZSI-1" ] && [ ! -d "$ZSI_SDK_ROOT" ]; then
  echo "========================================"
  echo "错误: ZSI SDK 目录不存在: $ZSI_SDK_ROOT"
  echo "请在与 PoseAdapter 同层目录克隆 zsibot_sdk，例如:"
  echo "  cd $(cd "$SCRIPT_DIR/.." && pwd)"
  echo "  git clone git@github.com:zsibot/zsibot_sdk.git"
  echo "========================================"
  exit 1
fi

echo "========================================"
echo "PoseAdapter 启动"
echo "机器狗类型: $BODY"
echo "SDK路径: $ZSI_SDK_ROOT"
echo "本地IP: $ZSI_LOCAL_IP"
echo "机器狗IP: $ZSI_DOG_IP"
echo "========================================"

# 启动 ROS
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 启动 pose_adapter
roslaunch pose_adapter pose_adapter.launch "$@"
