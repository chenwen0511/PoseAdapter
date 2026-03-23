#!/bin/bash
# PoseAdapter 启动脚本
# 使用 ZSI-1 机器狗时设置 BODY=ZSI-1

# 设置机器狗类型
export BODY=${BODY:-ZSI-1}

# ZSI-1 SDK 配置 (可选，根据实际情况修改)
export ZSI_SDK_ROOT=${ZSI_SDK_ROOT:-/home/stephen/.openclaw/workspace/zsibot_sdk}
export ZSI_LOCAL_IP=${ZSI_LOCAL_IP:-192.168.1.100}
export ZSI_LOCAL_PORT=${ZSI_LOCAL_PORT:-43988}
export ZSI_DOG_IP=${ZSI_DOG_IP:-192.168.234.1}

echo "========================================"
echo "PoseAdapter 启动"
echo "机器狗类型: $BODY"
echo "SDK路径: $ZSI_SDK_ROOT"
echo "本地IP: $ZSI_LOCAL_IP"
echo "机器狗IP: $ZSI_DOG_IP"
echo "========================================"

# 切换到脚本所在目录
cd "$(dirname "$0")"

# 启动 ROS
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 启动 pose_adapter
roslaunch pose_adapter pose_adapter.launch "$@"
