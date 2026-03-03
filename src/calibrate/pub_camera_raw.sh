conda activate task

export CYCLONEDDS_HOME=/home/unitree/cyclonedds/install

# 仅加载 ROS1（勿 source ROS2，避免 cyclonedds 冲突）
source /opt/ros/noetic/setup.bash

# 若用 conda 且 unitree_sdk2py 在 task：conda activate task
cd /home/unitree/stephen/PoseAdapter/src/calibrate

# 启动相机发布（保持运行）
python publish_go2_camera.py --no-network-interface