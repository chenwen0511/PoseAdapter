# PoseAdapter

宇树 Go2 机器狗巡检场景下，基于**前置相机 + 机身运动**的位姿自适应与电表读数采集方案（检测 + 追踪 + PnP 位姿 + 控制闭环 + OCR）。

---

## 目录规划

```
PoseAdapter/
├── README.md           # 本文件，项目与目录说明
├── doc/                # 方案与使用文档（与 src 模块对应）
│   ├── adapter_README.md   # 电表巡检方案：检测 / 追踪 / 位姿 / 控制 / OCR
│   └── calibrate_README.md  # 相机内参标定说明与经验参数
├── tests/              # 测试
│   ├── conftest.py     # 共享 fixture（内参 K/dist、标定 yaml 路径等）
│   ├── fixtures/       # 测试数据（如 calib_sample.yaml）
│   └── unit/           # 单元测试（calibrate 解析、adapter 占位）
└── src/                # 源码
    ├── adapter/        # 电表巡检适配：检测、追踪、位姿、控制、OCR
    │   ├── detector.py     # 目标检测（如 YOLOv8）
    │   ├── tracker.py      # 目标追踪（如 DeepSORT）
    │   ├── pose_solver.py  # PnP 位姿解算
    │   ├── controller.py   # 机身运动控制（cmd_vel / Go2 SDK）
    │   └── ocr.py          # 电表读数 OCR
    └── calibrate/      # 相机内参标定脚本与工具
        ├── calibrate_go2_camera.sh   # ROS2 标定启动脚本
        └── read_calib_params.py     # 读取标定 yaml，输出 K / 畸变
```

- **doc/**：集中存放方案说明与操作文档，不随代码分散。
- **src/adapter/**：巡检主流程（检测 → 追踪 → 位姿 → 控制 → OCR）。
- **src/calibrate/**：标定相关脚本，供内参标定与 `read_calib_params` 使用。
- **tests/**：单元测试与共享 fixture；在项目根目录执行 `pytest tests/ -v` 运行（见 [tests/README.md](tests/README.md)）。

---

## 快速入口

| 目的 | 入口 |
|------|------|
| 了解电表巡检方案与可行性 | [doc/adapter_README.md](doc/adapter_README.md) |
| 做相机内参标定、查经验内参 | [doc/calibrate_README.md](doc/calibrate_README.md) |
| 运行标定脚本、读取标定结果 | `src/calibrate/` |
| 运行单元测试 | `pytest tests/ -v`（见 [tests/README.md](tests/README.md)） |
=======
# PoseAdapter

宇树 Go2 机器狗巡检场景下，基于**前置相机 + 机身运动**的位姿自适应与电表读数采集方案（检测 + 追踪 + PnP 位姿 + 控制闭环 + OCR）。

---

## 目录规划

```
PoseAdapter/
├── README.md           # 本文件，项目与目录说明
├── start.sh            # 启动脚本（start/stop/restart/status）
├── data/               # 运行时数据（gitignore，自动创建）
│   └── logs/           # 日志目录
│       └── pose_adapter.log   # start.sh 启动时的输出日志
├── doc/                # 方案与使用文档（与 src 模块对应）
│   ├── adapter_README.md   # 电表巡检方案：检测 / 追踪 / 位姿 / 控制 / OCR
│   └── calibrate_README.md  # 相机内参标定说明与经验参数
├── tests/              # 测试
│   ├── conftest.py     # 共享 fixture（内参 K/dist、标定 yaml 路径等）
│   ├── fixtures/       # 测试数据（如 calib_sample.yaml）
│   └── unit/           # 单元测试（calibrate 解析、adapter 占位）
└── src/                # 源码
    ├── adapter/        # 电表巡检适配：检测、追踪、位姿、控制、OCR
    │   ├── detector.py     # 目标检测（如 YOLOv8）
    │   ├── tracker.py      # 目标追踪（如 DeepSORT）
    │   ├── pose_solver.py  # PnP 位姿解算
    │   ├── controller.py   # 机身运动控制（cmd_vel / Go2 SDK）
    │   └── ocr.py          # 电表读数 OCR
    └── calibrate/      # 相机内参标定脚本与工具
        ├── calibrate_go2_camera.sh   # ROS2 标定启动脚本
        └── read_calib_params.py     # 读取标定 yaml，输出 K / 畸变
```

- **doc/**：集中存放方案说明与操作文档，不随代码分散。
- **src/adapter/**：巡检主流程（检测 → 追踪 → 位姿 → 控制 → OCR）。
- **src/calibrate/**：标定相关脚本，供内参标定与 `read_calib_params` 使用。
- **tests/**：单元测试与共享 fixture；在项目根目录执行 `pytest tests/ -v` 运行（见 [tests/README.md](tests/README.md)）。

---

## 快速入口

| 目的 | 入口 |
|------|------|
| 了解电表巡检方案与可行性 | [doc/adapter_README.md](doc/adapter_README.md) |
| 构建 / 重新构建 pose_adapter 包 | 见下方“使用 catkin_make 重新构建”或 [doc/adapter_README.md#构建-pose_adapter-包](doc/adapter_README.md#构建-pose_adapter-包) |
| 启动 Adapter 节点 | `./start.sh start` 或 [doc/adapter_README.md#启动-adapter-节点](doc/adapter_README.md#启动-adapter-节点) |
| 做相机内参标定、查经验内参 | [doc/calibrate_README.md](doc/calibrate_README.md) |
| 运行标定脚本、读取标定结果 | `src/calibrate/` |
| 运行单元测试 | `pytest tests/ -v`（见 [tests/README.md](tests/README.md)） |

---

## 使用 catkin_make 重新构建

每次修改 `src/pose_adapter/` 下的 Python 节点或依赖后，建议在项目根目录重新构建一次工作空间，确保 `devel/setup.bash` 中的环境与最新代码一致：

```bash
cd ~/stephen/PoseAdapter       # 或你的实际工作空间路径
source /opt/ros/noetic/setup.bash
catkin_make
```

构建成功后，重新加载工作空间环境再启动：

```bash
source devel/setup.bash
./start.sh restart     # 或 ./start.sh start

---

## 机器狗类型

本工程支持两种机器狗类型：
- **GO2**: 使用 Unitree SDK (high_level)
- **ZSI-1**: 使用 zsibot_sdk (zsl-1)

### 启动方式

```bash
# ZSI-1 模式 (默认)
./start.sh

# 或
BODY=ZSI-1 roslaunch pose_adapter pose_adapter.launch

# GO2 模式
BODY=GO2 roslaunch pose_adapter pose_adapter.launch
```

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `BODY` | `ZSI-1` | 机器狗类型 |
| `ZSI_SDK_ROOT` | `/home/stephen/.openclaw/workspace/zsibot_sdk` | SDK 路径 |
| `ZSI_LOCAL_IP` | `192.168.1.100` | 本地 IP |
| `ZSI_LOCAL_PORT` | `43988` | 本地端口 |
| `ZSI_DOG_IP` | `192.168.234.1` | 机器狗 IP |

### ZSI-1 控制接口

- `move(vx, vy, yaw_rate)`: 速度控制
  - vx: 前进/后退速度 (m/s)
  - vy: 左右平移速度 (m/s)
  - yaw_rate: 偏航角速度 (rad/s)

- `attitudeControl(roll_vel, pitch_vel, yaw_vel, height_vel)`: 姿态控制
  - roll_vel: 翻滚角速度 (rad/s)
  - pitch_vel: 俯仰角速度 (rad/s)
  - yaw_vel: 偏航角速度 (rad/s)
  - height_vel: 高度变化速度 (m/s)

- `standUp()`: 站立
- `lieDown()`: 趴下

---

## 关键点提取方式

用于提升斜视角电表的位姿解算精度：

| 方式 | 说明 |
|------|------|
| `bbox` | 使用检测框的四个角点（默认，现有方式） |
| `contour` | 使用轮廓检测 + 角点提取（新增，推荐测试） |
| `keypoint` | 预留接口，需训练关键点检测模型 |

### 启动方式

```bash
# 使用 contour 方式
keypoint_method:=contour roslaunch pose_adapter pose_adapter.launch

# 或在 launch 中修改默认值
<arg name="keypoint_method" default="contour"/>
```
```
