# PoseAdapter

宇树 Go2 / ZSI-1 机器狗巡检场景下，基于**前置相机 + 机身运动**的位姿自适应与电表读数采集方案（检测 + 追踪 + PnP 位姿 + 控制闭环 + OCR）。

> ⚠️ **注意**：本项目已迁移到 **ROS2 Humble**

---

## 目录规划

```
PoseAdapter/
├── README.md           # 本文件
├── start.sh            # 启动脚本（ROS2 版）
├── doc/                # 方案与使用文档
│   ├── adapter_README.md   # 电表巡检方案
│   └── calibrate_README.md  # 相机内参标定说明
├── tests/              # 测试
│   ├── test_pipeline.py   # 完整流程单元测试
│   ├── conftest.py     # 共享 fixture
│   ├── fixtures/       # 测试数据
│   └── unit/           # 单元测试
└── src/                # 源码
    └── pose_adapter/   # ROS2 包
        ├── src/pose_adapter/  # Python 模块
        │   ├── detector.py     # 目标检测（YOLOv8）
        │   ├── tracker.py      # 目标追踪（DeepSORT）
        │   ├── pose_solver.py  # PnP 位姿解算
        │   ├── controller.py   # 运动控制（ZSI-1/Go2 SDK）
        │   ├── ocr.py          # 电表 OCR
        │   └── pose_adapter_node.py  # 主节点
        ├── launch/        # Launch 文件
        └── config/       # 配置文件
```

---

## 快速入口

| 目的 | 命令 |
|------|------|
| 构建包 | `colcon build --packages-select pose_adapter` |
| 后台启动服务 | `./start.sh start` |
| 查看服务状态 | `./start.sh status` |
| 停止服务 | `./start.sh stop` |
| 查看运行日志 | `./start.sh logs` |
| 启动 ZSI-1（推荐显式参数） | `BODY=ZSI-1 ros2 launch pose_adapter pose_adapter.launch body_type:=ZSI-1` |
| 启动 GO2 | `BODY=GO2 ros2 launch pose_adapter pose_adapter.launch` |
| 运行单元测试 | `python3.10 tests/test_pipeline.py` |

---

## 修改后重新运行

每次改完代码后，按下面步骤重新生效并启动：

```bash
cd /home/nvidia/stephen/PoseAdapter

# 1) 重新编译（只编 pose_adapter 包）
colcon build --packages-select pose_adapter

# 2) 重新加载环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3) 启动
BODY=ZSI-1 ros2 launch pose_adapter pose_adapter.launch body_type:=ZSI-1
# 或
BODY=GO2 ros2 launch pose_adapter pose_adapter.launch
```

说明：
- 若开了新终端，必须重新执行 `source /opt/ros/humble/setup.bash` 和 `source install/setup.bash`。
- 若启动报 `package 'pose_adapter' not found`，通常是第 2 步没有执行或执行顺序不对。
- ZSI SDK 仅支持 Python 3.10，本工程所有 Python 命令统一使用 `python3.10`。

若使用服务脚本（推荐）：

```bash
cd /home/nvidia/stephen/PoseAdapter
colcon build --packages-select pose_adapter
./start.sh restart
./start.sh status
```

服务日志位置：

- `data/logs/pose_adapter.log`

---

## 运行模式确认（ZSI-1 / GO2 / cmd_vel）

启动后请先看日志里的 `body_type` 与控制模式，判断是否走到预期 SDK：

- 期望 **ZSI-1 SDK**：日志应包含 `body_type: ZSI-1`，并出现 ZSI-1 SDK 初始化信息。
- 若出现 `body_type: None`，通常没有把 `body_type` 参数正确传入；请使用：
  `BODY=ZSI-1 ros2 launch pose_adapter pose_adapter.launch body_type:=ZSI-1`
- 若出现 `No module named 'unitree_sdk2py'` + `回退到 cmd_vel 模式`，表示 Go2 SDK 不可用，当前实际在用 `cmd_vel` 回退控制。

---

## 技术架构

### 数据流

```
┌─────────────────┐      queue(maxsize=1)      ┌─────────────────┐
│  Vision Thread  │ ──────────────────────→     │   Main Loop     │
│  (RTSP 拉流)    │   只保留最新一帧           │   (消费者)       │
│                 │                            │                 │
│  cv2.VideoCapture│                           │  检测→追踪→PnP │
└─────────────────┘                            └─────────────────┘
```

### 处理流程

1. **RTSP 拉流** - 生产者线程持续从 `rtsp://192.168.234.1:8554/test` 拉流
2. **目标检测** - YOLOv8 或备用检测
3. **目标追踪** - DeepSORT
4. **PnP 位姿解算** - 基于关键点的 6DoF 位姿估计
5. **运动控制** - PID 闭环控制（ZSI-1 SDK / Go2 SDK）
6. **OCR 识别** - 电表读数识别（可选）

---

## 机器狗类型

| 类型 | 控制方式 | SDK |
|------|----------|-----|
| **ZSI-1** | `move(vx, vy, yaw_rate)` | zsibot_sdk |
| **GO2** | `Move(vx, vy, vyaw)` | unitree_sdk2py |

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `BODY` | `ZSI-1` | 机器狗类型 |
| `ZSI_SDK_ROOT` | `../zsibot_sdk` | SDK 路径（与 PoseAdapter 同层） |
| `ZSI_LOCAL_IP` | `192.168.234.15` | 本地 IP |
| `ZSI_LOCAL_PORT` | `43988` | 本地端口 |
| `ZSI_DOG_IP` | `192.168.234.1` | 机器狗 IP |
| `MODEL_FILE` | `model/best.pt` | YOLO 模型路径（相对工程根目录） |

---

## 相机配置

- **图像来源**：RTSP 流（默认 `rtsp://192.168.234.1:8554/test`）
- **默认内参**：`src/calibrate/calibration_results/rtsp_camera_calib.yaml`

## 模型配置

- **默认模型路径**：`model/best.pt`（相对工程根目录）
- 启动时会校验模型文件是否存在；不存在将直接报错退出
- 也可通过环境变量覆盖：

```bash
MODEL_FILE=/home/nvidia/stephen/PoseAdapter/model/best.pt ./start.sh start
```

---

## 关键点提取方式

| 方式 | 说明 |
|------|------|
| `bbox` | 使用检测框的四个角点（默认） |
| `contour` | 轮廓检测 + 角点提取 |
| `keypoint` | 预留接口 |

```bash
keypoint_method:=contour ros2 launch pose_adapter pose_adapter.launch
```

---

## 运行测试

```bash
# 单元测试
python3.10 tests/test_pipeline.py

# 输出示例：
# === 测试 1: 检测器 ===
# 检测到 1 个目标 ✓
# ...
# 结果: 8 通过, 0 失败
```

---

## 依赖

- ROS2 Humble
- Python 3.10+
- OpenCV
- NumPy
- (可选) YOLOv8 / PaddleOCR

---

## 更新日志

### 2025-03-25

**RTMP 推流异常处理优化**

- 在 `_push_rtmp_frame` 调用处添加 try-except 异常捕获
- 新增 RTMP 调用失败的详细错误日志，便于排查推流异常问题
- 修改文件：`src/pose_adapter/src/pose_adapter/pose_adapter_node.py`

```python
# 修改前
self._push_rtmp_frame(debug_image)

# 修改后
try:
    self._push_rtmp_frame(debug_image)
except Exception as e:
    self.get_logger().error(f"[RTMP] 调用失败: {e}")
```

**推流架构说明**

```
PoseAdapter ──RTMP──> mediamtx ──RTSP──> 客户端
              rtmp://127.0.0.1:8554/pose   rtsp://127.0.0.1:8554/pose
```

- **RTMP**：用于推流（PoseAdapter → mediamtx）
- **RTSP**：用于拉流（客户端 → mediamtx）
- mediamtx 同时支持两种协议，做协议转换，这是业界标准做法
