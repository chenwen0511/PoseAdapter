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
| 启动 ZSI-1 | `BODY=ZSI-1 ros2 launch pose_adapter pose_adapter.launch` |
| 启动 GO2 | `BODY=GO2 ros2 launch pose_adapter pose_adapter.launch` |
| 运行单元测试 | `python3 tests/test_pipeline.py` |

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
| `ZSI_SDK_ROOT` | `/home/stephen/.openclaw/workspace/zsibot_sdk` | SDK 路径 |
| `ZSI_LOCAL_IP` | `192.168.1.100` | 本地 IP |
| `ZSI_LOCAL_PORT` | `43988` | 本地端口 |
| `ZSI_DOG_IP` | `192.168.234.1` | 机器狗 IP |

---

## 相机配置

- **图像来源**：RTSP 流（默认 `rtsp://192.168.234.1:8554/test`）
- **默认内参**：`src/calibrate/calibration_results/rtsp_camera_calib.yaml`

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
python3 tests/test_pipeline.py

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
