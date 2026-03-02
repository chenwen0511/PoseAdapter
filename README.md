# PoseAdapter

宇树 Go2 机器狗巡检场景下，基于**前置相机 + 机身运动**的位姿自适应与电表读数采集方案（检测 + 追踪 + PnP 位姿 + 控制闭环 + OCR）。

---

## 目录规划

```
PoseAdapter/
├── README.md           # 本文件，项目与目录说明
├── start.sh            # 启动脚本（start/stop/restart/status）
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
| 启动 Adapter 节点 | `./start.sh start` 或 [doc/adapter_README.md#启动-adapter-节点](doc/adapter_README.md#启动-adapter-节点) |
| 做相机内参标定、查经验内参 | [doc/calibrate_README.md](doc/calibrate_README.md) |
| 运行标定脚本、读取标定结果 | `src/calibrate/` |
| 运行单元测试 | `pytest tests/ -v`（见 [tests/README.md](tests/README.md)） |
