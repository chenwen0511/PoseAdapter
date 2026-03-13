# RGB图像与Livox雷达点云融合方案

## 机器狗巡检：RGB相机 + Livox LiDAR 融合定位

### 1. 方案概述

本方案采用 **RGB相机 + Livox固态雷达** 融合的方式获取电表相对位置，替代原有的 PnP 方案。

- **RGB相机**：Go2 前置相机，用于 YOLO 目标检测
- **Livox LiDAR**：获取目标深度信息
- **融合**：在检测框 ROI 内提取点云计算质心，得到目标相对于机器狗的 3D 坐标
- **优势**：无需知道电表物理尺寸，直接获取深度，不受光照影响

---

### 2. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        感知层                                    │
├─────────────────────┬───────────────────────────────────────────┤
│   RGB 相机          │         Livox LiDAR                       │
│   (Go2前置)         │         (Mid-70 / Mid-360)               │
│   /camera/image_raw │         /livox/lidar                      │
└─────────┬───────────┴───────────────┬───────────────────────────┘
          │                           │
          ▼                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                      融合模块                                    │
│  1. 外参标定 (T_camera_lidar)                                   │
│  2. 点云投影到图像 → ROI 过滤                                   │
│  3. 目标3D质心计算                                              │
│  4. 坐标转换 (LiDAR → 机器人坐标系)                             │
└─────────────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────────────┐
│                      控制与执行 (与原方案相同)                    │
│  距离/角度控制 → cmd_vel → 机器人运动 → 触发 OCR               │
└─────────────────────────────────────────────────────────────────┘
```

---

### 3. 原理详解

#### 3.1 外参标定

相机与 LiDAR 之间的相对位姿关系，用一个 4×4 变换矩阵表示：

```
T_camera_lidar = [ R | t ]
                 [ 0 | 1 ]
```

- **R**: 3×3 旋转矩阵
- **t**: 3×1 平移向量

**标定方法**：
1. **棋盘格+角点**: 打印标定板，同时被相机和 LiDAR 看到
2. **特征提取**: 相机检测角点 (OpenCV)，LiDAR 获取平面点云
3. **PnP + ICP**: 联合优化求解外参

#### 3.2 点云投影与过滤

已知外参后，可将 LiDAR 点投影到图像坐标系：

```
u = K * (R * point_lidar + t)
```

其中 K 是相机内参矩阵。

**目标 ROI 提取流程**：

```
1. YOLO 检测电表 → bbox (x1, y1, x2, y2)
2. 将所有 LiDAR 点投影到图像
3. 过滤：保留投影点在 bbox 内部的点
4. 可选：按距离或高度过滤，去除地面/噪声点
```

#### 3.3 目标3D位置计算

```python
# 在 ROI 内提取点云
roi_points = points[inside_bbox(projected_points, bbox)]

# 计算质心（3D坐标）
target_3d = np.mean(roi_points, axis=0)  # [x, y, z] in LiDAR frame

# 转换到机器人坐标系（需要 T_lidar_robot）
target_robot = T_lidar_robot @ target_3d
```

#### 3.4 输出

| 输出 | 说明 |
|------|------|
| `distance` | 目标到机器人的欧氏距离 (m) |
| `yaw` | 水平偏航角 (°) |
| `pitch` | 俯仰角 (°) |
| `track_id` | DeepSORT 追踪 ID |

---

### 4. 开发实施指南

#### 4.1 硬件清单

| 组件 | 型号 | 说明 |
|------|------|------|
| 相机 | Go2 前置 1080P | 已有 |
| LiDAR | Livox Mid-70 或 Mid-360 | 需额外采购 |
| 算力板 | Jetson Orin Nano 8GB | 已有 |

#### 4.2 依赖安装

```bash
# Livox SDK
git clone https://github.com/Livox-SDK/livox_ros2_driver.git
cd livox_ros2_driver
rosdep install --from-paths src --ignore-src -r -y
colcon build

# 点云处理库
pip install open3d numpy
```

#### 4.3 外参标定流程

**方法一：手动标定（推荐入门）**

1. 准备棋盘格（8×5，25mm）
2. 同时采集相机图像和点云：
   ```bash
   # 相机
   rosrun pose_adapter publish_go2_camera.py
   
   # LiDAR
   roslaunch livox_ros2_driver livox_lidar.launch
   ```
3. 使用标定工具（如 Kalibr 或自研脚本）计算外参
4. 保存到 `config/extrinsic.yaml`

**方法二：使用现成工具**

推荐使用 **Kalibr** 或 **livox_camera_calib** 进行标定。

#### 4.4 代码修改点

| 模块 | 原方案 | 新方案 |
|------|--------|--------|
| 检测 | YOLOv8 bbox | YOLOv8 bbox (不变) |
| 追踪 | DeepSORT | DeepSORT (不变) |
| 位姿 | PnP 计算 | 点云质心计算 |
| 控制 | cmd_vel 闭环 | cmd_vel 闭环 (不变) |

**核心代码示例**：

```python
import numpy as np
import open3d as o3d

class LidarCameraFusion:
    def __init__(self, extrinsic_file, camera_K):
        # 外参: LiDAR -> Camera
        self.T_lidar_to_cam = np.load(extrinsic_file)
        # 相机内参
        self.K = camera_K
        
    def project_points_to_image(self, points):
        """将点云投影到图像坐标"""
        points_h = np.hstack([points, np.ones((points.shape[0], 1))])
        points_cam = (self.T_lidar_to_cam @ points_h.T).T
        points_img = (self.K @ points_cam[:, :3].T).T
        points_2d = points_img[:, :2] / points_img[:, 2:]
        return points_2d
    
    def get_target_position(self, points_3d, bbox):
        """从ROI点云计算目标3D位置"""
        # 过滤在bbox内的点
        mask = (points_3d[:, 0] >= bbox[0]) & (points_3d[:, 0] <= bbox[2]) & \
               (points_3d[:, 1] >= bbox[1]) & (points_3d[:, 1] <= bbox[3])
        roi_points = points_3d[mask]
        
        if len(roi_points) == 0:
            return None
            
        # 计算质心
        centroid = np.mean(roi_points, axis=0)
        distance = np.linalg.norm(centroid)
        
        # 计算偏航角
        yaw = np.arctan2(centroid[1], centroid[0])
        
        return {
            'position': centroid,  # [x, y, z]
            'distance': distance,  # 米
            'yaw': np.degrees(yaw),
            'pitch': np.degrees(np.arctan2(centroid[2], np.sqrt(centroid[0]**2 + centroid[1]**2)))
        }
```

#### 4.5 融合配置

```yaml
# config/fusion.yaml
lidar:
  topic: "/livox/lidar"
  frame_id: "livox_frame"
  
camera:
  topic: "/camera/image_raw"
  frame_id: "camera_frame"
  intrinsic: "config/camera_calib.yaml"
  
extrinsic:
  file: "config/extrinsic.yaml"
  
fusion:
  min_points_in_roi: 10    # ROI内最少点数
  max_distance: 10.0       # 最大有效距离 (m)
  point_accumulate_frames: 3  # 点云累积帧数
```

#### 4.6 时间同步

LiDAR 和相机的帧率不同，需要同步：

```python
import message_filters

image_sub = message_filters.Subscriber('/camera/image_raw', Image)
lidar_sub = message_filters.Subscriber('/livox/lidar', PointCloud2)

sync = message_filters.ApproximateTimeSynchronizer([image_sub, lidar_sub], 10, 0.1)
sync.registerCallback(self.fusion_callback)
```

---

### 5. 数据流程

```
┌──────────────┐     ┌──────────────┐
│  RGB 图像    │     │  点云数据    │
│  1920×1080   │     │  ~2.4万点/秒 │
└──────┬───────┘     └──────┬───────┘
       │                     │
       ▼                     ▼
┌──────────────────────────────────┐
│      YOLO 检测 (bbox)            │
└──────────────┬───────────────────┘
               │
               ▼  bbox
┌──────────────────────────────────┐
│   点云投影到图像 + ROI 过滤       │
│   提取 bbox 内的点云              │
└──────────────┬───────────────────┘
               │
               ▼
┌──────────────────────────────────┐
│   质心计算 → 3D 坐标              │
│   坐标变换 → 机器人坐标系         │
└──────────────┬───────────────────┘
               │
               ▼
┌──────────────────────────────────┐
│   输出: distance, yaw, pitch     │
│   (后续控制逻辑不变)             │
└──────────────────────────────────┘
```

---

### 6. 关键参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| ROI 内最少点数 | 10 | 过滤噪声点 |
| 点云累积帧数 | 1-3 | 远距离可增加 |
| 最大有效距离 | 10m | Livox Mid-70 典型射程 |
| 同步时间窗口 | 100ms | 近似同步容差 |

---

### 7. 注意事项

1. **外参标定是核心**：标定精度直接影响定位精度，建议定期校验
2. **点云稀疏处理**：
   - 近距离：点云稠密，单帧即可
   - 远距离：多帧累积，或使用非重复扫描模式
3. **噪声过滤**：
   - 过滤地面点（高度阈值）
   - 过滤异常值（统计滤波）
4. **Livox 选型**：
   - Mid-70: 70°×70° FOV，适合近距离
   - Mid-360: 360°×59° FOV，适合大范围

---

### 8. 开发里程碑

| 阶段 | 内容 | 预计时间 |
|------|------|----------|
| 1 | 外参标定 (相机-LiDAR) | 1-2天 |
| 2 | 点云订阅与投影 | 2天 |
| 3 | ROI 提取与质心计算 | 2天 |
| 4 | 融合模块集成 | 2天 |
| 5 | 控制闭环联调 | 2天 |
| 6 | 整体测试与优化 | 2天 |

---

### 9. 参考资料

- [Livox ROS Driver](https://github.com/Livox-SDK/livox_ros2_driver)
- [Kalibr 标定工具](https://github.com/ethz-asl/kalibr)
- [Open3D 点云处理](https://github.com/isl-org/Open3D)
- [Livox-Camera Calib](https://github.com/Livox-SDK/livox_camera_calib)

---

### 10. 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-03-13 | 初版 |
