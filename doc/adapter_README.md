# 机器狗巡检：自适应位姿读取电表读数

机器狗巡检时，要稳定、清晰地采集电表读数，需要自动调整与电表的**距离**和**拍摄角度**。本方案仅依赖 **Go2 前置相机 + 机身运动控制**（无云台），通过**对象检测 + 物体追踪 + 位姿控制闭环**，让机器狗通过前进/后退/转向自动对准电表并保持合适视距，再触发 OCR 完成读数采集。

---

## 核心方案总览

- **目标检测**：用 **YOLOv8** 识别电表，输出 bbox 与置信度，过滤非电表目标。
- **目标追踪**：用 **DeepSORT** 关联帧间电表，抗遮挡、保持跟踪连续性。
- **位姿估计**：用 **PnP**，通过电表关键点（如四个角）与**已知电表物理尺寸**解算相对**距离、偏航角、俯仰角**（SfM 可选，用于多帧尺度/平滑）。
- **控制闭环**：以电表在画面中**占比 60%–70%**、**中心偏差 <5%** 为目标，通过机身**前进/后退/转向**实时调节距离与朝向，确保对焦清晰。
- **读数采集**：达标后触发 **OCR**（如 PaddleOCR / Tesseract），输出电表读数。

---

## 实现步骤（宇树 Go2 + ROS2）

### 1. 感知配置

- 部署 **YOLOv8**，训练/微调电表检测模型，输出 bbox 与类别。
- 部署 **DeepSORT**，关联 ID，维持遮挡时的跟踪。
- 用 **OpenCV** 在检测框内提取电表角点（或取 bbox 四角），配合**电表实际尺寸**（宽/高，米）构建 3D 点，通过 **PnP** 解算相对位姿（需相机内参与畸变，参见 `src/calibrate/`）。

### 2. 控制逻辑（伪代码）

```python
# 位姿解算
ret, rvec, tvec = cv2.solvePnP(3d_points, 2d_points, K, dist)
distance = np.linalg.norm(tvec)  # 距离
yaw = calculate_yaw(rvec)        # 偏航角
pitch = calculate_pitch(rvec)   # 俯仰角

# 距离控制（目标 1.5–2.0 m）
if distance < 1.5:
    cmd_vel.linear.x = -0.2   # 后退
elif distance > 2.0:
    cmd_vel.linear.x = 0.2    # 前进
else:
    cmd_vel.linear.x = 0

# 角度控制（目标偏航在 ±2° 内；无云台，俯仰依赖机身与电表相对高度）
if abs(yaw) > 2:
    cmd_vel.angular.z = -yaw * 0.1   # 转向对准
# 俯仰不可单独调节，可通过前进/后退微调距离或依赖巡检路径使电表落在画面中心
```

### 3. 部署与优化

- **硬件**：Go2 标配 **Jetson Orin Nano 8GB** 算力板 + **前置 1080P 相机**（固定安装，无云台）。在 8GB 显存约束下，采用轻量模型（如 YOLOv8n）可满足 30+ FPS 实时检测；若显存紧张可降分辨率（如 720P）或减小模型输入尺寸。
- **通信**：ROS2 话题传输检测/追踪/位姿/控制指令，确保低延迟。
- **鲁棒性**：跟踪丢失时启用**重检测**、强光/弱光使用**动态阈值**、位姿使用**多帧平滑**防抖动。
- **标定**：提前完成**相机内参标定**与**手眼标定**，保证位姿精度（内参标定见 `src/calibrate/` 与 `doc/calibrate_README.md`）。

---

## 关键参数与阈值

| 项 | 目标值 | 说明 |
|----|--------|------|
| 画面占比 | 60%–70% | 电表 bbox 宽度 / 图像宽度，保证细节清晰 |
| 中心偏差 | <5% | bbox 中心与图像中心的偏差，减少透视畸变 |
| 距离范围 | 1.5–2.0 m | 根据镜头 FOV 调整，以清晰读数为准 |
| 角度范围 | ±2°（偏航） | 无云台，偏航通过转向控制；俯仰由前置相机固定安装决定，可通过走位间接改善 |

---

## 部署清单

| 模块 | 选型 | 说明 |
|------|------|------|
| 算力板 | Jetson Orin Nano 8GB | Go2 标配，满足检测+追踪+OCR 实时推理 |
| 检测 | YOLOv8n | 轻量模型，适配 8GB 显存，目标 30+ FPS |
| 追踪 | DeepSORT | ID 关联，抗遮挡 |
| 位姿 | PnP | 关键点 + 电表尺寸解算相对位姿 |
| 控制 | ROS2 + Go2 SDK | 机身运动控制（前进/后退/转向） |
| OCR | PaddleOCR | 读数识别（可选用轻量配置以节省显存） |

---

## 构建 pose_adapter 包

pose_adapter 已支持通过 **catkin** 正常编译。在启动节点前，需先完成工作空间构建。

### 构建步骤

```bash
cd /path/to/PoseAdapter
source /opt/ros/noetic/setup.bash
catkin_make
```

### 依赖说明

若使用 **conda/miniconda** 的 Python 环境，系统自带的 `python3-empy` 可能不可用，需通过 pip 安装以下包：

```bash
pip install empy catkin_pkg
```

> 若使用系统 Python，可执行 `sudo apt install python3-empy`，通常 `catkin_pkg` 已随 ROS 安装。

### 验证构建

构建成功后，应看到 `pose_adapter` 包处理完成，无错误输出。

---

## 启动 Adapter 节点

Adapter 节点位于 **`src/pose_adapter/`**，基于 **ROS1**。

### 方式一：使用启动脚本（推荐）

项目根目录提供 **`start.sh`**，支持 start/stop/restart/status：

```bash
cd /path/to/PoseAdapter
chmod +x start.sh
./start.sh start
# 使用自定义标定：CALIB_FILE=/path/to/calib_result.yaml ./start.sh start
./start.sh stop
./start.sh restart
./start.sh status
```

### 方式二：手动 roslaunch

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch pose_adapter pose_adapter.launch calib_file:=/path/to/calib_result.yaml
```

**依赖安装**：`sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport`

### 图像来源（默认从 topic）

pose_adapter **默认从话题 `/camera/image_raw` 获取图像**，与 `src/calibrate` 标定脚本约定一致。需有其他节点向该话题发布相机 raw 图。

### 主要 launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `calib_file` | 见 launch | 相机标定 yaml 路径 |
| `camera_image_topic` | `/camera/image_raw` | 图像来源话题，与 calibrate 一致 |
| `use_go2_camera` | false | 是否用 Go2 SDK 取流 |
| `network_interface` | 空 | Unitree SDK 网卡，留空自动检测 |
| `loop_hz` | 5 | 主循环频率（低算力建议 3–5） |
| `show_debug_image` | false | 是否启动 image_view |

---

## 常见问题与对策

- **遮挡/丢失**：启用**重检测机制**，丢失后快速重新识别。
- **光照不均**：使用 **CLAHE** 增强，或切换**红外补光**。
- **位姿抖动**：对位姿做**滑动平均滤波**（窗口 3–5 帧）。

---

## 方案可行性检查（后续开发依据）

以下在**技术、硬件、前提条件**三方面做一致性检查，确保后续细节开发不偏离方案。

### 结论：方案整体可行，需落实若干前置与约束

| 维度 | 评估 | 说明 |
|------|------|------|
| 检测+追踪 | ✅ 可行 | YOLOv8n + DeepSORT 在 Orin Nano 8GB 上可达到 30+ FPS，电表单类检测数据易采集、易微调。 |
| 位姿解算 | ✅ 可行 | PnP 依赖**已知电表物理尺寸**（宽/高，米）才能得到真实距离；2D 点可用 bbox 四角或角点检测，需与 3D 点一一对应。 |
| 控制闭环 | ✅ 可行 | 仅前进/后退/转向，无云台，与 Go2 能力匹配；需确认 ROS2 `cmd_vel` 或 Unitree SDK 接口与机型权限（部分需 EDU）。 |
| 硬件与算力 | ✅ 可行 | Orin Nano 8GB + 前置 1080P 可支撑检测+追踪+轻量 OCR；显存紧张时降分辨率或减小检测输入。 |
| 相机与标定 | ✅ 可行 | 内参标定流程已具备（`src/calibrate/`）；无云台时「手眼」可简化为相机与机身同向或固定俯仰角，后续可再补外参。 |

### 必须明确的前置条件

1. **电表 3D 尺寸**  
   PnP 得到的是**有尺度**的 tvec（米），前提是 3D 点用真实尺寸。需在配置或文档中约定：按电表型号查表、或现场测量一次录入（宽/高，单位：米）。

2. **2D 关键点来源**  
   YOLO 只出 bbox。二选一或组合：  
   - 用 **bbox 四角** 作为 2D 点，3D 点取电表矩形四角（依赖尺寸）；  
   - 或在 bbox 内做**角点/关键点检测**，对应到电表上已知 3D 点（更准但需标定或标点）。  
   实现时需统一约定「2D–3D 对应关系」和单位。

3. **俯仰与安装高度**  
   无云台时俯仰不可调，电表若高于相机视线，需靠**巡检路径或机身姿态**（如站高）使电表进入视野。建议在需求中约定：电表安装高度范围、是否允许站高/抬头，避免方案与现场不符。

4. **图像与驱动**  
   确认 Go2 前置相机在 Orin Nano 上的**驱动与 ROS2 话题**（如 `/camera/image_raw`、`camera_info`）已打通，且分辨率/帧率与标定一致。

### 风险与缓解

| 风险 | 缓解 |
|------|------|
| 电表尺寸未知或型号混杂 | 配置表按型号维护尺寸；或首次巡检测量录入，后续复用。 |
| bbox 四角代替角点导致 PnP 误差大 | 先用 bbox 四角跑通流程，再迭代为角点检测或关键点模型提升精度。 |
| 多块电表同屏 | 策略选一（如距离最近/最居中）再对该目标做闭环控制与 OCR。 |
| 控制振荡或响应慢 | 距离/偏航做死区与限幅，角速度增益调参；位姿多帧平滑（如 3–5 帧）。 |
| 光照导致 OCR 失败 | 已列 CLAHE/红外补光；可加「不合格不保存、重拍」逻辑。 |

### 效果预期的适用条件

- **响应时间 <200 ms**：在 30 FPS、控制律调好、无严重丢帧前提下可达。  
- **距离误差 <5 cm、角度 <1°**：依赖**内参标定良好 + 电表尺寸准确 + 2D 点稳定**，若用 bbox 四角则适当放宽预期。  
- **读数成功率 >95%**：在正常光照、对焦清晰、表盘类型在 OCR 训练范围内成立。

### 建议开发顺序

1. **相机与标定**：打通前置相机话题，完成内参标定并写入配置（可先用 `src/calibrate/` 或 doc 中经验值联调）。  
2. **电表尺寸配置**：确定 3D 点定义方式（如 bbox 四角对应矩形），建立电表型号→尺寸（米）配置。  
3. **检测 + 追踪**：部署 YOLOv8n 电表检测与 DeepSORT，输出稳定 bbox/ID。  
4. **PnP 位姿**：用 2D 点（先 bbox 四角）+ 3D 点 + K/dist 解算距离与偏航，多帧平滑。  
5. **控制闭环**：按距离/偏航与目标占比、中心偏差生成 cmd_vel，调参并加死区/限幅。  
6. **OCR 与触发**：达标后截 ROI 调用 PaddleOCR，记录读数与置信度。

---

## 效果预期

- **响应时间**：<200 ms，快速调节到位。
- **定位精度**：距离误差 <5 cm，角度误差 <1°（依赖标定与电表尺寸）。
- **读数成功率**：正常光照下 >95%。
