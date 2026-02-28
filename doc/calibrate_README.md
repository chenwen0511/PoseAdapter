# 相机内参标定

宇树 Go2 前置相机的内参标定说明。标定结果用于 pose_adapter 的 PnP 位姿解算等模块。

---

## 一、一键标定完整步骤（小白适用）

本项目提供**无头模式**标定，无需 GUI 弹窗，在 Go2 上或 SSH 到 Go2 即可完成。按下面顺序操作即可。

### 1. 准备棋盘格

- **内角点数量**：默认 8×5（即 9×6 个方格、内部 8×5 个角点）。若你的棋盘格不同，后续命令中的 `--size` 需相应修改。
- **方格边长**：默认 25mm（0.025 米）。用尺子量一下实际边长，若不同需改 `--square` 参数。
- 打印棋盘格后贴在平整硬板上，确保无明显弯曲。

### 2. 安装依赖（在 Go2 上执行）

**2.1 加载 ROS2 环境**

```bash
source /opt/ros/foxy/setup.bash
```

若提示找不到 foxy，需先安装 ROS2 Foxy；若使用 Humble，则改为 `source /opt/ros/humble/setup.bash`。

**2.2 安装 Python 3.8（若系统默认不是 3.8）**

ROS2 Foxy 的 rclpy 依赖 Python 3.8。若运行时报 `No module named 'rclpy._rclpy'`，说明当前 Python 版本不匹配，需安装 3.8：

```bash
sudo apt update
sudo apt install -y python3.8 python3.8-venv
```

确认安装成功：

```bash
/usr/bin/python3.8 --version
```

应显示 `Python 3.8.x`。

**2.3 安装 Python 包**

```bash
pip3 install pyyaml opencv-python
# 或使用系统包（可选）
sudo apt install -y python3-opencv python3-yaml
```

**2.4 安装宇树 SDK（用于从相机取图）**

按宇树官方文档安装 `unitree_sdk2py`。若已安装过可跳过。

**2.5 安装相机标定依赖（ROS2 包，可选，仅 GUI 标定需要）**

```bash
sudo apt update
sudo apt install -y ros-foxy-camera-calibration
```

本流程使用一键无头标定，可不装；若要用 ROS 自带的 GUI 标定再装。

### 3. 三窗口操作流程

需要**同时打开 3 个终端**（可通过 SSH 开 3 个连接，或使用 tmux/screen 分屏）。

---

#### 终端 1：启动相机发布

在 Go2 上运行，将相机图像发布到 ROS2 话题：

```bash
# 加载 ROS2
source /opt/ros/foxy/setup.bash

# 进入标定目录
cd ~/stephen/PoseAdapter/src/calibrate
# 若项目在其他路径，改为你的实际路径

# 启动相机发布（保持运行，不要关）
python3.8 publish_go2_camera.py --no-network-interface
```

看到 `Go2 相机初始化成功，开始发布: /camera/image_raw` 即成功。此终端保持运行，直到标定完成。

---

#### 终端 2：验证图像是否在发布（可选）

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=1
ros2 topic hz /camera/image_raw
```

若有频率输出（如 `average rate: 30.000`），说明相机正常发布。按 `Ctrl+C` 退出即可，此终端可关掉。

---

#### 终端 3：一键标定

```bash
# 进入标定目录
cd ~/stephen/PoseAdapter/src/calibrate
# 若项目在其他路径，改为你的实际路径

# 执行一键标定
./calibrate_go2_onekey.sh
```

若脚本无执行权限，先执行：

```bash
chmod +x calibrate_go2_onekey.sh
```

脚本会提示：**请将棋盘格对准相机，并在约 40 秒内多移动棋盘格（左右、上下、远近、倾斜）**。

- 保持棋盘格在画面内，多换角度和距离。
- 脚本会自动采集约 20 张，然后自动标定并保存。
- 完成后结果保存到 `~/calibration_results/calib_result.yaml`。
- 脚本会调用 `read_calib_params.py` 打印内参预览。

**重要**：终端 1 的 `publish_go2_camera.py` 必须一直在跑，否则终端 3 收不到图像，无法采集。

### 4. 使用标定结果

标定生成的 `~/calibration_results/calib_result.yaml` 可直接用于 pose_adapter：

```bash
roslaunch pose_adapter pose_adapter.launch calib_file:=/home/unitree/calibration_results/calib_result.yaml
```

也可将 `calib_result.yaml` 复制到项目的 `src/pose_adapter/config/` 或 `src/calibrate/`，并命名为 `default_camera_calib.yaml`，作为默认内参使用。

---

## 二、常见问题

| 现象 | 处理 |
|------|------|
| `No module named 'rclpy._rclpy'` | 使用 `python3.8` 或 `/usr/bin/python3.8`，不要用 conda/venv 的 python3 |
| `No module named 'yaml'` | `pip3 install pyyaml` |
| `No module named 'cv2'` | `pip3 install opencv-python` 或 `sudo apt install python3-opencv` |
| `channel factory init error` / `eth0 does not match` | 使用 `--no-network-interface`，不要指定网卡 |
| `rmw_create_node: failed to create domain` | 脚本会自动设置 `ROS_DOMAIN_ID=1`；标定时确保 `export ROS_DOMAIN_ID=1` |
| `ros2 topic hz` 无输出 | 确认终端 1 在跑；标定脚本内已设 `ROS_DOMAIN_ID=1`，与相机节点一致 |
| 一直「等待图像…」或「已发布 0 帧」 | 相机/SDK 未返回图，检查 Go2 是否开机、网络、宇树 SDK 是否正常 |
| `Corrupt JPEG data` | 正常，已抑制该警告；若持续有「已发布 xxx 帧」则标定可进行 |

---

## 三、经验默认内参（仅用于快速测试）

在**尚未完成真机标定**但需要先跑通流程时，可使用下方经验值（**精度不保证，正式使用需真机标定**）：

- **1080P（1920×1080）**：fx=1000, fy=1000, cx=960, cy=540；畸变 k1~k5 = [0.1, -0.2, 0, 0, 0.05]
- **720P（1280×720）**：fx=700, fy=700, cx=640, cy=360；畸变同上

详见 `src/calibrate/default_camera_calib.yaml` 与 `src/pose_adapter/config/default_camera_calib.yaml`。完成真机标定后，会替换为实测内参。

---

## 四、关于 Go2 官方默认内参

宇树官网与文档中心**未公开** Go2 的默认相机内参与畸变系数。需准确参数时，请使用本项目标定流程在真机上标定，或向宇树技术支持/售后索取。

---

## 五、其他标定方式（可选）

- **GUI 标定**：使用 `calibrate_go2_camera.sh`，需在有显示器的环境运行，会弹出 ROS 标定窗口。Go2 上若 Qt 不弹窗，可改用本文档的一键无头流程。
- **手动 Python 标定**：直接运行 `calibrate_opencv.py`，加 `--headless` 为无头模式；不加则尝试弹窗，按 `s` 采集、`c` 标定、`q` 退出。
