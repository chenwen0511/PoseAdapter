# 相机内参标定与校验（ROS2 Humble + RTSP）

本文档对应 `src/calibrate/` 的最新实现：  
**RTSP 拉流 -> 发布 `/camera/image_raw` -> OpenCV 标定 -> YAML 内参校验**。

默认 RTSP 地址：

`rtsp://192.168.234.1:8554/test`

---

## 1. 环境与依赖

```bash
source /opt/ros/humble/setup.bash
pip3 install pyyaml opencv-python
```

说明：
- 当前流程已统一为 **ROS2 Humble**，不再依赖 ROS1/noetic、ros1_bridge 或 Unitree SDK 取流。
- 如你用系统包，也可安装 `python3-opencv`、`python3-yaml`。

---

## 2. 一键无头标定（推荐）

```bash
cd ~/stephen/PoseAdapter/src/calibrate
chmod +x calibrate_go2_onekey.sh
./calibrate_go2_onekey.sh
```

脚本会自动执行：
1. 启动 RTSP 拉流发布节点（`publish_go2_camera.py`）
2. 检查 `/camera/image_raw` 是否收到图像
3. 运行无头标定并输出 YAML

默认输出路径：

`~/stephen/PoseAdapter/src/calibrate/rtsp_camera_calib.yaml`（即 `src/calibrate/` 下）

---

## 3. 参数可配置（重点）

可以通过环境变量覆盖默认值：

```bash
RTSP_URL=rtsp://192.168.1.10:8554/live \
CAMERA_TOPIC=/camera/image_raw \
CHESSBOARD_SIZE=8x5 \
SQUARE_SIZE=0.025 \
CAMERA_NAME=rtsp_camera \
SAVE_PATH=$HOME/stephen/PoseAdapter/src/calibrate \
FPS=15 \
./calibrate_go2_onekey.sh
```

其中：
- `RTSP_URL`：RTSP 拉流地址（默认 `rtsp://192.168.234.1:8554/test`）
- `CHESSBOARD_SIZE`：棋盘格内角点（如 `8x5`）
- `SQUARE_SIZE`：方格边长（米）

---

## 4. GUI 标定（可选）

有显示器时可使用 GUI 采集：

```bash
cd ~/stephen/PoseAdapter/src/calibrate
chmod +x calibrate_go2_camera.sh
./calibrate_go2_camera.sh
```

窗口按键：
- `s` 采集一帧
- `c` 标定并保存
- `q` 退出

---

## 5. 内参校验

标定完成后，使用以下命令校验并打印可直接复制的 `K` 与 `dist_coeffs`：

```bash
python3 read_calib_params.py --file ~/stephen/PoseAdapter/src/calibrate/rtsp_camera_calib.yaml
```

校验要点：
- `image_width / image_height` 是否与实际流分辨率一致
- `camera_matrix.data` 是否为 9 个值
- `distortion_coefficients.data` 是否为 5 个值（plumb_bob）

---

## 6. 手动分步流程（调试用）

### 6.1 仅启动 RTSP 发布

```bash
cd ~/stephen/PoseAdapter/src/calibrate
python3 publish_go2_camera.py \
  --rtsp-url rtsp://192.168.234.1:8554/test \
  --topic /camera/image_raw \
  --fps 15
```

### 6.2 检查话题是否有图像

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /camera/image_raw --once
```

### 6.3 运行标定

无头：

```bash
python3 calibrate_opencv.py \
  --topic /camera/image_raw \
  --size 8x5 \
  --square 0.025 \
  --camera-name rtsp_camera \
  --out ~/stephen/PoseAdapter/src/calibrate/rtsp_camera_calib.yaml \
  --headless
```

---

## 7. 常见问题

| 现象 | 处理 |
|------|------|
| 收不到图像消息 | 检查 `RTSP_URL` 是否可达、端口是否开放、流是否在线 |
| `无法打开 RTSP 流` | 确认地址/用户名密码（如有）正确，确认设备网络连通 |
| `No module named cv_bridge` | 当前脚本已移除 `cv_bridge` 依赖，拉取最新代码后重试 |
| `No module named cv2` | `pip3 install opencv-python` |
| `No module named yaml` | `pip3 install pyyaml` |
| `No module named rclpy` | 确认已 `source /opt/ros/humble/setup.bash` |

---

## 8. 与 pose_adapter 对接

将标定生成的 YAML 作为 `calib_file` 使用即可，例如：

```bash
roslaunch pose_adapter pose_adapter.launch calib_file:=/home/nvidia/stephen/PoseAdapter/src/calibrate/rtsp_camera_calib.yaml
```
