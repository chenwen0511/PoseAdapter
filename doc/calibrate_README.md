<<<<<<< HEAD


# 相机内参标定

使用 ROS2 `camera_calibration` 对宇树 Go2 相机做内参标定，并用 `read_calib_params.py` 读出内参矩阵与畸变系数，供后续代码使用。

### 经验默认内参（仅用于快速测试）

在**尚未完成真机标定**但需要先跑通流程时，可以使用下面的经验值作为占位参数（**精度不保证，正式使用必须用真实标定结果替换**）：

- **1080P 分辨率（1920×1080）**

  | 内参矩阵 K（fx, fy, cx, cy） |  |
  |---|---|
  | **fx** | 1000. |
  | **fy** | 1000. |
  | **cx** | 960. |
  | **cy** | 540. |

  | 畸变系数（5 参数） | k1 | k2 | p1 | p2 | k3 |
  |---|---|---|---|---|---|
  | 值 | 0.1 | -0.2 | 0. | 0. | 0.05 |

- **720P 分辨率（1280×720）**

  | 内参矩阵 K（fx, fy, cx, cy） |  |
  |---|---|
  | **fx** | 700. |
  | **fy** | 700. |
  | **cx** | 640. |
  | **cy** | 360. |

  | 畸变系数（5 参数） | k1 | k2 | p1 | p2 | k3 |
  |---|---|---|---|---|---|
  | 值 | 0.1 | -0.2 | 0. | 0. | 0.05 |

> **说明**：以上参数只作为代码调试与流程联调时的**参考默认值**，不要在需要精确测量、投影、定位的场景中长期使用。

## 关于 Go2 官方默认内参

经查 **宇树官网与文档中心**：

- **[unitree.com/go2](https://www.unitree.com/go2)** 产品页仅注明「HD Wide-angle Camera」，未提供相机内参或畸变系数。
- **[support.unitree.com](https://support.unitree.com)** 文档中心中，Go2 相关页面未公开默认内参/畸变；且部分页面存在加载异常，无法从当前公开入口找到标定文档。
- 官方 **Unitree Camera SDK**（GitHub: unitreerobotics/UnitreecameraSDK）面向 **GO1** 双目，标定通过设备上运行 `example_getCalibParamsFile` 获取，未提供 Go2 的通用默认 K/畸变。
- 社区资料中常见 **1280×720、120° 广角** 的规格描述，但**无宇树官方给出的默认内参矩阵与畸变系数**。

**结论**：宇树**未在官网或公开文档中提供 Go2 的默认相机内参与畸变**。如需准确参数，请使用项目内 **`src/calibrate/`** 的标定流程在真机上标定，或向宇树技术支持/售后索取标定数据。

## 使用步骤

标定脚本与工具位于 **`src/calibrate/`**，建议在该目录下执行以下步骤。

1. **修改棋盘格参数**  
   编辑 `src/calibrate/calibrate_go2_camera.sh` 中的 `CHESSBOARD_SIZE`（内角点数量，如 9×6 棋盘填 `8x5`）和 `SQUARE_SIZE`（方格边长，单位米，如 25mm 填 `0.025`）。

2. **执行标定脚本**  
   ```bash
   cd src/calibrate
   chmod +x calibrate_go2_camera.sh
   ./calibrate_go2_camera.sh
   ```  
   按界面提示移动棋盘格，待 CALIBRATE 可用后点击标定，再 SAVE，最后 COMMIT 退出。  
   标定数据默认会写入 **`/tmp/calibrationdata.tar.gz`**，需自行解压并用 `camera_calibration_parsers` 转为 yaml，或将终端打印的 K/D 整理成 yaml。

3. **读取内参**  
   将得到的 `calib_params.yaml` 放到例如 `~/calibration_results/` 后执行：  
   ```bash
   python3 src/calibrate/read_calib_params.py -f ~/calibration_results/calib_params.yaml
   ```  
=======


# 相机内参标定

使用 ROS2 `camera_calibration` 对宇树 Go2 相机做内参标定，并用 `read_calib_params.py` 读出内参矩阵与畸变系数，供后续代码使用。

### 经验默认内参（仅用于快速测试）

在**尚未完成真机标定**但需要先跑通流程时，可以使用下面的经验值作为占位参数（**精度不保证，正式使用必须用真实标定结果替换**）：

- **1080P 分辨率（1920×1080）**

  | 内参矩阵 K（fx, fy, cx, cy） |  |
  |---|---|
  | **fx** | 1000. |
  | **fy** | 1000. |
  | **cx** | 960. |
  | **cy** | 540. |

  | 畸变系数（5 参数） | k1 | k2 | p1 | p2 | k3 |
  |---|---|---|---|---|---|
  | 值 | 0.1 | -0.2 | 0. | 0. | 0.05 |

- **720P 分辨率（1280×720）**

  | 内参矩阵 K（fx, fy, cx, cy） |  |
  |---|---|
  | **fx** | 700. |
  | **fy** | 700. |
  | **cx** | 640. |
  | **cy** | 360. |

  | 畸变系数（5 参数） | k1 | k2 | p1 | p2 | k3 |
  |---|---|---|---|---|---|
  | 值 | 0.1 | -0.2 | 0. | 0. | 0.05 |

> **说明**：以上参数只作为代码调试与流程联调时的**参考默认值**，不要在需要精确测量、投影、定位的场景中长期使用。

## 关于 Go2 官方默认内参

经查 **宇树官网与文档中心**：

- **[unitree.com/go2](https://www.unitree.com/go2)** 产品页仅注明「HD Wide-angle Camera」，未提供相机内参或畸变系数。
- **[support.unitree.com](https://support.unitree.com)** 文档中心中，Go2 相关页面未公开默认内参/畸变；且部分页面存在加载异常，无法从当前公开入口找到标定文档。
- 官方 **Unitree Camera SDK**（GitHub: unitreerobotics/UnitreecameraSDK）面向 **GO1** 双目，标定通过设备上运行 `example_getCalibParamsFile` 获取，未提供 Go2 的通用默认 K/畸变。
- 社区资料中常见 **1280×720、120° 广角** 的规格描述，但**无宇树官方给出的默认内参矩阵与畸变系数**。

**结论**：宇树**未在官网或公开文档中提供 Go2 的默认相机内参与畸变**。如需准确参数，请使用项目内 **`src/calibrate/`** 的标定流程在真机上标定，或向宇树技术支持/售后索取标定数据。

## 使用步骤

标定脚本与工具位于 **`src/calibrate/`**，建议在该目录下执行以下步骤。

1. **修改棋盘格参数**  
   编辑 `src/calibrate/calibrate_go2_camera.sh` 中的 `CHESSBOARD_SIZE`（内角点数量，如 9×6 棋盘填 `8x5`）和 `SQUARE_SIZE`（方格边长，单位米，如 25mm 填 `0.025`）。

2. **执行标定脚本**  
   ```bash
   cd src/calibrate
   chmod +x calibrate_go2_camera.sh
   ./calibrate_go2_camera.sh
   ```  
   按界面提示移动棋盘格，待 CALIBRATE 可用后点击标定，再 SAVE，最后 COMMIT 退出。  
   标定数据默认会写入 **`/tmp/calibrationdata.tar.gz`**，需自行解压并用 `camera_calibration_parsers` 转为 yaml，或将终端打印的 K/D 整理成 yaml。

3. **读取内参**  
   将得到的 `calib_params.yaml` 放到例如 `~/calibration_results/` 后执行：  
   ```bash
   python3 src/calibrate/read_calib_params.py -f ~/calibration_results/calib_params.yaml
   ```  
>>>>>>> a5a410c (add default camera calib yaml)
   输出可直接复制到工程中的内参配置（如 `self.K`、`self.dist_coeffs`）。