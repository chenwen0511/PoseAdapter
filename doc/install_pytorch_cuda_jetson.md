# Orin Nano / Jetson 安装 PyTorch 与 torchvision（带 CUDA）

在 Orin Nano 等 Jetson 上让 PoseAdapter 的 YOLO 使用 GPU，需要安装 **NVIDIA 为 JetPack 提供的 PyTorch wheel**（带 CUDA），并搭配**与当前 torch 匹配的 torchvision**。不能直接 `pip install torch torchvision`（多为 CPU 版或 ABI 不匹配）。

---

## 当前推荐组合（本机实测）

| 组件 | 版本 | 说明 |
|------|------|------|
| Python | **3.8.x**（如 3.8.20） | 与 conda 环境一致，决定选 cp38 的 wheel |
| torch | **2.0.0+nv23.5** | NVIDIA JetPack 5.1 专用 wheel（带 CUDA） |
| torchvision | **0.15.2** | 与 torch 2.0 匹配，Jetson 上建议从源码编译 |

验证：`pip list | grep torch` 应看到 `torch 2.0.0+nv23.5` 与 `torchvision 0.15.2`。

---

## 1. 环境确认

在板子上执行：

```bash
# JetPack / L4T 版本（如 R35.x = JP 5.1，R36.x = JP 6.x）
cat /etc/nv_tegra_release

# CUDA 版本（有 nvcc 即可；nvidia-smi 部分镜像未装可忽略）
nvcc --version

# 用于跑 pose_adapter 的 Python 版本（决定选 cp38 还是 cp310 的 wheel）
python3 --version
```

记下：**CUDA 版本**（如 11.4）、**JetPack 主版本**（5.1→v511，6.1→v61）、**Python 版本**（3.8→cp38，3.10→cp310）。

---

## 2. 安装 PyTorch（NVIDIA wheel）

**必须在跑 pose_adapter 的同一环境中安装**（如 `conda activate task`）。

### 2.1 依赖

```bash
sudo apt-get -y update
sudo apt-get install -y python3-pip libopenblas-dev
```

### 2.2 JetPack 5.1.x（CUDA 11.4，Python 3.8）

安装后 `pip list` 中可能显示为 **torch 2.0.0+nv23.5**（与 wheel 文件名中的 nv23.05 等价）。

```bash
conda activate task
pip3 install --upgrade pip
pip3 install 'numpy<2'
pip3 install --no-cache-dir \
  https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
```

### 2.3 JetPack 6.1 / 6.2（Python 3.10）

```bash
conda activate task
pip3 install --upgrade pip
pip3 install 'numpy<2'
# 若 PyTorch 2.4+ 需先装 cuSPARSELt，见 NVIDIA 文档
pip3 install --no-cache-dir \
  https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
```

### 2.4 验证 PyTorch 与 CUDA

```bash
python3 -c "
import torch
print('PyTorch:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
print('Device count:', torch.cuda.device_count())
"
```

期望输出中 `CUDA available: True`。

---

## 3. 安装 torchvision（与 torch 2.0 匹配）

torch 2.0 需搭配 **torchvision 0.15.x**。在 Jetson 上 **pip 的预编译 torchvision 不可用**（针对标准 PyTorch，与 NVIDIA Jetson 专用 torch ABI 不一致，会报 **Couldn't load custom C++ ops**），**必须用 git clone 从源码编译安装**。

### 3.1 从源码编译 torchvision

在**同一 conda 环境**下执行，使 C++ 扩展与当前安装的 torch 一致。

#### 编译前必读：注意内存，否则会编译失败

Orin Nano 内存有限，**默认多线程编译会占满内存，导致 `cc1plus` 被系统 Kill**（报错：`fatal error: Killed signal terminated program cc1plus`）。必须：

1. **始终设置 `export MAX_JOBS=1`**（只开 1 个编译任务，降低峰值内存）。
2. 编译前**关闭浏览器、其它大程序**，只保留终端。
3. 若仍被 Kill，可临时增加 2GB swap 再编译：
   ```bash
   sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile
   sudo mkswap /swapfile && sudo swapon /swapfile
   ```
4. 编译耗时约 **20–40 分钟** 属正常，不要中断。

#### 编译步骤（Python 3.8 + torch 2.0.0+nv23.5 → torchvision 0.15.2）

```bash
conda activate task
pip3 uninstall -y torchvision   # 若之前用 pip 装过，先卸掉

sudo apt-get install -y \
  libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev \
  libavcodec-dev libavformat-dev libswscale-dev

cd /tmp
git clone --depth 1 --branch v0.15.2 https://github.com/pytorch/vision torchvision_src
cd torchvision_src

export BUILD_VERSION=0.15.2
# 必须：限制并行编译，避免内存不足被 Kill（否则会编译失败）
export MAX_JOBS=1

python3 setup.py install
cd /tmp && rm -rf torchvision_src
```

### 3.2 验证 torchvision

```bash
python3 -c "
import torch
import torchvision
print('torch:', torch.__version__, '| torchvision:', torchvision.__version__)
print('CUDA:', torch.cuda.is_available())
"
```

---

## 4. 安装 ultralytics（YOLOv8）

```bash
conda activate task
pip3 install 'ultralytics>=8.0'
```

---

## 5. 故障排查

| 现象 | 处理 |
|------|------|
| **torch.cuda.is_available() 为 False** | 确认装的是 NVIDIA JetPack 对应版本的 wheel，不是 `pip install torch` 的通用包。 |
| **torchvision==0.19 is incompatible with torch==2.0** | 装 torchvision 0.15.x，且须从源码编译（见 3.1），不要用 pip 安装。 |
| **No module named 'torch._custom_ops'** | torch 与 torchvision 不匹配。须按 3.1 从源码编译安装 torchvision 0.15.2。 |
| **Couldn't load custom C++ ops** | pip 的 torchvision 与 Jetson 的 torch ABI 不一致。必须按 3.1 从源码编译 torchvision。 |
| **Killed / fatal error: Killed signal terminated program cc1plus** | 编译时内存不足，**必须先** `export MAX_JOBS=1` 再执行 `python3 setup.py install`；编译前关掉浏览器等大程序，仍失败则临时加 2GB swap（见 3.1 节）。 |
| **RuntimeError: expected scalar type Half but found Float** | 不要在加载时对模型做 `.half()`；推理时传 `half=False`。本仓库 detector 已按此处理。 |

---

## 6. 一键命令（Python 3.8 + torch 2.0.0+nv23.5 + torchvision 0.15.2）

**仅 PyTorch + 验证（torchvision 需按 3.1 从源码编译）：**

```bash
sudo apt-get -y update && sudo apt-get install -y python3-pip libopenblas-dev
conda activate task
pip3 install --upgrade pip && pip3 install 'numpy<2'
pip3 install --no-cache-dir https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
python3 -c "import torch; print('torch:', torch.__version__, 'CUDA:', torch.cuda.is_available())"
```

**torchvision 0.15.2 从源码编译（在已装好上述 PyTorch 的前提下；注意内存，务必先设 MAX_JOBS=1）：**

```bash
conda activate task
pip3 uninstall -y torchvision
sudo apt-get install -y libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
cd /tmp && git clone --depth 1 --branch v0.15.2 https://github.com/pytorch/vision torchvision_src && cd torchvision_src
export BUILD_VERSION=0.15.2
export MAX_JOBS=1
python3 setup.py install
cd /tmp && rm -rf torchvision_src
python3 -c "import torch, torchvision; print('torch:', torch.__version__, 'torchvision:', torchvision.__version__)"
```

---

## 7. 参考链接

- [NVIDIA: Installing PyTorch for Jetson Platform](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
- [PyTorch / torchvision 版本对应](https://github.com/pytorch/vision#installation)
- [NVIDIA Jetson PyTorch 兼容表](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)
