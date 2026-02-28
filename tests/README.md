<<<<<<< HEAD
# 测试说明

## 目录结构

```
tests/
├── README.md           # 本文件
├── conftest.py         # 共享 fixture（project_root, sample_K_720p, sample_dist, sample_calib_yaml_path）
├── fixtures/           # 测试数据
│   └── calib_sample.yaml   # 标定 yaml 样例，供 test_read_calib_params 使用
└── unit/               # 单元测试
    ├── test_read_calib_params.py   # 标定参数解析
    ├── test_pose_solver.py         # 位姿解算（占位）
    └── test_adapter_placeholders.py # detector / tracker / controller / ocr 导入占位
```

## 运行方式

在**项目根目录**下执行：

```bash
# 安装测试依赖（若尚未安装）
pip install -r requirements-test.txt

# 运行全部测试
pytest tests/ -v

# 仅运行 unit
pytest tests/unit/ -v

# 仅运行标定相关
pytest tests/unit/test_read_calib_params.py -v
```

## 依赖

- `pytest`
- `numpy`（conftest 与后续 pose_solver 测试）
- `pyyaml`（read_calib_params 及 test_read_calib_params）
=======
# 测试说明

## 目录结构

```
tests/
├── README.md           # 本文件
├── conftest.py         # 共享 fixture（project_root, sample_K_720p, sample_dist, sample_calib_yaml_path）
├── fixtures/           # 测试数据
│   └── calib_sample.yaml   # 标定 yaml 样例，供 test_read_calib_params 使用
└── unit/               # 单元测试
    ├── test_read_calib_params.py   # 标定参数解析
    ├── test_pose_solver.py         # 位姿解算（占位）
    └── test_adapter_placeholders.py # detector / tracker / controller / ocr 导入占位
```

## 运行方式

在**项目根目录**下执行：

```bash
# 安装测试依赖（若尚未安装）
pip install -r requirements-test.txt

# 运行全部测试
pytest tests/ -v

# 仅运行 unit
pytest tests/unit/ -v

# 仅运行标定相关
pytest tests/unit/test_read_calib_params.py -v
```

## 依赖

- `pytest`
- `numpy`（conftest 与后续 pose_solver 测试）
- `pyyaml`（read_calib_params 及 test_read_calib_params）
>>>>>>> a5a410c (add default camera calib yaml)
