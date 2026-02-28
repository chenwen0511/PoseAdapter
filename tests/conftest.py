<<<<<<< HEAD
"""
pytest 共享 fixture：项目根路径、相机内参、标定 yaml 等。
运行方式：在项目根目录执行 pytest tests/ -v
"""
import sys
from pathlib import Path

import numpy as np
import pytest

# 保证项目根在 path 中，便于 from src.xxx 导入
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


@pytest.fixture
def project_root():
    """项目根目录 Path."""
    return ROOT


@pytest.fixture
def sample_K_720p():
    """720P 经验内参矩阵 (3x3)."""
    return np.array([
        [700.0, 0.0, 640.0],
        [0.0, 700.0, 360.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)


@pytest.fixture
def sample_dist():
    """5 参数畸变系数."""
    return np.array([0.1, -0.2, 0.0, 0.0, 0.05], dtype=np.float32)


@pytest.fixture
def sample_calib_yaml_path(project_root):
    """测试用标定 yaml 路径（tests/fixtures 下）."""
    p = project_root / "tests" / "fixtures" / "calib_sample.yaml"
    return str(p)
=======
"""
pytest 共享 fixture：项目根路径、相机内参、标定 yaml 等。
运行方式：在项目根目录执行 pytest tests/ -v
"""
import sys
from pathlib import Path

import numpy as np
import pytest

# 保证项目根在 path 中，便于 from src.xxx 导入
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


@pytest.fixture
def project_root():
    """项目根目录 Path."""
    return ROOT


@pytest.fixture
def sample_K_720p():
    """720P 经验内参矩阵 (3x3)."""
    return np.array([
        [700.0, 0.0, 640.0],
        [0.0, 700.0, 360.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)


@pytest.fixture
def sample_dist():
    """5 参数畸变系数."""
    return np.array([0.1, -0.2, 0.0, 0.0, 0.05], dtype=np.float32)


@pytest.fixture
def sample_calib_yaml_path(project_root):
    """测试用标定 yaml 路径（tests/fixtures 下）."""
    p = project_root / "tests" / "fixtures" / "calib_sample.yaml"
    return str(p)
>>>>>>> a5a410c (add default camera calib yaml)
