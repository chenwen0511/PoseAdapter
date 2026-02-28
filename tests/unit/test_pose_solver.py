<<<<<<< HEAD
"""
src/adapter/pose_solver 的单元测试占位。
待 pose_solver 实现 PnP 解算后，补充：距离/偏航/俯仰计算、多帧平滑等。
"""
import numpy as np
import pytest


def test_pose_solver_module_importable():
    """pose_solver 模块可被导入（当前可为空实现）."""
    try:
        import src.adapter.pose_solver as m
        assert m is not None
    except ImportError:
        pytest.skip("src.adapter.pose_solver 尚未实现或依赖缺失")


def test_sample_K_shape(sample_K_720p):
    """内参 K 为 3x3."""
    assert sample_K_720p.shape == (3, 3)
    assert sample_K_720p[2, 2] == 1.0
=======
"""
src/adapter/pose_solver 的单元测试占位。
待 pose_solver 实现 PnP 解算后，补充：距离/偏航/俯仰计算、多帧平滑等。
"""
import numpy as np
import pytest


def test_pose_solver_module_importable():
    """pose_solver 模块可被导入（当前可为空实现）."""
    try:
        import src.adapter.pose_solver as m
        assert m is not None
    except ImportError:
        pytest.skip("src.adapter.pose_solver 尚未实现或依赖缺失")


def test_sample_K_shape(sample_K_720p):
    """内参 K 为 3x3."""
    assert sample_K_720p.shape == (3, 3)
    assert sample_K_720p[2, 2] == 1.0
>>>>>>> a5a410c (add default camera calib yaml)
