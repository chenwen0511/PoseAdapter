"""
src/adapter 下 detector / tracker / controller / ocr 的占位测试。
各模块实现后在此补充或拆分为 test_detector.py 等。
"""
import pytest


def test_adapter_detector_importable():
    """detector 模块可导入."""
    try:
        import src.adapter.detector as m
        assert m is not None
    except ImportError:
        pytest.skip("src.adapter.detector 尚未实现或依赖缺失")


def test_adapter_tracker_importable():
    """tracker 模块可导入."""
    try:
        import src.adapter.tracker as m
        assert m is not None
    except ImportError:
        pytest.skip("src.adapter.tracker 尚未实现或依赖缺失")


def test_adapter_controller_importable():
    """controller 模块可导入."""
    try:
        import src.adapter.controller as m
        assert m is not None
    except ImportError:
        pytest.skip("src.adapter.controller 尚未实现或依赖缺失")


def test_adapter_ocr_importable():
    """ocr 模块可导入."""
    try:
        import src.adapter.ocr as m
        assert m is not None
    except ImportError:
        pytest.skip("src.adapter.ocr 尚未实现或依赖缺失")
