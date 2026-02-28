"""
src/calibrate/read_calib_params.py 的单元测试。
"""
import pytest


def test_read_calibration_params_valid(sample_calib_yaml_path):
    """有效 yaml 应解析出 K、dist 与 calib_data."""
    from src.calibrate.read_calib_params import read_calibration_params

    K, dist, calib_data = read_calibration_params(sample_calib_yaml_path)
    assert K is not None and dist is not None and calib_data is not None
    assert K.shape == (3, 3)
    assert K[0, 0] == 700.0 and K[2, 2] == 1.0
    assert len(dist) == 5
    assert calib_data.get("image_width") == 1280
    assert calib_data.get("image_height") == 720


def test_read_calibration_params_file_not_found():
    """不存在文件应返回 (None, None, None)."""
    from src.calibrate.read_calib_params import read_calibration_params

    K, dist, calib_data = read_calibration_params("/nonexistent/calib.yaml")
    assert K is None and dist is None and calib_data is None


def test_read_calibration_params_invalid_yaml(tmp_path):
    """缺少 camera_matrix 的 yaml 应返回 (None, None, None)."""
    from src.calibrate.read_calib_params import read_calibration_params

    bad_yaml = tmp_path / "bad.yaml"
    bad_yaml.write_text("image_width: 640\nimage_height: 480\n")
    K, dist, calib_data = read_calibration_params(str(bad_yaml))
    assert K is None and dist is None and calib_data is None
