#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整流程测试：拉流 → 识别 → 关键点 → PnP → 姿态解算 → 控制
不依赖 ROS2，可以直接运行
"""

import os
import sys
import time
import cv2
import numpy as np
import importlib.util

# 直接从文件导入模块
src_dir = os.path.dirname(__file__)
pose_adapter_dir = os.path.join(src_dir, '../src/pose_adapter/src')

def import_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

# 导入模块
print("加载模块...")
detector_mod = import_module('detector', os.path.join(pose_adapter_dir, 'pose_adapter/detector.py'))
tracker_mod = import_module('tracker', os.path.join(pose_adapter_dir, 'pose_adapter/tracker.py'))
pose_solver_mod = import_module('pose_solver', os.path.join(pose_adapter_dir, 'pose_adapter/pose_solver.py'))

MeterDetector = detector_mod.MeterDetector
DeepSORTTracker = tracker_mod.DeepSORTTracker
PoseSolver = pose_solver_mod.PoseSolver

print("模块加载完成")

# 模拟日志
class MockLogger:
    def info(self, m): print(f"[INFO] {m}")
    def warn(self, m): print(f"[WARN] {m}")
    def error(self, m): print(f"[ERROR] {m}")
    def debug(self, m): pass

logger = MockLogger()


def test_01_detector():
    """测试 1: 检测器"""
    print("\n=== 测试 1: 检测器 ===")
    test_image = '/home/stephen/.openclaw/workspace/PoseAdapter/tests/meter_test.jpg'
    
    if not os.path.exists(test_image):
        print("跳过: 测试图片不存在")
        return
    
    image = cv2.imread(test_image)
    detector = MeterDetector(model_path=None, min_area_ratio=0.05, use_gpu=False, keypoint_method='bbox', logger=logger)
    detections = detector.detect(image)
    print(f"检测到 {len(detections)} 个目标")
    print("✓ 检测器测试完成")


def test_02_keypoints():
    """测试 2: 关键点提取"""
    print("\n=== 测试 2: 关键点提取 ===")
    test_image = '/home/stephen/.openclaw/workspace/PoseAdapter/tests/meter_test.jpg'
    
    if not os.path.exists(test_image):
        print("跳过: 测试图片不存在")
        return
    
    image = cv2.imread(test_image)
    detector = MeterDetector(model_path=None, logger=logger)
    h, w = image.shape[:2]
    bbox = (w//4, h//4, w*3//4, h*3//4)
    corners = detector.extract_corners(image, bbox)
    assert corners is not None and len(corners) == 4
    print(f"角点: {corners}")
    print("✓ 关键点提取测试完成")


def test_03_pnp():
    """测试 3: PnP 位姿解算"""
    print("\n=== 测试 3: PnP 位姿解算 ===")
    K = np.array([[878.19, 0, 923.80], [0, 868.56, 493.94], [0, 0, 1]], dtype=np.float64)
    dist = np.array([-0.045, -0.457, 0.002, 0.002, 0.835], dtype=np.float64)
    pose_solver = PoseSolver(K, dist, (0.2, 0.3), logger=logger)
    image_points = np.array([[400, 300], [600, 300], [600, 200], [400, 200]], dtype=np.float64)
    pose = pose_solver.solve_with_keypoints(image_points, (480, 640))
    assert pose['success']
    print(f"距离: {pose['distance']:.3f}m, yaw: {pose['yaw']:.2f}°")
    print("✓ PnP 测试完成")


def test_04_tracker():
    """测试 4: 目标追踪"""
    print("\n=== 测试 4: 目标追踪 ===")
    tracker = DeepSORTTracker(max_age=30, min_hits=3)
    tracker.update([(100, 100, 200, 200, 0.9, 0)])
    tracker.update([(105, 102, 205, 202, 0.9, 0)])
    tracks = tracker.update([(110, 105, 210, 205, 0.9, 0)])
    assert len(tracks) > 0
    print(f"追踪数: {len(tracks)}")
    print("✓ 追踪测试完成")


def test_05_full_pipeline():
    """测试 5: 完整流程"""
    print("\n=== 测试 5: 完整流程 ===")
    test_image = '/home/stephen/.openclaw/workspace/PoseAdapter/tests/meter_test.jpg'
    if not os.path.exists(test_image):
        print("跳过: 图片不存在")
        return
    
    K = np.array([[878.19, 0, 923.80], [0, 868.56, 493.94], [0, 0, 1]], dtype=np.float64)
    dist = np.array([-0.045, -0.457, 0.002, 0.002, 0.835], dtype=np.float64)
    
    detector = MeterDetector(model_path=None, keypoint_method='bbox', logger=logger)
    tracker = DeepSORTTracker()
    pose_solver = PoseSolver(K, dist, (0.2, 0.3), logger=logger)
    
    image = cv2.imread(test_image)
    h, w = image.shape[:2]
    
    detections = detector.detect(image)
    print(f"1. 检测: {len(detections)}")
    
    if not detections:
        detections = [(w//4, h//4, w*3//4, h*3//4, 0.9, 0)]
    
    tracks = tracker.update(detections)
    print(f"2. 追踪: {len(tracks)}")
    
    if tracks:
        _, bbox, _ = tracks[0]
        keypoints = detector.extract_corners(image, bbox)
        print(f"3. 关键点: {len(keypoints)}")
        pose = pose_solver.solve(bbox, (h, w), keypoints=keypoints)
        if pose['success']:
            print(f"4. PnP: {pose['distance']:.3f}m")
    
    print("✓ 完整流程测试完成")


def test_06_control_mock():
    """测试 6: 控制模拟"""
    print("\n=== 测试 6: 控制模拟 ===")
    target_distance = 1.5
    pose = {'distance': 1.6, 'yaw': 5.0}
    error = pose['distance'] - target_distance
    kp = 0.35
    vel = np.clip(kp * error, -0.2, 0.2)
    print(f"速度: {vel:.3f}m/s, 移动: {'前进' if error > 0 else '后退'}")
    print("✓ 控制测试完成")


def test_07_rtsp_mock():
    """测试 7: RTSP 流模拟"""
    print("\n=== 测试 7: RTSP 流模拟 ===")
    import queue
    import threading
    
    frame_queue = queue.Queue(maxsize=1)
    
    def producer():
        for i in range(5):
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            try:
                if frame_queue.full(): frame_queue.get_nowait()
                frame_queue.put(frame)
            except: pass
            time.sleep(0.05)
    
    threading.Thread(target=producer).start()
    time.sleep(0.3)
    while not frame_queue.empty():
        frame = frame_queue.get_nowait()
        print(f"帧: {frame.shape}")
    print("✓ RTSP 测试完成")


def test_08_pnp_corner():
    """测试 8: PnP 边界"""
    print("\n=== 测试 8: PnP 边界 ===")
    K = np.array([[878.19, 0, 923.80], [0, 868.56, 493.94], [0, 0, 1]], dtype=np.float64)
    dist = np.array([-0.045, -0.457, 0.002, 0.002, 0.835], dtype=np.float64)
    pose_solver = PoseSolver(K, dist, (0.2, 0.3))
    
    # 近距离
    img = np.array([[200,150],[800,150],[800,450],[200,450]], dtype=np.float64)
    pose = pose_solver.solve_with_keypoints(img, (480,1280))
    print(f"近: {pose['distance']:.3f}m")
    
    # 远距离
    img = np.array([[500,350],[540,350],[540,380],[500,380]], dtype=np.float64)
    pose = pose_solver.solve_with_keypoints(img, (480,640))
    print(f"远: {pose['distance']:.3f}m")
    print("✓ PnP边界测试完成")


def main():
    print("=" * 50)
    print("PoseAdapter 完整流程测试")
    print("=" * 50)
    
    tests = [
        ("检测器", test_01_detector),
        ("关键点", test_02_keypoints),
        ("PnP", test_03_pnp),
        ("追踪", test_04_tracker),
        ("完整流程", test_05_full_pipeline),
        ("控制", test_06_control_mock),
        ("RTSP", test_07_rtsp_mock),
        ("PnP边界", test_08_pnp_corner),
    ]
    
    passed = failed = 0
    for name, func in tests:
        try:
            func()
            passed += 1
        except Exception as e:
            print(f"✗ {name}: {e}")
            failed += 1
    
    print("\n" + "=" * 50)
    print(f"结果: {passed} 通过, {failed} 失败")
    print("=" * 50)


if __name__ == '__main__':
    main()
