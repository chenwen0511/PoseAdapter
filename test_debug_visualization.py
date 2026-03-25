#!/usr/bin/env python3
"""
测试可视化功能：边缘检测、角点、PnP结果、控制命令
"""

import cv2
import numpy as np
import sys
import os

# 添加路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/pose_adapter/src'))

from pose_adapter.detector import MeterDetector
from pose_adapter.pose_solver import PoseSolver
from pose_adapter.controller import MotionController


def test_visualization():
    # 读取测试图片
    image_path = '/home/stephen/.openclaw/workspace/PoseAdapter/tests/meter_test.jpg'
    cv_image = cv2.imread(image_path)
    
    if cv_image is None:
        print(f"无法读取图片: {image_path}")
        return
    
    print(f"图片尺寸: {cv_image.shape}")
    
    # 模拟参数
    camera_matrix = np.array([
        [700, 0, 640],
        [0, 700, 360],
        [0, 0, 1]
    ], dtype=np.float64)
    dist_coeffs = np.array([0.1, -0.2, 0, 0, 0.05], dtype=np.float64)
    meter_size = (0.2, 0.3)
    
    # 初始化模块
    detector = MeterDetector(
        model_path=None,  # 不使用YOLO，用备用方案
        logger=print
    )
    pose_solver = PoseSolver(camera_matrix, dist_coeffs, meter_size, logger=print)
    
    # 1. 检测
    print("\n=== 1. 检测 ===")
    detections = detector.detect(cv_image)
    print(f"检测到 {len(detections)} 个目标")
    
    if not detections:
        print("未检测到目标，使用预设bbox测试")
        # 手动创建一个测试 bbox
        h, w = cv_image.shape[:2]
        detections = [(int(w*0.3), int(h*0.3), int(w*0.7), int(h*0.7), 0.8, 0)]
    
    # 取第一个检测结果
    x1, y1, x2, y2, conf, cls = detections[0]
    bbox = (x1, y1, x2, y2)
    print(f"Bbox: {bbox}")
    
    # 2. 边缘检测可视化
    print("\n=== 2. 边缘检测 ===")
    edge_image = detector.get_edge_image(cv_image, bbox)
    if edge_image is not None:
        print(f"边缘图像尺寸: {edge_image.shape}")
        cv2.imwrite('/tmp/debug_edge.jpg', edge_image)
        print("已保存: /tmp/debug_edge.jpg")
    
    # 3. 角点检测
    print("\n=== 3. 角点检测 ===")
    keypoints = detector.extract_corners(cv_image, bbox)
    print(f"4个角点: {keypoints}")
    
    # 4. PnP 解算
    print("\n=== 4. PnP 解算 ===")
    image_shape = cv_image.shape[:2]
    pose = pose_solver.solve(bbox, image_shape, keypoints=keypoints)
    print(f"PnP结果: {pose}")
    
    if pose.get('success'):
        print(f"  距离: {pose['distance']:.3f}m")
        print(f"  Yaw: {pose['yaw']:.1f}°")
    
    # 5. 模拟控制命令
    print("\n=== 5. 控制命令 ===")
    target_distance = 1.7
    distance = pose.get('distance', 0) if pose.get('success') else 0
    yaw = pose.get('yaw', 0) if pose.get('success') else 0
    angle_tolerance = 2.0
    distance_tolerance = 0.05
    
    dist_diff = abs(distance - target_distance)
    if abs(yaw) > angle_tolerance:
        control_cmd = f"旋转 {yaw:.1f}°"
    elif dist_diff > distance_tolerance:
        direction = "前进" if distance > target_distance else "后退"
        control_cmd = f"{direction} {dist_diff:.2f}m"
    else:
        control_cmd = "已到位"
    
    print(f"控制命令: {control_cmd}")
    
    # 6. 生成完整可视化图像
    print("\n=== 6. 生成可视化图像 ===")
    debug_image = cv_image.copy()
    
    # 绘制检测框（绿色）
    cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(debug_image, f"YOLO:{conf:.2f}", (x1, y1-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # 绘制角点（黄色）
    if keypoints and len(keypoints) == 4:
        for idx, (cx, cy) in enumerate(keypoints):
            cv2.circle(debug_image, (int(cx), int(cy)), 8, (0, 255, 255), -1)
            cv2.putText(debug_image, str(idx), (int(cx)+8, int(cy)-8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        # 连接角点
        pts = np.array(keypoints, dtype=np.int32)
        cv2.polylines(debug_image, [pts], True, (0, 255, 255), 2)
    
    # 绘制边缘检测小图（右上角）
    if edge_image is not None:
        h, w = debug_image.shape[:2]
        edge_h, edge_w = edge_image.shape[:2]
        scale = min(200 / edge_h, 300 / edge_w)
        edge_small = cv2.resize(edge_image, (int(edge_w * scale), int(edge_h * scale)))
        roi_y, roi_x = 10, 10
        roi_h, roi_w = edge_small.shape[:2]
        if roi_y + roi_h < h and roi_x + roi_w < w:
            debug_image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w] = edge_small
        cv2.rectangle(debug_image, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (0, 255, 255), 1)
        cv2.putText(debug_image, "Edge Detect", (roi_x, roi_y-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    
    # 绘制 PnP 结果（右上角）
    if pose.get('success'):
        distance = pose.get('distance', 0)
        yaw = pose.get('yaw', 0)
        pose_text = f"Distance: {distance:.2f}m | Yaw: {yaw:.1f}°"
        text_size = cv2.getTextSize(pose_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
        text_x = debug_image.shape[1] - text_size[0] - 20
        cv2.rectangle(debug_image, (text_x-10, 20), (debug_image.shape[1]-10, 50), (0, 0, 0), -1)
        cv2.putText(debug_image, pose_text, (text_x, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 绘制控制命令（底部）
    if control_cmd:
        cmd_text = f"CMD: {control_cmd}"
        text_size = cv2.getTextSize(cmd_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
        text_x = (debug_image.shape[1] - text_size[0]) // 2
        text_y = debug_image.shape[0] - 30
        cv2.rectangle(debug_image, (text_x-20, text_y-25), (text_x+text_size[0]+20, text_y+10), (0, 0, 0), -1)
        cv2.putText(debug_image, cmd_text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    # 状态栏
    status_text = f"Tracks: 1 | Target: 1"
    cv2.putText(debug_image, status_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 保存结果
    output_path = '/tmp/debug_visualization.jpg'
    cv2.imwrite(output_path, debug_image)
    print(f"已保存可视化结果: {output_path}")
    
    print("\n✅ 测试完成！")


if __name__ == '__main__':
    test_visualization()