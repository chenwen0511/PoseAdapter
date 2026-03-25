#!/usr/bin/env python3
"""
测试 YOLO 检测（不依赖 ROS）
"""

import cv2
import numpy as np

# 直接导入 detector
import sys
import os
os.chdir('/home/stephen/.openclaw/workspace/PoseAdapter')
sys.path.insert(0, '/home/stephen/.openclaw/workspace/PoseAdapter/src/pose_adapter/src')

# 临时绕过 rclpy 导入
import types
import sys

# Mock rclpy
rclpy_mock = types.ModuleType('rclpy')
class MockNode:
    def __init__(self, *args, **kwargs):
        pass
    def get_logger(self):
        return lambda x: print(x)
rclpy_mock.Node = MockNode
sys.modules['rclpy'] = rclpy_mock

# 现在可以导入了
from pose_adapter.detector import MeterDetector


def main():
    # 读取测试图片
    image_path = '/home/stephen/.openclaw/workspace/PoseAdapter/tests/meter_test.jpg'
    cv_image = cv2.imread(image_path)
    
    if cv_image is None:
        print(f"无法读取图片: {image_path}")
        return
    
    print(f"图片尺寸: {cv_image.shape}")
    h, w = cv_image.shape[:2]
    
    # 使用真正的 YOLO 检测
    model_path = '/home/stephen/.openclaw/workspace/PoseAdapter/model/best.pt'
    detector = MeterDetector(
        model_path=model_path,
        conf_threshold=0.3,
        logger=print
    )
    
    print("\n=== YOLO 检测 ===")
    detections = detector.detect(cv_image)
    print(f"检测到 {len(detections)} 个目标")
    
    if not detections:
        print("未检测到目标！")
        return
    
    # 取所有检测结果
    for i, det in enumerate(detections):
        x1, y1, x2, y2, conf, cls = det
        print(f"检测框 {i+1}: x1={x1}, y1={y1}, x2={x2}, y2={y2}, conf={conf:.2f}")
    
    # 取第一个检测结果
    x1, y1, x2, y2, conf, cls = detections[0]
    
    # 提取角点
    bbox = (x1, y1, x2, y2)
    keypoints = detector.extract_corners(cv_image, bbox)
    print(f"4个角点: {keypoints}")
    
    # 获取边缘检测图
    edge_image = detector.get_edge_image(cv_image, bbox)
    
    # ========== 生成可视化图像 ==========
    debug_image = cv_image.copy()
    
    # 1. YOLO 检测框（绿色）
    cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
    cv2.putText(debug_image, f"YOLO:{conf:.2f}", (x1, y1-15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 2. 边缘检测小图（左上角）
    if edge_image is not None:
        edge_h, edge_w = edge_image.shape[:2]
        scale = min(200 / edge_h, 300 / edge_w)
        edge_small = cv2.resize(edge_image, (int(edge_w * scale), int(edge_h * scale)))
        roi_y, roi_x = 10, 10
        roi_h, roi_w = edge_small.shape[:2]
        debug_image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w] = edge_small
        cv2.rectangle(debug_image, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (0, 255, 255), 2)
        cv2.putText(debug_image, "Edge", (roi_x, roi_y-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    # 3. 4个角点（黄色圆点+连线）
    if keypoints and len(keypoints) == 4:
        for idx, (cx, cy) in enumerate(keypoints):
            cv2.circle(debug_image, (int(cx), int(cy)), 10, (0, 255, 255), -1)
            cv2.putText(debug_image, str(idx), (int(cx)+12, int(cy)-12),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        pts = np.array(keypoints, dtype=np.int32)
        cv2.polylines(debug_image, [pts], True, (0, 255, 255), 2)
    
    # 4. 状态栏
    status_text = f"Detections: {len(detections)}"
    cv2.putText(debug_image, status_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 保存结果
    output_path = '/tmp/debug_yolo.jpg'
    cv2.imwrite(output_path, debug_image)
    print(f"\n✅ 已保存: {output_path}")
    
    import shutil
    shutil.copy(output_path, '/home/stephen/.openclaw/workspace/debug_yolo.jpg')
    print(f"✅ 已复制到: /home/stephen/.openclaw/workspace/debug_yolo.jpg")


if __name__ == '__main__':
    main()