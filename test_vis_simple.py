#!/usr/bin/env python3
"""
测试可视化功能（独立测试版）
"""

import cv2
import numpy as np


def get_edge_image(cv_image, bbox):
    """获取边缘检测结果"""
    x1, y1, x2, y2 = bbox
    h, w = cv_image.shape[:2]
    margin = 5
    x1 = max(0, x1 - margin)
    y1 = max(0, y1 - margin)
    x2 = min(w, x2 + margin)
    y2 = min(h, y2 + margin)
    
    roi = cv_image[y1:y2, x1:x2]
    if roi.size == 0:
        return None
    
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    edge_vis = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(edge_vis, contours, -1, (0, 0, 255), 2)
    
    return edge_vis


def main():
    # 读取测试图片
    image_path = '/home/stephen/.openclaw/workspace/PoseAdapter/tests/meter_test.jpg'
    cv_image = cv2.imread(image_path)
    
    if cv_image is None:
        print(f"无法读取图片: {image_path}")
        return
    
    print(f"图片尺寸: {cv_image.shape}")
    h, w = cv_image.shape[:2]
    
    # 模拟检测到的 bbox（YOLO 结果）
    bbox = (int(w*0.2), int(h*0.15), int(w*0.8), int(h*0.85))
    print(f"YOLO Bbox: {bbox}")
    
    # 模拟 4 个角点（基于 bbox）
    keypoints = [
        [bbox[0], bbox[3]],  # 左下
        [bbox[2], bbox[3]],  # 右下
        [bbox[2], bbox[1]],  # 右上
        [bbox[0], bbox[1]]   # 左上
    ]
    print(f"4个角点: {keypoints}")
    
    # 模拟 PnP 结果
    pose = {'success': True, 'distance': 1.85, 'yaw': 5.2}
    print(f"PnP: 距离={pose['distance']}m, 偏航角={pose['yaw']}°")
    
    # 模拟控制命令（英文）
    target_distance = 1.7
    distance = pose['distance']
    yaw = pose['yaw']
    angle_tolerance = 2.0
    distance_tolerance = 0.05
    
    dist_diff = abs(distance - target_distance)
    if abs(yaw) > angle_tolerance:
        control_cmd = f"ROTATE {yaw:.1f}deg"
    elif dist_diff > distance_tolerance:
        direction = "FORWARD" if distance > target_distance else "BACKWARD"
        control_cmd = f"{direction} {dist_diff:.2f}m"
    else:
        control_cmd = "IN_POSITION"
    
    print(f"控制命令: {control_cmd}")
    
    # ========== 生成可视化图像 ==========
    debug_image = cv_image.copy()
    x1, y1, x2, y2 = bbox
    
    # 1. YOLO 检测框（绿色）
    cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 3)  # 粗绿色线
    cv2.putText(debug_image, f"YOLO:0.95", (x1, y1-15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 2. 边缘检测小图（左上角）
    edge_image = get_edge_image(cv_image, bbox)
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
    
    # 4. PnP 结果（右上角）
    pose_text = f"Dist: {distance:.2f}m | Yaw: {yaw:.1f}deg"
    cv2.rectangle(debug_image, (debug_image.shape[1]-260, 20), (debug_image.shape[1]-10, 55), (0, 0, 0), -1)
    cv2.putText(debug_image, pose_text, (debug_image.shape[1]-250, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 5. 控制命令（底部）
    cmd_text = f"CMD: {control_cmd}"
    text_size = cv2.getTextSize(cmd_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
    text_x = (debug_image.shape[1] - text_size[0]) // 2
    text_y = debug_image.shape[0] - 30
    cv2.rectangle(debug_image, (text_x-20, text_y-30), (text_x+text_size[0]+20, text_y+10), (0, 0, 0), -1)
    cv2.putText(debug_image, cmd_text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    # 6. 状态栏
    status_text = "Tracks: 1 | Target: 1"
    cv2.putText(debug_image, status_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 保存结果
    output_path = '/tmp/debug_visualization.jpg'
    cv2.imwrite(output_path, debug_image)
    print(f"\n✅ 已保存: {output_path}")
    
    # 复制到 workspace
    import shutil
    shutil.copy(output_path, '/home/stephen/.openclaw/workspace/debug_test.jpg')
    print(f"✅ 已复制到: /home/stephen/.openclaw/workspace/debug_test.jpg")


if __name__ == '__main__':
    main()