#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试轮廓角点提取 + PnP 位姿解算（独立版本，不依赖 ROS）
"""

import os
import cv2
import numpy as np


def extract_contour_corners(cv_image, bbox):
    """
    使用轮廓检测 + 角点提取
    """
    x1, y1, x2, y2 = bbox
    
    # 扩展 bbox 区域
    h, w = cv_image.shape[:2]
    margin = 5
    x1 = max(0, x1 - margin)
    y1 = max(0, y1 - margin)
    x2 = min(w, x2 + margin)
    y2 = min(h, y2 + margin)
    
    # 裁剪 ROI
    roi = cv_image[y1:y2, x1:x2]
    if roi.size == 0:
        print("  [警告] ROI 为空，回退到 bbox")
        return get_bbox_corners(bbox)
    
    # 转灰度
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # 预处理：模糊 + 边缘检测
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    # 形态学操作：闭合边缘
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    
    # 找轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        print("  [警告] 未找到轮廓，回退到 bbox")
        return get_bbox_corners(bbox)
    
    # 找最大轮廓
    max_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_contour)
    
    # 面积太小的不要
    roi_area = roi.shape[0] * roi.shape[1]
    if area < roi_area * 0.03:  # 降低到 3%
        print(f"  [警告] 轮廓面积太小 ({area/roi_area*100:.1f}%)，回退到 bbox")
        return get_bbox_corners(bbox)
    
    # 近似多边形，尝试拟合4个角点
    epsilon = 0.02 * cv2.arcLength(max_contour, True)
    approx = cv2.approxPolyDP(max_contour, epsilon, True)
    
    # 如果拟合结果不是4点，尝试调整 epsilon
    if len(approx) < 4:
        epsilon = 0.04 * cv2.arcLength(max_contour, True)
        approx = cv2.approxPolyDP(max_contour, epsilon, True)
    
    if len(approx) < 4:
        # 用凸包
        hull = cv2.convexHull(max_contour)
        hull_approx = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
        if len(hull_approx) >= 4:
            approx = hull_approx[:4]
        else:
            print("  [警告] 无法提取4个角点，回退到 bbox")
            return get_bbox_corners(bbox)
    
    # 如果点数 > 4，取前4个
    if len(approx) > 4:
        hull = cv2.convexHull(max_contour)
        hull_approx = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
        approx = hull_approx[:4]
    
    # 提取角点坐标并排序
    corners = []
    for pt in approx:
        if len(pt[0]) >= 2:
            corners.append([pt[0][0] + x1, pt[0][1] + y1])
    
    if len(corners) < 4:
        return get_bbox_corners(bbox)
    
    # 排序
    corners = sort_corners(corners)
    return corners


def get_bbox_corners(bbox):
    """使用 bbox 四个角点"""
    x1, y1, x2, y2 = bbox
    return [[int(x1), int(y1)], [int(x2), int(y1)], [int(x2), int(y2)], [int(x1), int(y2)]]


def sort_corners(corners):
    """将4个角点排序为顺时针顺序（左上、右上、右下、左下）"""
    if len(corners) != 4:
        return corners
    
    corners = sorted(corners, key=lambda p: p[0] + p[1])
    top_left = corners[0]
    bottom_right = corners[3]
    
    remaining = [corners[1], corners[2]]
    remaining = sorted(remaining, key=lambda p: p[0] - p[1])
    top_right = remaining[0]
    bottom_left = remaining[1]
    
    return [top_left, top_right, bottom_right, bottom_left]


def simple_detection(cv_image):
    """
    简单的电表检测（基于颜色：浅色区域）
    返回第一个检测到的 bbox
    """
    h, w = cv_image.shape[:2]
    total_area = h * w
    
    # 方法1: HSV 找浅色区域（电表白色/浅灰色）
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([180, 50, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 找大面积的浅色矩形区域
    best_bbox = None
    best_area = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < total_area * 0.02:  # 至少 2%
            continue
        
        x, y, bw, bh = cv2.boundingRect(cnt)
        aspect = bw / float(bh) if bh > 0 else 0
        
        # 电表宽高比通常 0.5-2.0
        if 0.5 < aspect < 2.0:
            if area > best_area:
                best_area = area
                best_bbox = (x, y, x+bw, y+bh)
    
    if best_bbox:
        return best_bbox, best_area / total_area
    
    # 方法2: 自适应阈值（备用）
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY_INV, 11, 2)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    min_area = total_area * 0.02
    max_area = total_area * 0.5
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (min_area < area < max_area):
            continue
        
        x, y, bw, bh = cv2.boundingRect(cnt)
        aspect_ratio = bw / float(bh)
        
        if 0.5 < aspect_ratio < 2.0:
            return (x, y, x+bw, y+bh), area / total_area
    
    return None, 0


def solve_pnp(corners, image_shape, meter_size=(0.2, 0.3)):
    """
    PnP 位姿解算
    """
    h, w = image_shape
    
    # 默认相机内参
    K = np.array([
        [800, 0, w/2],
        [0, 800, h/2],
        [0, 0, 1]
    ], dtype=np.float64)
    dist_coeffs = np.zeros(5, dtype=np.float64)
    
    # 电表 3D 点
    mw, mh = meter_size
    object_points = np.array([
        [-mw/2, -mh/2, 0],
        [mw/2, -mh/2, 0],
        [mw/2, mh/2, 0],
        [-mw/2, mh/2, 0]
    ], dtype=np.float64)
    
    # 图像点（关键点顺序：左上->右上->右下->左下，需要转换）
    image_points = np.array([
        [corners[3][0], corners[3][1]],  # 左下
        [corners[2][0], corners[2][1]],  # 右下
        [corners[1][0], corners[1][1]],  # 右上
        [corners[0][0], corners[0][1]]   # 左上
    ], dtype=np.float64)
    
    try:
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, K, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not success:
            return None
        
        distance = np.linalg.norm(tvec)
        
        # 旋转向量转欧拉角
        R, _ = cv2.Rodrigues(rvec)
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        if sy > 1e-6:
            yaw = np.arctan2(R[1,0], R[0,0])
            pitch = np.arctan2(-R[2,0], sy)
            roll = np.arctan2(R[2,1], R[2,2])
        else:
            yaw, pitch, roll = 0, 0, 0
        
        return {
            'distance': float(distance),
            'yaw': float(np.degrees(yaw)),
            'pitch': float(np.degrees(pitch)),
            'roll': float(np.degrees(roll)),
            'tvec': tvec.flatten()
        }
    except Exception as e:
        print(f"  PnP 错误: {e}")
        return None


def test_image(image_path, method='contour'):
    """测试单张图片"""
    print(f"\n{'='*50}")
    print(f"测试图片: {image_path}")
    print(f"方法: {method}")
    print(f"{'='*50}")
    
    cv_image = cv2.imread(image_path)
    if cv_image is None:
        print(f"无法读取图片: {image_path}")
        return
    
    h, w = cv_image.shape[:2]
    print(f"图片尺寸: {w}x{h}")
    
    # 检测
    print("\n[1] 电表检测...")
    bbox, ratio = simple_detection(cv_image)
    if bbox is None:
        print("未检测到电表！")
        return
    
    print(f"  Bbox: {bbox}, 面积比: {ratio*100:.1f}%")
    
    # 角点提取
    print("\n[2] 角点提取...")
    if method == 'bbox':
        corners = get_bbox_corners(bbox)
    else:
        corners = extract_contour_corners(cv_image, bbox)
    
    print(f"  角点: {corners}")
    
    # 可视化
    vis = cv_image.copy()
    x1, y1, x2, y2 = bbox
    cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
    for i, (cx, cy) in enumerate(corners):
        cv2.circle(vis, (int(cx), int(cy)), 10, colors[i], -1)
        cv2.putText(vis, str(i+1), (int(cx)+15, int(cy)+15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors[i], 2)
    
    cv2.imwrite('/tmp/test_detection.jpg', vis)
    print("  已保存: /tmp/test_detection.jpg")
    
    # PnP
    print("\n[3] PnP 位姿解算...")
    pose = solve_pnp(corners, (h, w))
    
    if pose:
        print(f"  ✓ 距离: {pose['distance']:.3f} m")
        print(f"  ✓ 偏航角(yaw): {pose['yaw']:.2f}°")
        print(f"  ✓ 俯仰角(pitch): {pose['pitch']:.2f}°")
        print(f"  ✓ 翻滚角(roll): {pose['roll']:.2f}°")
    else:
        print("  ✗ 解算失败")
    
    return pose, corners, bbox


def test_video(video_path, method='contour', max_frames=50):
    """测试视频"""
    print(f"\n{'='*50}")
    print(f"测试视频: {video_path}")
    print(f"方法: {method}")
    print(f"{'='*50}")
    
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"无法打开视频: {video_path}")
        return
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"视频: {width}x{height}, {fps}fps, {total_frames}帧")
    
    frame_idx = 0
    success_count = 0
    results = []
    
    while frame_idx < max_frames:
        ret, cv_image = cap.read()
        if not ret:
            break
        
        frame_idx += 1
        
        # 检测
        bbox, _ = simple_detection(cv_image)
        if bbox is None:
            continue
        
        # 角点提取
        if method == 'bbox':
            corners = get_bbox_corners(bbox)
        else:
            corners = extract_contour_corners(cv_image, bbox)
        
        # PnP
        pose = solve_pnp(corners, (height, width))
        
        if pose:
            success_count += 1
            results.append({
                'frame': frame_idx,
                'distance': pose['distance'],
                'yaw': pose['yaw']
            })
            if frame_idx % 10 == 0:
                print(f"帧 {frame_idx}: 距离={pose['distance']:.3f}m, yaw={pose['yaw']:.1f}°")
            
            # 保存关键帧
            if frame_idx in [1, 10, 20, 30, 40]:
                vis = cv_image.copy()
                x1, y1, x2, y2 = bbox
                cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                for i, (cx, cy) in enumerate(corners):
                    cv2.circle(vis, (int(cx), int(cy)), 8, (0, 0, 255), -1)
                cv2.imwrite(f'/tmp/frame_{frame_idx}.jpg', vis)
        else:
            if frame_idx % 10 == 0:
                print(f"帧 {frame_idx}: 解算失败")
    
    cap.release()
    
    print(f"\n{'='*50}")
    print(f"视频测试完成: {frame_idx} 帧, 成功 {success_count} 次")
    print(f"成功率: {success_count/frame_idx*100:.1f}%")
    
    if results:
        distances = [r['distance'] for r in results]
        yaws = [r['yaw'] for r in results]
        print(f"距离范围: {min(distances):.3f}m - {max(distances):.3f}m")
        print(f"偏航角范围: {min(yaws):.1f}° - {max(yaws):.1f}°")
        print(f"关键帧已保存: /tmp/frame_*.jpg")
    
    return results


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=str, default='/home/stephen/.openclaw/workspace/PoseAdapter/tests/meter_test.jpg')
    parser.add_argument('--video', type=str, default='/home/stephen/.openclaw/workspace/PoseAdapter/tests/meter_video.mp4')
    parser.add_argument('--method', type=str, default='contour', choices=['bbox', 'contour'])
    parser.add_argument('--max-frames', type=int, default=50)
    args = parser.parse_args()
    
    # 测试图片
    if os.path.exists(args.image):
        test_image(args.image, args.method)
    
    # 测试视频
    if os.path.exists(args.video):
        test_video(args.video, args.method, args.max_frames)
