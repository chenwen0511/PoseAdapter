#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
位姿解算器 - 基于 PnP 的 6DoF 位姿估计
"""

import cv2
import numpy as np
import rospy


class PoseSolver:
    """
    PnP 位姿解算器
    
    通过电表关键点（2D）和已知物理尺寸（3D）解算相机相对电表的位姿
    """
    
    def __init__(self, camera_matrix, dist_coeffs, meter_size=(0.2, 0.3)):
        """
        初始化位姿解算器
        
        Args:
            camera_matrix: 3x3 相机内参矩阵
            dist_coeffs: 畸变系数
            meter_size: (width, height) 电表物理尺寸（米）
        """
        self.K = np.array(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64)
        self.meter_width, self.meter_height = meter_size
        
        # 构建电表 3D 点（假设电表是平面矩形，中心在原点）
        w, h = self.meter_width / 2, self.meter_height / 2
        self.object_points = np.array([
            [-w, -h, 0],   # 左下
            [w, -h, 0],    # 右下
            [w, h, 0],     # 右上
            [-w, h, 0]     # 左上
        ], dtype=np.float64)
        
        # 位姿平滑滤波
        self.pose_history = []
        self.history_size = 5
    
    def solve(self, bbox, image_shape):
        """
        解算位姿
        
        Args:
            bbox: (x1, y1, x2, y2) 检测框
            image_shape: (h, w) 图像尺寸
            
        Returns:
            dict: {
                'success': bool,
                'distance': float,  # 距离（米）
                'yaw': float,       # 偏航角（度）
                'pitch': float,     # 俯仰角（度）
                'roll': float,      # 翻滚角（度）
                'tvec': np.array,   # 平移向量
                'rvec': np.array    # 旋转向量
            }
        """
        x1, y1, x2, y2 = bbox
        
        # 从 bbox 获取 2D 点（四角）
        image_points = np.array([
            [x1, y2],   # 左下
            [x2, y2],   # 右下
            [x2, y1],   # 右上
            [x1, y1]    # 左上
        ], dtype=np.float64)
        
        try:
            # PnP 解算
            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.K,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if not success:
                return {'success': False}
            
            # 计算距离
            distance = np.linalg.norm(tvec)
            
            # 转换为欧拉角
            R, _ = cv2.Rodrigues(rvec)
            yaw, pitch, roll = self._rotation_matrix_to_euler(R)
            
            pose = {
                'success': True,
                'distance': float(distance),
                'yaw': float(np.degrees(yaw)),
                'pitch': float(np.degrees(pitch)),
                'roll': float(np.degrees(roll)),
                'tvec': tvec.flatten(),
                'rvec': rvec.flatten()
            }
            
            # 平滑滤波
            smoothed_pose = self._smooth_pose(pose)
            return smoothed_pose
            
        except Exception as e:
            rospy.logerr(f"PnP 解算失败: {e}")
            return {'success': False}
    
    def _rotation_matrix_to_euler(self, R):
        """
        旋转矩阵转欧拉角 (ZYX 顺序)
        
        Returns:
            (yaw, pitch, roll) 弧度
        """
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        
        return z, y, x  # yaw, pitch, roll
    
    def _smooth_pose(self, pose):
        """对位姿进行滑动平均滤波"""
        self.pose_history.append(pose)
        if len(self.pose_history) > self.history_size:
            self.pose_history.pop(0)
        
        if len(self.pose_history) < 2:
            return pose
        
        # 平滑后的位姿
        smoothed = {
            'success': True,
            'distance': np.mean([p['distance'] for p in self.pose_history]),
            'yaw': np.mean([p['yaw'] for p in self.pose_history]),
            'pitch': np.mean([p['pitch'] for p in self.pose_history]),
            'roll': np.mean([p['roll'] for p in self.pose_history]),
            'tvec': pose['tvec'],  # 使用最新值
            'rvec': pose['rvec']
        }
        
        return smoothed
    
    def get_target_bbox_ratio(self, bbox, image_shape):
        """
        计算目标在画面中的占比
        
        Args:
            bbox: (x1, y1, x2, y2)
            image_shape: (h, w)
            
        Returns:
            float: 宽度占比 (0-1)
        """
        x1, y1, x2, y2 = bbox
        bbox_width = x2 - x1
        image_width = image_shape[1]
        return bbox_width / image_width
    
    def get_center_offset(self, bbox, image_shape):
        """
        计算目标中心与画面中心的偏差
        
        Args:
            bbox: (x1, y1, x2, y2)
            image_shape: (h, w)
            
        Returns:
            (offset_x, offset_y): 归一化偏差 (-1 到 1)
        """
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        
        image_cx = image_shape[1] / 2
        image_cy = image_shape[0] / 2
        
        offset_x = (cx - image_cx) / image_shape[1] * 2  # -1 到 1
        offset_y = (cy - image_cy) / image_shape[0] * 2
        
        return offset_x, offset_y