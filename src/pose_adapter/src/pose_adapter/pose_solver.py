#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿解算器 - 基于 PnP 的 6DoF 位姿估计 (ROS2 版本)
"""

import cv2
import numpy as np
import time


class PoseSolver:
    """
    PnP 位姿解算器
    
    通过电表关键点（2D）和已知物理尺寸（3D）解算相机相对电表的位姿
    """
    
    def __init__(self, camera_matrix, dist_coeffs, meter_size=(0.2, 0.3), logger=None):
        """
        初始化位姿解算器
        
        Args:
            camera_matrix: 3x3 相机内参矩阵
            dist_coeffs: 畸变系数
            meter_size: (width, height) 电表物理尺寸（米）
            logger: ROS2 节点日志器（可选）
        """
        self.K = np.array(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64)
        self.meter_width, self.meter_height = meter_size
        self.logger = logger
        
        # 构建电表 3D 点
        w, h = self.meter_width / 2, self.meter_height / 2
        self.object_points = np.array([
            [-w, -h, 0],   # 左下
            [w, -h, 0],    # 右下
            [w, h, 0],     # 右上
            [-w, h, 0]     # 左上
        ], dtype=np.float64)
        
        self.pose_history = []
        self.history_size = 5
        
        self._total_time = 0.0
        self._count = 0
    
    def _loginfo(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")
    
    def _logerr(self, msg):
        if self.logger:
            self.logger.error(msg)
        else:
            print(f"[ERROR] {msg}")
    
    def solve(self, bbox, image_shape, keypoints=None):
        """解算位姿"""
        if keypoints is not None and len(keypoints) == 4:
            return self.solve_with_keypoints(keypoints, image_shape)
        
        start_time = time.time()
        
        x1, y1, x2, y2 = bbox
        
        image_points = np.array([
            [x1, y2],
            [x2, y2],
            [x2, y1],
            [x1, y1]
        ], dtype=np.float64)
        
        try:
            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.K,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if not success:
                return {'success': False}
            
            distance = np.linalg.norm(tvec)
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
            
            elapsed = (time.time() - start_time) * 1000
            self._total_time += elapsed
            self._count += 1
            avg_time = self._total_time / self._count if self._count > 0 else 0
            
            self._loginfo(
                f"[PnP] bbox=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}), "
                f"distance={distance:.3f}m, yaw={np.degrees(yaw):.2f}deg, "
                f"耗时={elapsed:.1f}ms, 平均={avg_time:.1f}ms"
            )
            
            return self._smooth_pose(pose)
            
        except Exception as e:
            self._logerr(f"PnP 解算失败: {e}")
            return {'success': False}
    
    def solve_with_keypoints(self, keypoints, image_shape):
        """使用关键点解算位姿"""
        start_time = time.time()
        
        image_points = np.array([
            [keypoints[3][0], keypoints[3][1]],  # 左下
            [keypoints[2][0], keypoints[2][1]],  # 右下
            [keypoints[1][0], keypoints[1][1]],  # 右上
            [keypoints[0][0], keypoints[0][1]]   # 左上
        ], dtype=np.float64)
        
        try:
            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.K,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if not success:
                return {'success': False}
            
            distance = np.linalg.norm(tvec)
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
            
            elapsed = (time.time() - start_time) * 1000
            
            self._loginfo(
                f"[PnP] keypoints, distance={distance:.3f}m, "
                f"yaw={np.degrees(yaw):.2f}deg, 耗时={elapsed:.1f}ms"
            )
            
            return self._smooth_pose(pose)
            
        except Exception as e:
            self._logerr(f"PnP 关键点解算失败: {e}")
            return {'success': False}
    
    def _rotation_matrix_to_euler(self, R):
        """旋转矩阵转欧拉角 (ZYX 顺序)"""
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
        
        return z, y, x
    
    def _smooth_pose(self, pose):
        """滑动平均滤波"""
        self.pose_history.append(pose)
        if len(self.pose_history) > self.history_size:
            self.pose_history.pop(0)
        
        if len(self.pose_history) < 2:
            return pose
        
        smoothed = {
            'success': True,
            'distance': np.mean([p['distance'] for p in self.pose_history]),
            'yaw': np.mean([p['yaw'] for p in self.pose_history]),
            'pitch': np.mean([p['pitch'] for p in self.pose_history]),
            'roll': np.mean([p['roll'] for p in self.pose_history]),
            'tvec': pose['tvec'],
            'rvec': pose['rvec']
        }
        
        return smoothed
    
    def get_target_bbox_ratio(self, bbox, image_shape):
        """计算目标在画面中的占比"""
        x1, y1, x2, y2 = bbox
        bbox_width = x2 - x1
        image_width = image_shape[1]
        return bbox_width / image_width
    
    def get_center_offset(self, bbox, image_shape):
        """计算目标中心与画面中心的偏差"""
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        
        image_cx = image_shape[1] / 2
        image_cy = image_shape[0] / 2
        
        offset_x = (cx - image_cx) / image_shape[1] * 2
        offset_y = (cy - image_cy) / image_shape[0] * 2
        
        return offset_x, offset_y
