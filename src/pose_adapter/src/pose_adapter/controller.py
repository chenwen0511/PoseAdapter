#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
运动控制器 - 生成 cmd_vel 控制机器狗运动
"""

import rospy
from geometry_msgs.msg import Twist
import numpy as np


class MotionController:
    """
    运动控制器
    
    根据位姿误差生成控制指令，实现闭环控制
    """
    
    def __init__(self, 
                 target_distance=1.7,      # 目标距离（米）
                 target_ratio=0.65,         # 目标画面占比
                 distance_tolerance=0.25,   # 距离容差（米）
                 angle_tolerance=2.0,       # 角度容差（度）
                 center_tolerance=0.05,     # 中心偏差容差
                 max_linear_speed=0.3,      # 最大线速度（m/s）
                 max_angular_speed=0.5):    # 最大角速度（rad/s）
        """
        初始化控制器
        
        Args:
            target_distance: 目标距离（米）
            target_ratio: 目标画面占比（0-1）
            distance_tolerance: 距离容差
            angle_tolerance: 角度容差（度）
            center_tolerance: 中心偏差容差
            max_linear_speed: 最大线速度
            max_angular_speed: 最大角速度
        """
        self.target_distance = target_distance
        self.target_ratio = target_ratio
        self.distance_tolerance = distance_tolerance
        self.angle_tolerance = angle_tolerance
        self.center_tolerance = center_tolerance
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # PID 参数
        self.kp_linear = 0.5
        self.ki_linear = 0.0
        self.kd_linear = 0.1
        
        self.kp_angular = 0.8
        self.ki_angular = 0.0
        self.kd_angular = 0.1
        
        # 误差积分和微分
        self.error_int_linear = 0
        self.error_int_angular = 0
        self.error_prev_linear = 0
        self.error_prev_angular = 0
        
        # 状态
        self.is_positioned = False
        self.positioned_count = 0
        self.positioned_threshold = 10  # 连续多少帧达标才算到位
        
        # ROS 发布器
        self.cmd_vel_pub = None
        self._init_ros()
    
    def _init_ros(self):
        """初始化 ROS"""
        try:
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            rospy.loginfo("运动控制器已初始化，发布到 /cmd_vel")
        except Exception as e:
            rospy.logerr(f"ROS 初始化失败: {e}")
    
    def compute_control(self, pose, bbox_ratio, center_offset):
        """
        计算控制指令
        
        Args:
            pose: 位姿字典（来自 PoseSolver）
            bbox_ratio: 画面占比
            center_offset: (offset_x, offset_y) 中心偏差
            
        Returns:
            Twist: 控制指令
        """
        cmd = Twist()
        
        if not pose.get('success', False):
            rospy.logwarn("位姿解算失败，停止运动")
            return cmd
        
        distance = pose['distance']
        yaw = pose['yaw']
        offset_x, offset_y = center_offset
        
        # 距离控制（使用画面占比作为辅助）
        distance_error = distance - self.target_distance
        
        # 如果距离误差大，优先调整距离
        if abs(distance_error) > self.distance_tolerance:
            # 比例控制
            linear_vel = -self.kp_linear * distance_error
            # 限幅
            linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
            cmd.linear.x = linear_vel
        
        # 角度控制（偏航角 + 中心偏差）
        # 将中心偏差转换为角度误差
        angle_error = yaw + offset_x * 30  # 中心偏差贡献角度误差
        
        if abs(angle_error) > self.angle_tolerance:
            angular_vel = -self.kp_angular * np.radians(angle_error)
            angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
            cmd.angular.z = angular_vel
        
        # 检查是否到位
        self._check_positioned(distance_error, angle_error, offset_x)
        
        return cmd
    
    def _check_positioned(self, distance_error, angle_error, offset_x):
        """检查是否已经到位"""
        distance_ok = abs(distance_error) <= self.distance_tolerance
        angle_ok = abs(angle_error) <= self.angle_tolerance
        center_ok = abs(offset_x) <= self.center_tolerance
        
        if distance_ok and angle_ok and center_ok:
            self.positioned_count += 1
            if self.positioned_count >= self.positioned_threshold:
                self.is_positioned = True
        else:
            self.positioned_count = 0
            self.is_positioned = False
    
    def publish_control(self, cmd):
        """发布控制指令"""
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(cmd)
    
    def stop(self):
        """停止运动"""
        cmd = Twist()
        self.publish_control(cmd)
        rospy.loginfo("停止运动")
    
    def is_ready_for_ocr(self):
        """检查是否准备好进行 OCR"""
        return self.is_positioned
    
    def get_status(self):
        """获取控制器状态"""
        return {
            'is_positioned': self.is_positioned,
            'positioned_count': self.positioned_count
        }