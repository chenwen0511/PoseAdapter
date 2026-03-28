#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运动控制器 - 抽象基类
"""

from abc import ABC, abstractmethod
import numpy as np
import time


class MotionState:
    """运动状态机"""
    IDLE = "idle"
    MOVING = "moving"
    ROTATING = "rotating"
    WAITING = "waiting"


class BaseMotionController(ABC):
    """
    运动控制器抽象基类
    
    定义机器狗控制的公共接口
    """

    def __init__(self,
                 target_distance=0.3,
                 target_ratio=0.5,
                 distance_tolerance=0.05,
                 angle_tolerance=2.0,
                 center_tolerance=0.05,
                 max_linear_speed=0.12,
                 min_linear_speed=0.12,
                 max_angular_speed=0.25,
                 step_distance=0.2,
                 step_angle=5.0):
        """初始化控制器"""
        self.target_distance = target_distance
        self.target_ratio = target_ratio
        self.distance_tolerance = distance_tolerance
        self.angle_tolerance = angle_tolerance
        self.center_tolerance = center_tolerance
        self.max_linear_speed = max_linear_speed
        self.min_linear_speed = min_linear_speed
        self.max_angular_speed = max_angular_speed
        self.step_distance = step_distance
        self.step_angle = step_angle

        # PID 参数
        self.kp_linear = 0.35
        self.ki_linear = 0.0
        self.kd_linear = 0.05
        self.kp_angular = 0.5
        self.ki_angular = 0.0
        self.kd_angular = 0.05

        # 状态
        self.is_positioned = False
        self.positioned_count = 0
        self.positioned_threshold = 10
        self.motion_state = MotionState.IDLE
        self.is_running = True
        self.sdk_initialized = False

    @abstractmethod
    def initialize(self):
        """初始化 SDK"""
        pass

    @abstractmethod
    def move_forward_distance(self, distance_m, timeout=10.0):
        """前进/后退指定距离"""
        pass

    @abstractmethod
    def turn_angle(self, angle_deg, timeout=10.0):
        """旋转指定角度"""
        pass

    @abstractmethod
    def stop(self):
        """停止运动"""
        pass

    @abstractmethod
    def stand_up(self):
        """站立"""
        pass

    @abstractmethod
    def stand_down(self):
        """趴下"""
        pass

    def compute_control(self, pose, bbox_ratio, center_offset):
        """计算控制指令"""
        raise NotImplementedError

    def _check_positioned(self, distance_error, angle_error, offset_x):
        """检查是否到位"""
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

    def _estimate_motion_duration(self, distance_error, angle_error):
        """预估运动持续时间"""
        if abs(distance_error) > self.distance_tolerance:
            avg_linear_speed = self.max_linear_speed * 0.6
            distance_duration = abs(distance_error) / avg_linear_speed
        else:
            distance_duration = 0

        if abs(angle_error) > self.angle_tolerance:
            avg_angular_speed = self.max_angular_speed * 0.6
            angle_duration = abs(np.radians(angle_error)) / avg_angular_speed
        else:
            angle_duration = 0

        duration = max(distance_duration, angle_duration) + 0.5
        return np.clip(duration, 0.5, 3.0)

    def _sleep(self, seconds):
        """sleep 兼容"""
        time.sleep(seconds)

    def get_status(self):
        """获取状态"""
        return {
            'is_positioned': self.is_positioned,
            'positioned_count': self.positioned_count,
            'mode': self.__class__.__name__
        }
