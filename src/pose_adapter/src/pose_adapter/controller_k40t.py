#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
K40T 云台相机控制器 - 通过串口 SDK 控制
"""

import os
import rclpy
from rclpy.node import Node
import numpy as np
import time
from controller_base import BaseMotionController, MotionState


class K40TMotionController(Node, BaseMotionController):
    """
    K40T 云台相机控制器
    
    通过 K40T SDK 控制云台转动和变倍
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
                 step_angle=5.0,
                 serial_port="/dev/ttyUSB0",
                 serial_baudrate=115200):
        """初始化控制器"""
        Node.__init__(self, 'k40t_motion_controller')
        BaseMotionController.__init__(self, target_distance, target_ratio,
                                      distance_tolerance, angle_tolerance,
                                      center_tolerance, max_linear_speed,
                                      min_linear_speed, max_angular_speed,
                                      step_distance, step_angle)
        
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate
        
        # K40T SDK
        self.k40t = None
        
        # 当前状态
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_zoom = 1.0
        
        # 初始化 SDK
        self.initialize()
    
    def initialize(self):
        """初始化 K40T SDK"""
        try:
            # 尝试导入 k40t_sdk_python
            try:
                from k40t_sdk_python import K40T
            except ImportError:
                from k40t_sdk import K40T
            
            self.get_logger().info(f"[K40T] 连接串口: {self.serial_port}")
            self.k40t = K40T(port=self.serial_port, baudrate=self.serial_baudrate)
            
            if self.k40t.connect():
                self.sdk_initialized = True
                self.get_logger().info("[K40T] 已连接")
                
                # 回中
                self.k40t.gimbal.center()
                self._sleep(1)
                
                self.get_logger().info("[K40T] 初始化完成")
            else:
                raise RuntimeError("K40T 连接失败")
                
        except ImportError as e:
            self.get_logger().error(f"[K40T] SDK 导入失败: {e}")
            raise RuntimeError("K40T SDK 未安装，请 pip install k40t_sdk_python")
        except Exception as e:
            self.get_logger().error(f"[K40T] 初始化失败: {e}")
            raise RuntimeError(f"K40T 初始化失败: {e}")
    
    def move(self, yaw_dir: int = 2, pitch_dir: int = 2):
        """移动云台
        
        Args:
            yaw_dir: 偏航方向 (0=左, 1=右, 2=停止)
            pitch_dir: 俯仰方向 (0=下, 1=上, 2=停止)
        """
        if self.k40t:
            self.k40t.gimbal.move(yaw_dir, pitch_dir)
    
    def set_speed(self, yaw_speed: int, pitch_speed: int):
        """设置移动速度
        
        Args:
            yaw_speed: 偏航速度 (0-30)
            pitch_speed: 俯仰速度 (0-30)
        """
        if self.k40t:
            # K40T SDK: set_speed(pitch, yaw)
            self.k40t.gimbal.set_speed(pitch_speed, yaw_speed)
    
    def set_angle(self, yaw: float, pitch: float) -> bool:
        """设置绝对角度
        
        Args:
            yaw: 偏航角 (-180 ~ 180)
            pitch: 俯仰角 (-90 ~ 30)
        """
        if not self.k40t:
            return False
        
        yaw = max(-180, min(180, yaw))
        pitch = max(-90, min(30, pitch))
        
        self.current_yaw = yaw
        self.current_pitch = pitch
        
        return self.k40t.gimbal.set_angle(yaw=yaw, pitch=pitch)
    
    def center(self):
        """回中"""
        self.set_angle(0, 0)
    
    def look_down(self):
        """下视"""
        self.set_angle(0, -90)
    
    def zoom(self, factor: float, lens: int = 0) -> bool:
        """变倍
        
        Args:
            factor: 变倍值 (0.1 ~ 160)
            lens: 镜头 (0=主镜头)
        """
        if not self.k40t:
            return False
        
        factor = max(0.1, min(160, factor))
        self.current_zoom = factor
        
        return self.k40t.camera.zoom(factor, lens)
    
    def zoom_in(self):
        """放大"""
        self.zoom(self.current_zoom + 3.0)
    
    def zoom_out(self):
        """缩小"""
        self.zoom(max(1.0, self.current_zoom - 3.0))
    
    def zoom_stop(self):
        """停止变倍"""
        if self.k40t:
            self.k40t.camera.zoom_stop()
    
    # === 实现抽象基类接口 ===
    
    def move_forward_distance(self, distance_m, timeout=10.0):
        """前进/后退 → 通过变倍模拟
        
        注意：K40T 没有前进后退，用变倍替代
        - distance_m > 0: 目标太近，变小(zoom out)
        - distance_m < 0: 目标太远，变大(zoom in)
        """
        direction = "目标太近" if distance_m > 0 else "目标太远"
        self.get_logger().info(f"[K40T] {direction}，调整变倍")
        
        if distance_m > 0:
            # 目标太近，缩小
            self.zoom_out()
        else:
            # 目标太远，放大
            self.zoom_in()
        
        self._sleep(1)
        self.zoom_stop()
        
        return True
    
    def turn_angle(self, angle_deg, timeout=10.0):
        """旋转 → 云台左右转动 + 俯仰转动
        
        angle_deg > 0: 左转
        angle_deg < 0: 右转
        
        可选 pitch_offset 控制上下转动:
        pitch_offset > 0: 目标偏下，需要向上看
        pitch_offset < 0: 目标偏上，需要向下看
        """
        direction = "左转" if angle_deg > 0 else "右转"
        self.get_logger().info(f"[K40T] {direction} {abs(angle_deg):.1f}度")
        
        # 使用方向控制
        yaw_dir = 0 if angle_deg > 0 else 1
        self.set_speed(15, 15)
        self.move(yaw_dir, 2)  # 水平转动
        
        duration = abs(angle_deg) / 30  # 估计时间
        duration = min(duration, timeout)
        
        self._sleep(duration)
        self.stop()
        
        return True
    
    def adjust_pitch(self, pitch_offset: float, timeout=10.0):
        """调整俯仰角
        
        pitch_offset > 0: 目标偏下，向上看
        pitch_offset < 0: 目标偏上，向下看
        """
        direction = "上转" if pitch_offset > 0 else "下转"
        self.get_logger().info(f"[K40T] {direction} {abs(pitch_offset):.1f}度")
        
        # 使用方向控制
        pitch_dir = 1 if pitch_offset > 0 else 0
        self.set_speed(15, 15)
        self.move(2, pitch_dir)  # 垂直转动
        
        duration = abs(pitch_offset) / 30
        duration = min(duration, timeout)
        
        self._sleep(duration)
        self.stop()
        
        return True
    
    def stop(self):
        """停止云台运动"""
        self.is_running = False
        if self.k40t:
            self.k40t.gimbal.move_stop()
            self.k40t.camera.zoom_stop()
            self.get_logger().info("[K40T] 停止运动")
    
    def stand_up(self):
        """站立 → K40T 回中"""
        self.center()
        self.get_logger().info("[K40T] 回中")
    
    def stand_down(self):
        """趴下 → K40T 下视"""
        self.look_down()
        self.get_logger().info("[K40T] 下视")
    
    def balance_stand(self):
        """平衡站立 → 回中"""
        self.center()
        self.get_logger().info("[K40T] 回中")
