#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZSI-1 运动控制器 - 通过 zsibot_sdk 控制
"""

import os
import rclpy
from rclpy.node import Node
import numpy as np
import time
import threading
from controller_base import BaseMotionController, MotionState


# SDK 单例
_zsi_app_instance = None
_zsi_app_lock = threading.Lock()


class ZSI1MotionController(Node, BaseMotionController):
    """
    ZSI-1 运动控制器
    
    通过 zsibot_sdk 控制机器狗
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
                 local_ip='192.168.234.15',
                 local_port=43988,
                 dog_ip='192.168.234.1'):
        """初始化控制器"""
        Node.__init__(self, 'zsi1_motion_controller')
        BaseMotionController.__init__(self, target_distance, target_ratio,
                                      distance_tolerance, angle_tolerance,
                                      center_tolerance, max_linear_speed,
                                      min_linear_speed, max_angular_speed,
                                      step_distance, step_angle)
        
        self.local_ip = local_ip
        self.local_port = local_port
        self.dog_ip = dog_ip
        
        # SDK 客户端
        self.zsi_client = None
        
        # 速度限制参数
        self._zsi_deadzone_vx = 0.05
        self._zsi_deadzone_vy = 0.05
        self._zsi_deadzone_wz = 0.02
        self._zsi_max_vx = 2.0
        self._zsi_max_vy = 2.0
        self._zsi_max_wz = 3.0
        self._zsi_move_lock = threading.Lock()
        
        # 状态
        self.is_robot_standing = False
        self.is_robot_unlocked = False
        
        # 初始化 SDK
        self.initialize()
    
    def initialize(self):
        """初始化 ZSI-1 SDK"""
        try:
            import sys
            import platform
            
            arch = platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64')
            env_zsi_root = os.environ.get('ZSI_SDK_ROOT')
            if env_zsi_root:
                sdk_root = os.path.abspath(os.path.expanduser(env_zsi_root))
            else:
                sdk_root = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'zsibot_sdk'))
            
            sdk_path = os.path.join(sdk_root, f'lib/zsl-1/{arch}')
            
            if not os.path.isdir(sdk_path):
                raise RuntimeError(f"[ZSI-1 SDK] 未找到目录: {sdk_path}")
            
            if sdk_path not in sys.path:
                sys.path.insert(0, sdk_path)
            
            import mc_sdk_zsl_1_py
            
            global _zsi_app_instance
            if _zsi_app_instance is None:
                with _zsi_app_lock:
                    if _zsi_app_instance is None:
                        _zsi_app_instance = mc_sdk_zsl_1_py.HighLevel()
                        _zsi_app_instance.initRobot(self.local_ip, self.local_port, self.dog_ip)
            
            self.zsi_client = _zsi_app_instance
            self.sdk_initialized = True
            self.get_logger().info(f"[ZSI-1 SDK] 已初始化，连接 {self.local_ip}:{self.local_port} -> {self.dog_ip}")
            
            # 预检
            self._preflight_check()
            
            self.get_logger().info("[ZSI-1 SDK] 初始化完成")
            
        except ImportError as e:
            self.get_logger().error(f"[ZSI-1 SDK] 导入失败: {e}")
            raise RuntimeError("ZSI-1 SDK 初始化失败")
        except Exception as e:
            self.get_logger().error(f"[ZSI-1 SDK] 初始化失败: {e}")
            raise RuntimeError("ZSI-1 SDK 初始化失败")
    
    def _preflight_check(self):
        """预检"""
        self.get_logger().info("[ZSI-1 SDK] 预检开始")
        for i in range(3):
            try:
                self.zsi_client.standUp()
                self._sleep(0.5)
                self.zsi_client.move(0.0, 0.0, 0.0)
                self._sleep(0.2)
                self.get_logger().info(f"[ZSI-1 SDK] 预检通过")
                self.is_robot_standing = True
                self.is_robot_unlocked = True
                return
            except Exception as e:
                self.get_logger().warn(f"[ZSI-1 SDK] 预检失败 (attempt={i+1}): {e}")
                self._sleep(0.5)
        
        raise RuntimeError("[ZSI-1 SDK] 预检失败")
    
    def _clamp_speed(self, vx, vy, wz):
        """速度限制"""
        # 死区
        if -self._zsi_deadzone_wz < wz < self._zsi_deadzone_wz:
            wz = 0.0
        if -self._zsi_deadzone_vx < vx < self._zsi_deadzone_vx:
            vx = 0.0
        if -self._zsi_deadzone_vy < vy < self._zsi_deadzone_vy:
            vy = 0.0
        # 饱和
        wz = max(-self._zsi_max_wz, min(self._zsi_max_wz, wz))
        vx = max(-self._zsi_max_vx, min(self._zsi_max_vx, vx))
        vy = max(-self._zsi_max_vy, min(self._zsi_max_vy, vy))
        return vx, vy, wz
    
    def move_forward_distance(self, distance_m, timeout=10.0):
        """前进指定距离"""
        direction = "前进" if distance_m > 0 else "后退"
        self.get_logger().info(f"[ZSI-1] {direction} {abs(distance_m)*100:.1f}cm")
        
        if not self.is_robot_standing:
            self.zsi_client.standUp()
            self._sleep(2)
            self.is_robot_standing = True
        
        speed = max(0.1, min(self.max_linear_speed, 0.15))
        duration = abs(distance_m) / speed
        duration = min(duration, timeout)
        
        vx = speed if distance_m > 0 else -speed
        vx, vy, wz = self._clamp_speed(vx, 0.0, 0.0)
        
        with self._zsi_move_lock:
            self.zsi_client.move(vx, vy, wz)
            self._sleep(duration)
            self.zsi_client.move(0, 0, 0)
        
        return True
    
    def turn_angle(self, angle_deg, timeout=10.0):
        """旋转指定角度"""
        direction = "左转" if angle_deg > 0 else "右转"
        self.get_logger().info(f"[ZSI-1] {direction} {abs(angle_deg):.1f}度")
        
        if not self.is_robot_standing:
            self.zsi_client.standUp()
            self._sleep(2)
            self.is_robot_standing = True
        
        angular_speed = min(self.max_angular_speed, 0.2)
        duration = abs(np.radians(angle_deg)) / angular_speed
        duration = min(duration, timeout)
        
        # ZSI-1 转向符号相反
        wz = -angular_speed if angle_deg > 0 else angular_speed
        vx, vy, wz = self._clamp_speed(0.0, 0.0, wz)
        
        with self._zsi_move_lock:
            self.zsi_client.move(vx, vy, wz)
            self._sleep(duration)
            self.zsi_client.move(0, 0, 0)
        
        return True
    
    def stop(self):
        """停止运动"""
        self.is_running = False
        if self.zsi_client:
            with self._zsi_move_lock:
                self.zsi_client.move(0, 0, 0)
            self.get_logger().info("[ZSI-1] 停止运动")
    
    def stand_up(self):
        """站立"""
        if self.zsi_client:
            self.zsi_client.standUp()
            self.get_logger().info("[ZSI-1] 站立")
            self.is_robot_standing = True
            self.is_robot_unlocked = True
    
    def stand_down(self):
        """趴下"""
        if self.zsi_client:
            self.zsi_client.lieDown()
            self.get_logger().info("[ZSI-1] 趴下")
            self.is_robot_standing = False
