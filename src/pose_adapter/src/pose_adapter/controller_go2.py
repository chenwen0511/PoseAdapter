#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Go2 运动控制器 - 通过 Unitree SDK high_level 接口控制
"""

import os
import rclpy
from rclpy.node import Node
import numpy as np
import time
from controller_base import BaseMotionController, MotionState


class Go2MotionController(Node, BaseMotionController):
    """
    Go2 运动控制器
    
    通过 Unitree SDK 控制机器狗
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
                 interface_name="eth0",
                 disable_obstacle_avoidance_on_start=True,
                 use_classic_walk=False,
                 speed_level=0):
        """初始化控制器"""
        Node.__init__(self, 'go2_motion_controller')
        BaseMotionController.__init__(self, target_distance, target_ratio,
                                      distance_tolerance, angle_tolerance,
                                      center_tolerance, max_linear_speed,
                                      min_linear_speed, max_angular_speed,
                                      step_distance, step_angle)
        
        self.interface_name = interface_name
        self.disable_obstacle_avoidance_on_start = disable_obstacle_avoidance_on_start
        self.use_classic_walk = use_classic_walk
        self.speed_level = speed_level
        
        # SDK 客户端
        self.sport_client = None
        
        # 机器人状态
        self.robot_state = None
        self.current_euler = [0.0, 0.0, 0.0]
        
        # 初始化 SDK
        self.initialize()
    
    def initialize(self):
        """初始化 Unitree SDK"""
        try:
            cyclonedds = os.environ.get("CYCLONEDDS_HOME", "")
            self.get_logger().info(
                f"[Go2 SDK] 环境: CYCLONEDDS_HOME={cyclonedds if cyclonedds else '(未设置)'}, 网口: {self.interface_name}"
            )
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.core.channel import ChannelSubscriber

            self.get_logger().info("[Go2 SDK] Step 1: 初始化 DDS 通道")
            ChannelFactoryInitialize(0, self.interface_name)
            self.get_logger().info(f"[Go2 SDK] DDS 通道已初始化")

            self.get_logger().info("[Go2 SDK] Step 2: 创建 SportClient")
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()

            self.sdk_initialized = True
            self.get_logger().info("[Go2 SDK] SportClient 已初始化")

            self._subscribe_pose()

            if self.disable_obstacle_avoidance_on_start:
                self._try_disable_obstacle_avoidance()

            self._set_gentle_gait()

            self.get_logger().info("[Go2 SDK] 初始化完成")

        except ImportError as e:
            self.get_logger().error(f"[Go2 SDK] 导入失败: {e}")
            raise RuntimeError("Go2 SDK 初始化失败")
        except Exception as e:
            self.get_logger().error(f"[Go2 SDK] 初始化失败: {e}")
            raise RuntimeError("Go2 SDK 初始化失败")
    
    def _subscribe_pose(self):
        """订阅机器人姿态状态"""
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            self.pose_sub = ChannelSubscriber("rt/SportModeState", SportModeState_)
            self.pose_sub.Init()
            self.get_logger().info("已订阅 SportModeState 主题")
        except Exception as e:
            self.get_logger().warn(f"姿态订阅失败: {e}")
            self.pose_sub = None
    
    def _try_disable_obstacle_avoidance(self):
        """尝试关闭避障"""
        try:
            from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
            client = ObstaclesAvoidClient()
            client.Init()
            client.SwitchSet(False)
            self.get_logger().info("[Go2 SDK] 已关闭避障")
        except Exception as e:
            self.get_logger().warn(f"[Go2 SDK] 关闭避障失败: {e}")
    
    def _set_gentle_gait(self):
        """设置柔和步态"""
        try:
            if self.use_classic_walk and hasattr(self.sport_client, 'ClassicWalk'):
                self.sport_client.ClassicWalk(True)
            if hasattr(self.sport_client, 'SpeedLevel'):
                self.sport_client.SpeedLevel(self.speed_level)
            self.get_logger().info(f"[Go2 SDK] 步态设置完成")
        except Exception as e:
            self.get_logger().warn(f"[Go2 SDK] 设置步态失败: {e}")
    
    def _get_pose_subscriber_state(self):
        """获取机器人状态"""
        if not hasattr(self, 'pose_sub') or self.pose_sub is None:
            return None
        getter = getattr(self.pose_sub, 'Get', None) or getattr(self.pose_sub, 'get', None)
        if callable(getter):
            try:
                return getter()
            except Exception:
                return None
        return None
    
    def _update_current_pose(self):
        """更新当前姿态"""
        state = self._get_pose_subscriber_state()
        if state:
            self.current_euler = [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]]
    
    def ensure_robot_ready(self):
        """确保机器狗已站立并解锁"""
        if not self.sport_client or not self.sdk_initialized:
            return True
        
        state = self._get_pose_subscriber_state()
        if state is None:
            self.get_logger().warn("[Go2 SDK] 无法获取机器人状态")
            try:
                self.sport_client.BalanceStand()
                self._sleep(2.5)
                self.sport_client.Move(0.0, 0.0, 0.0)
            except Exception as e:
                self.get_logger().warn(f"初始化失败: {e}")
            return True
        
        # 检查运动模式
        mode = getattr(state, 'mode', None)
        gait_status = getattr(state, 'gait_status', 0)
        is_unlocked = mode == 9
        
        if not is_unlocked:
            self.get_logger().warn("[Go2 SDK] 机器人未进入运动模式")
            try:
                self.sport_client.BalanceStand()
                self._sleep(2.0)
                self.sport_client.Move(0.0, 0.0, 0.0)
            except Exception as e:
                self.get_logger().warn(f"切换运动模式失败: {e}")
        
        return True
    
    def move_forward_distance(self, distance_m, timeout=10.0):
        """前进指定距离"""
        direction = "前进" if distance_m > 0 else "后退"
        self.get_logger().info(f"[Go2] {direction} {abs(distance_m)*100:.1f}cm")
        
        if not self.ensure_robot_ready():
            return False
        
        speed = max(0.1, min(self.max_linear_speed, 0.15))
        duration = abs(distance_m) / speed
        duration = min(duration, timeout)
        
        vx = speed if distance_m > 0 else -speed
        
        self.sport_client.Move(vx, 0.0, 0.0)
        self._sleep(duration)
        self.sport_client.StopMove()
        
        return True
    
    def turn_angle(self, angle_deg, timeout=10.0):
        """旋转指定角度"""
        direction = "左转" if angle_deg > 0 else "右转"
        self.get_logger().info(f"[Go2] {direction} {abs(angle_deg):.1f}度")
        
        if not self.ensure_robot_ready():
            return False
        
        angular_speed = min(self.max_angular_speed, 0.2)
        duration = abs(np.radians(angle_deg)) / angular_speed
        duration = min(duration, timeout)
        
        vyaw = angular_speed if angle_deg > 0 else -angular_speed
        
        self.sport_client.Move(0.0, 0.0, vyaw)
        self._sleep(duration)
        self.sport_client.StopMove()
        
        return True
    
    def stop(self):
        """停止运动"""
        self.is_running = False
        if self.sport_client:
            self.sport_client.StopMove()
            self.get_logger().info("[Go2] 停止运动")
    
    def stand_up(self):
        """站立"""
        if self.sport_client:
            self.sport_client.StandUp()
            self.get_logger().info("[Go2] 站立")
    
    def stand_down(self):
        """趴下"""
        if self.sport_client:
            self.sport_client.StandDown()
            self.get_logger().info("[Go2] 趴下")
    
    def balance_stand(self):
        """平衡站立"""
        if self.sport_client:
            self.sport_client.BalanceStand()
            self.get_logger().info("[Go2] 平衡站立")
