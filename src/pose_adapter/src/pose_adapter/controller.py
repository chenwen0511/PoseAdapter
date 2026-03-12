#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
运动控制器 - 生成控制指令实现机器狗位姿调整

支持两种模式：
1. cmd_vel 模式：通过 ROS cmd_vel 控制（原有方式）
2. high_level 模式：通过 Unitree SDK high_level 接口控制
"""

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time


class MotionController:
    """
    运动控制器
    
    根据位姿误差生成控制指令，实现闭环控制
    支持 cmd_vel 和 high_level SDK 两种模式
    """
    
    def __init__(self, 
                 target_distance=1.7,      # 目标距离（米）
                 target_ratio=0.65,         # 目标画面占比
                 distance_tolerance=0.05,   # 距离容差（米）
                 angle_tolerance=2.0,       # 角度容差（度）
                 center_tolerance=0.05,     # 中心偏差容差
                 max_linear_speed=0.3,      # 最大线速度（m/s）
                 max_angular_speed=0.5,     # 最大角速度（rad/s）
                 use_high_level_sdk=False,  # 是否使用 high_level SDK
                 interface_name="eth0"):    # 网络接口名称
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
            use_high_level_sdk: 是否使用 Unitree SDK high_level 接口
            interface_name: 网络接口名称（如 eth0, enp2s0 等）
        """
        self.target_distance = target_distance
        self.target_ratio = target_ratio
        self.distance_tolerance = distance_tolerance
        self.angle_tolerance = angle_tolerance
        self.center_tolerance = center_tolerance
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.use_high_level_sdk = use_high_level_sdk
        self.interface_name = interface_name
        
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
        
        # 当前姿态（用于 high_level 模式）
        self.current_euler = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.target_euler = [0.0, 0.0, 0.0]
        
        # ROS 发布器
        self.cmd_vel_pub = None
        
        # Unitree SDK 客户端
        self.sport_client = None
        self.sdk_initialized = False
        
        # 初始化
        if self.use_high_level_sdk:
            self._init_high_level_sdk()
        else:
            self._init_ros()
    
    def _init_ros(self):
        """初始化 ROS"""
        try:
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            rospy.loginfo("运动控制器已初始化（cmd_vel 模式），发布到 /cmd_vel")
        except Exception as e:
            rospy.logerr(f"ROS 初始化失败: {e}")
    
    def _init_high_level_sdk(self):
        """初始化 Unitree SDK high_level 接口"""
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
            from unitree_sdk2py.core.channel import ChannelSubscriber
            
            # 初始化 DDS 通道
            ChannelFactoryInitialize(0, self.interface_name)
            rospy.loginfo(f"Unitree SDK DDS 通道已初始化（接口: {self.interface_name}）")
            
            # 创建 SportClient
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            
            self.sdk_initialized = True
            rospy.loginfo("Unitree SDK SportClient 已初始化（high_level 模式）")
            
            # 启动姿态订阅
            self._subscribe_pose()
            
        except ImportError as e:
            rospy.logerr(f"Unitree SDK 导入失败: {e}")
            rospy.logwarn("回退到 cmd_vel 模式")
            self.use_high_level_sdk = False
            self._init_ros()
        except Exception as e:
            rospy.logerr(f"Unitree SDK 初始化失败: {e}")
            rospy.logwarn("回退到 cmd_vel 模式")
            self.use_high_level_sdk = False
            self._init_ros()
    
    def _subscribe_pose(self):
        """订阅机器人姿态状态"""
        try:
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
            from unitree_sdk2py.core.channel import ChannelSubscriber
            
            self.pose_sub = ChannelSubscriber(
                "rt/SportModeState",
                unitree_go_msg_dds__SportModeState_
            )
            self.pose_sub.Init()
            rospy.loginfo("已订阅 SportModeState 主题")
        except Exception as e:
            rospy.logwarn(f"姿态订阅失败: {e}")
    
    def _update_current_pose(self):
        """更新当前姿态"""
        if hasattr(self, 'pose_sub') and self.pose_sub.HasReceived():
            state = self.pose_sub.Get()
            if state:
                # 从状态中获取欧拉角 (roll, pitch, yaw)
                self.current_euler = [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]]
    
    def compute_control(self, pose, bbox_ratio, center_offset):
        """
        计算控制指令
        
        Args:
            pose: 位姿字典（来自 PoseSolver）
            bbox_ratio: 画面占比
            center_offset: (offset_x, offset_y) 中心偏差
            
        Returns:
            控制指令 (cmd_vel Twist 或 None 表示使用 SDK)
        """
        if not pose.get('success', False):
            rospy.logwarn("位姿解算失败，停止运动")
            if self.use_high_level_sdk and self.sport_client:
                self.sport_client.StopMove()
            return Twist()
        
        distance = pose['distance']
        yaw = pose['yaw']
        offset_x, offset_y = center_offset
        
        if self.use_high_level_sdk:
            return self._compute_high_level_control(distance, yaw, offset_x)
        else:
            return self._compute_cmd_vel_control(distance, yaw, offset_x)
    
    def _compute_high_level_control(self, distance, yaw, offset_x):
        """
        使用 high_level SDK 计算控制指令
        
        主要使用 Euler 调整姿态 + Move 控制移动
        """
        # 更新当前姿态
        self._update_current_pose()
        
        # 距离控制
        distance_error = distance - self.target_distance
        
        # 角度控制
        angle_error = yaw + offset_x * 30  # 中心偏差贡献角度误差
        
        # 判断是否到位
        distance_ok = abs(distance_error) <= self.distance_tolerance
        angle_ok = abs(angle_error) <= self.angle_tolerance
        center_ok = abs(offset_x) <= self.center_tolerance
        
        self._check_positioned(distance_error, angle_error, offset_x)
        
        if not self.sdk_initialized or not self.sport_client:
            return None
        
        # 根据误差决定使用哪种控制方式，并增加日志便于观测
        if not distance_ok:
            # 距离不在范围内，使用 Move 控制
            linear_vel = np.clip(-self.kp_linear * distance_error,
                                 -self.max_linear_speed, self.max_linear_speed)
            rospy.loginfo_throttle(
                1.0,
                f"[Go2 SDK] Move 距离控制: "
                f"distance={distance:.3f}m, target={self.target_distance:.3f}m, "
                f"error={distance_error:.3f}m, vx={linear_vel:.3f} m/s"
            )
            # x 方向速度（前进/后退）
            self.sport_client.Move(linear_vel, 0.0, 0.0)
        elif not angle_ok:
            # 角度不在范围内，使用 Move 旋转
            angular_vel = np.clip(-self.kp_angular * np.radians(angle_error),
                                  -self.max_angular_speed, self.max_angular_speed)
            rospy.loginfo_throttle(
                1.0,
                f"[Go2 SDK] Move 角度控制: "
                f"yaw_err={angle_error:.2f}deg, wz={angular_vel:.3f} rad/s"
            )
            self.sport_client.Move(0.0, 0.0, angular_vel)
        else:
            # 到位后停止移动
            rospy.loginfo_throttle(
                5.0,
                "[Go2 SDK] 目标已到位，发送 StopMove() 停止运动"
            )
            self.sport_client.StopMove()
            
            # 可以使用 Euler 调整姿态（微调）
            # self._adjust_euler(offset_x, offset_y)
        
        # 返回 None 表示已通过 SDK 发送指令
        return None
    
    def _adjust_euler(self, offset_x, offset_y):
        """
        使用 Euler 接口调整姿态
        
        Args:
            offset_x: 水平中心偏差
            offset_y: 垂直中心偏差
        """
        if not self.sport_client:
            return
        
        # 将中心偏差转换为微调姿态
        # roll: 左右倾斜, pitch: 前后倾斜, yaw: 旋转
        roll_adjust = 0.0
        pitch_adjust = 0.0  # 可以根据需要调整
        
        # 保持当前姿态，只做微调
        current_roll = self.current_euler[0]
        current_pitch = self.current_euler[1]
        
        # 可以选择使用 Euler 设置绝对姿态
        # self.sport_client.Euler(current_roll + roll_adjust, 
        #                        current_pitch + pitch_adjust, 
        #                        self.current_euler[2])
    
    def _compute_cmd_vel_control(self, distance, yaw, offset_x):
        """
        使用 cmd_vel 计算控制指令
        """
        cmd = Twist()
        
        distance_error = distance - self.target_distance
        
        # 距离控制
        if abs(distance_error) > self.distance_tolerance:
            linear_vel = -self.kp_linear * distance_error
            linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
            cmd.linear.x = linear_vel
        
        # 角度控制
        angle_error = yaw + offset_x * 30
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
        if self.use_high_level_sdk:
            # SDK 模式下，指令已在 compute_control 中发送
            pass
        elif self.cmd_vel_pub:
            self.cmd_vel_pub.publish(cmd)
    
    def stop(self):
        """停止运动"""
        if self.use_high_level_sdk and self.sport_client:
            self.sport_client.StopMove()
            rospy.loginfo("停止运动 (SDK)")
        else:
            cmd = Twist()
            self.publish_control(cmd)
            rospy.loginfo("停止运动 (cmd_vel)")
    
    def stand_up(self):
        """站立"""
        if self.use_high_level_sdk and self.sport_client:
            self.sport_client.StandUp()
            rospy.loginfo("站立 (SDK)")
    
    def stand_down(self):
        """趴下"""
        if self.use_high_level_sdk and self.sport_client:
            self.sport_client.StandDown()
            rospy.loginfo("趴下 (SDK)")
    
    def balance_stand(self):
        """平衡站立"""
        if self.use_high_level_sdk and self.sport_client:
            self.sport_client.BalanceStand()
            rospy.loginfo("平衡站立 (SDK)")
    
    def is_ready_for_ocr(self):
        """检查是否准备好进行 OCR"""
        return self.is_positioned
    
    def get_status(self):
        """获取控制器状态"""
        status = {
            'is_positioned': self.is_positioned,
            'positioned_count': self.positioned_count,
            'mode': 'high_level_sdk' if self.use_high_level_sdk else 'cmd_vel'
        }
        
        if self.use_high_level_sdk and self.sdk_initialized:
            status['euler'] = self.current_euler
        
        return status


class HighLevelMotionController:
    """
    独立的 High-Level 控制器
    
    直接使用 Unitree SDK 进行控制，不依赖 ROS
    适用于不需要 ROS 的场景
    """
    
    def __init__(self, interface_name="eth0"):
        """
        初始化
        
        Args:
            interface_name: 网络接口名称
        """
        self.interface_name = interface_name
        self.sport_client = None
        self.current_euler = [0.0, 0.0, 0.0]
        
        self._init_sdk()
    
    def _init_sdk(self):
        """初始化 SDK"""
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            
            ChannelFactoryInitialize(0, self.interface_name)
            
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            
            print("Unitree SDK 初始化成功")
            
        except ImportError as e:
            print(f"Unitree SDK 导入失败: {e}")
            raise
    
    def move(self, vx=0.0, vy=0.0, vyaw=0.0):
        """
        速度移动控制
        
        Args:
            vx: 前进速度 (m/s)
            vy: 横向速度 (m/s)
            vyaw: 旋转速度 (rad/s)
        """
        if self.sport_client:
            return self.sport_client.Move(vx, vy, vyaw)
    
    def stop(self):
        """停止移动"""
        if self.sport_client:
            self.sport_client.StopMove()
    
    def stand_up(self):
        """站立"""
        if self.sport_client:
            self.sport_client.StandUp()
    
    def stand_down(self):
        """趴下"""
        if self.sport_client:
            self.sport_client.StandDown()
    
    def balance_stand(self):
        """平衡站立"""
        if self.sport_client:
            self.sport_client.BalanceStand()
    
    def euler(self, roll, pitch, yaw):
        """
        设置姿态
        
        Args:
            roll: 翻滚角 (弧度)
            pitch: 俯仰角 (弧度)
            yaw: 偏航角 (弧度)
        """
        if self.sport_client:
            return self.sport_client.Euler(roll, pitch, yaw)
    
    def get_pose(self):
        """获取当前姿态"""
        return self.current_euler
    
    def damp(self):
        """阻尼模式"""
        if self.sport_client:
            self.sport_client.Damp()
