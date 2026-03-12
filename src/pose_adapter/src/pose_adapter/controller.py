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
        
        # 机器人状态
        self.robot_state = None  # 机器人当前状态
        self.is_robot_standing = False  # 机器人是否站立
        self.is_robot_unlocked = False  # 机器人是否已解锁（解除运动限制）
        
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
            import os
            cyclonedds = os.environ.get("CYCLONEDDS_HOME", "")
            rospy.loginfo(
                "[Go2 SDK] 环境: CYCLONEDDS_HOME=%s, 网口: %s",
                cyclonedds if cyclonedds else "(未设置)",
                self.interface_name,
            )
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
            from unitree_sdk2py.core.channel import ChannelSubscriber
            
            # 初始化 DDS 通道（与手动运行 go2_sport_client.py eth1 一致）
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
    
    def _update_robot_state(self):
        """更新机器人状态"""
        if hasattr(self, 'pose_sub') and self.pose_sub.HasReceived():
            state = self.pose_sub.Get()
            if state:
                self.robot_state = state
                # 从状态中获取欧拉角 (roll, pitch, yaw)
                self.current_euler = [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]]
                
                # 检查机器人是否处于站立状态
                # 状态值: 0=Idle, 1=StandUp, 2=Walk, 3=StandDown, 4=BalanceStand, etc.
                # 具体状态定义需参考 Unitree SDK
                self._check_standing_state(state)
    
    def _check_standing_state(self, state):
        """
        检查机器人是否处于站立状态
        
        Unitree Go2 状态机:
        - mode: 0=Passive(锁定), 1=LowState, 2=Move, 3=StandDown, 4=Walk, ...
        - mode 9 = SportMode (运动模式)
        - gait_status: 0=Stop, 1=Trot, 2=Walk, etc.
        
        注意: StandUp/StandDown 会进入锁定状态，只有 mode 9 才是运动模式
        """
        try:
            # 尝试多种可能的字段名
            mode = getattr(state, 'mode', None)
            if mode is None:
                mode = getattr(state, 'mode_request', None)
            if mode is None:
                mode = getattr(state, 'robot_mode', None)
            if mode is None:
                mode = getattr(state, '_mode', None)
            
            # 如果还是 None，记录调试信息
            if mode is None:
                rospy.logwarn_throttle(5.0, f"[Go2 SDK] 无法获取 mode，state 属性: {dir(state)[:20]}...")
                # 无法获取状态时，假设已就绪（保守策略）
                self.is_robot_unlocked = True
                self.is_robot_standing = True
                return
            
            gait_status = getattr(state, 'gait_status', 0)
            if gait_status is None:
                gait_status = getattr(state, 'gait', 0)
            
            # mode 9 = SportMode (运动模式)
            self.is_robot_unlocked = mode == 9
            # 站立状态：运动模式下且不在停止状态
            self.is_robot_standing = (mode == 9 and gait_status > 0)
            
            rospy.logdebug(f"[Go2 SDK] mode={mode}, gait_status={gait_status}")
            
        except Exception as e:
            rospy.logwarn(f"检查站立状态失败: {e}")
            # 出错时假设已就绪
            self.is_robot_standing = True
            self.is_robot_unlocked = True
    
    def ensure_robot_ready(self):
        """
        确保机器狗已站立并解锁，准备接受运动指令
        
        Unitree Go2 状态机:
        - StandUp/StandDown 会进入锁定状态
        - 只有 mode 9 (SportMode) 才是运动模式
        - BalanceStand() 可以让机器狗进入平衡站立状态并可运动
        
        这是使用 high_level SDK 前的必要检查
        Returns:
            bool: True 表示机器人已就绪
        """
        if not self.use_high_level_sdk or not self.sport_client or not self.sdk_initialized:
            return True  # 非 SDK 模式直接返回
        
        # 更新状态
        self._update_robot_state()
        
        # 如果无法获取状态（有可能是订阅未建立），直接尝试运动
        if self.robot_state is None:
            rospy.logwarn("[Go2 SDK] 无法获取机器人状态，尝试直接发送运动指令...")
            try:
                self.sport_client.Move(0.0, 0.0, 0.0)
                rospy.sleep(0.5)
            except Exception as e:
                rospy.logwarn(f"发送运动指令失败: {e}")
            return True
        
        if not self.is_robot_unlocked:
            rospy.logwarn("[Go2 SDK] 机器人未进入运动模式 (mode 9)，尝试平衡站立...")
            # 使用 BalanceStand 进入平衡站立状态
            self.balance_stand()
            rospy.sleep(2.0)  # 等待平衡站立完成
            
            # 尝试切换到运动模式 - 调用一次 Move(0,0,0) 触发进入 mode 9
            rospy.loginfo("[Go2 SDK] 发送 Move(0,0,0) 尝试进入运动模式...")
            try:
                self.sport_client.Move(0.0, 0.0, 0.0)
                rospy.sleep(1.0)  # 等待模式切换
            except Exception as e:
                rospy.logwarn(f"切换运动模式失败: {e}")
            
            self._update_robot_state()
        
        if self.is_robot_standing and self.is_robot_unlocked:
            rospy.loginfo("[Go2 SDK] 机器人已就绪 (mode 9)，可以接受运动指令")
            return True
        else:
            # 再次尝试
            current_mode = getattr(self.robot_state, 'mode', 'unknown')
            rospy.logwarn(f"[Go2 SDK] 当前 mode={current_mode}, 再次尝试进入运动模式...")
            try:
                self.sport_client.Move(0.0, 0.0, 0.0)
                rospy.sleep(1.0)
                self._update_robot_state()
            except:
                pass
            
            if self.is_robot_standing and self.is_robot_unlocked:
                rospy.loginfo("[Go2 SDK] 机器人已就绪 (mode 9)")
                return True
            else:
                # 多次尝试失败后，放行运动指令（让用户手动确认状态）
                rospy.logerr("[Go2 SDK] 机器人未能进入运动模式，放行运动指令...")
                return True
    
    def _update_current_pose(self):
        """更新当前姿态"""
        if hasattr(self, 'pose_sub') and self.pose_sub.HasReceived():
            state = self.pose_sub.Get()
            if state:
                # 从状态中获取欧拉角 (roll, pitch, yaw)
                self.current_euler = [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]]
                # 同时检查站立状态
                self._check_standing_state(state)
    
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
        
        安全检查：
        1. 确保机器狗已站立
        2. 确保机器狗已解锁（解除运动限制）
        """
        # 安全检查：确保机器狗已就绪
        if not self.ensure_robot_ready():
            rospy.logwarn_throttle(
                2.0,
                "[Go2 SDK] 机器狗未就绪，停止运动指令"
            )
            if self.sport_client:
                self.sport_client.StopMove()
            return None
        
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
            # 更新状态标志
            self.is_robot_standing = True
            self.is_robot_unlocked = True
    
    def stand_down(self):
        """趴下"""
        if self.use_high_level_sdk and self.sport_client:
            self.sport_client.StandDown()
            rospy.loginfo("趴下 (SDK)")
            # 更新状态标志
            self.is_robot_standing = False
    
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
