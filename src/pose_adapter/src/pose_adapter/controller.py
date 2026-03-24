#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运动控制器 - 生成控制指令实现机器狗位姿调整 (ROS2 版本)

支持三种模式：
1. cmd_vel 模式：通过 ROS cmd_vel 控制（原有方式）
2. high_level 模式：通过 Unitree SDK high_level 接口控制（Go2）
3. ZSI-1 模式：通过 zsibot_sdk 控制（zsl-1 机器狗）
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time
import threading
import threading


# SDK 单例（与 dog_device 一致）
_zsi_app_instance = None
_zsi_app_lock = threading.Lock()

class MotionState:
    """运动状态机"""
    IDLE = "idle"
    MOVING = "moving"
    ROTATING = "rotating"
    WAITING = "waiting"


class MotionController(Node):
    """
    运动控制器 (ROS2)
    
    根据位姿误差生成控制指令，实现闭环控制
    支持 cmd_vel 和 high_level SDK 两种模式
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
                 use_high_level_sdk=False,
                 interface_name="eth0",
                 disable_obstacle_avoidance_on_start=True,
                 use_classic_walk=False,
                 speed_level=0,
                 body_type=None):
        """初始化控制器"""
        super().__init__('motion_controller')
        
        # 自动检测 body_type
        if body_type is None:
            body_type = os.environ.get('BODY', '').strip()
        
        self.body_type = body_type.upper() if body_type else None
        self.get_logger().info(f"[运动控制器] body_type: {self.body_type}")
        
        # ZSI-1 模式优先级高于 use_high_level_sdk
        self.use_zsi1_sdk = self.body_type == 'ZSI-1'
        
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
        self.use_high_level_sdk = use_high_level_sdk
        self.interface_name = interface_name
        self.disable_obstacle_avoidance_on_start = disable_obstacle_avoidance_on_start
        self.use_classic_walk = use_classic_walk
        self.speed_level = speed_level

        # PID 参数
        self.kp_linear = 0.35
        self.ki_linear = 0.0
        self.kd_linear = 0.05
        self.kp_angular = 0.5
        self.ki_angular = 0.0
        self.kd_angular = 0.05

        self.error_int_linear = 0
        self.error_int_angular = 0
        self.error_prev_linear = 0
        self.error_prev_angular = 0

        # 状态
        self.is_positioned = False
        self.positioned_count = 0
        self.positioned_threshold = 10
        self._last_distance_error = 0.0

        # 运动状态机
        self.motion_state = MotionState.IDLE
        self.motion_start_time = 0.0
        self.estimated_duration = 0.0
        self.wait_timeout = 6.0
        self.motion_type = None

        self.is_running = True

        self.current_euler = [0.0, 0.0, 0.0]
        self.target_euler = [0.0, 0.0, 0.0]

        # ROS 发布器
        self.cmd_vel_pub = None

        # SDK 客户端
        self.sport_client = None
        self.sdk_initialized = False

        # 机器人状态
        self.robot_state = None
        self.is_robot_standing = False
        self.is_robot_unlocked = False
        self._unknown_state_init_done = False
        self._waiting_vx = 0.0
        self._waiting_vy = 0.0
        self._waiting_vyaw = 0.0
        self._last_gentle_gait_time = 0.0

        # 初始化
        if self.use_zsi1_sdk:
            self._init_zsi1_sdk()
        elif self.use_high_level_sdk:
            self._init_high_level_sdk()
        else:
            self._init_ros()

    def _init_ros(self):
        """初始化 ROS2 cmd_vel"""
        try:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.get_logger().info("运动控制器已初始化（cmd_vel 模式）")
        except Exception as e:
            self.get_logger().error(f"ROS 初始化失败: {e}")

    def _init_zsi1_sdk(self):
        """初始化 zsibot_sdk (ZSI-1) 接口"""
        try:
            import sys
            import platform
            
            arch = platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64')
            env_zsi_root = os.environ.get('ZSI_SDK_ROOT')
            if env_zsi_root:
                sdk_root = os.path.abspath(os.path.expanduser(env_zsi_root))
            else:
                candidates = [
                    # 常见用法：在 PoseAdapter 目录执行 ros2 launch
                    os.path.abspath(os.path.join(os.getcwd(), '..', 'zsibot_sdk')),
                    # 从源码目录运行时的相对推导
                    os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'zsibot_sdk')),
                    # 从 install 目录运行时的相对推导
                    os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..', '..', '..', 'zsibot_sdk')),
                ]
                sdk_root = next((p for p in candidates if os.path.isdir(p)), candidates[0])

            sdk_path = os.path.join(sdk_root, f'lib/zsl-1/{arch}')
            self.get_logger().info(f"[ZSI-1 SDK] ZSI_SDK_ROOT={sdk_root}")
            self.get_logger().info(f"[ZSI-1 SDK] Python 模块目录={sdk_path}")

            if not os.path.isdir(sdk_path):
                raise RuntimeError(
                    f"[ZSI-1 SDK] 未找到目录: {sdk_path}。"
                    "请确认 ZSI_SDK_ROOT=../zsibot_sdk，且已按官方文档编译生成 lib/zsl-1/<arch>。"
                    "参考: https://zsibot.github.io/zsibot_sdk"
                )
            
            if sdk_path not in sys.path:
                sys.path.insert(0, sdk_path)
            
            import mc_sdk_zsl_1_py
            
            local_ip = os.environ.get('ZSI_LOCAL_IP', '192.168.234.15')
            local_port = int(os.environ.get('ZSI_LOCAL_PORT', 43988))
            dog_ip = os.environ.get('ZSI_DOG_IP', '192.168.234.1')
            
            # 使用单例模式（与 dog_device 一致）
            global _zsi_app_instance
            if _zsi_app_instance is None:
                with _zsi_app_lock:
                    if _zsi_app_instance is None:
                        _zsi_app_instance = mc_sdk_zsl_1_py.HighLevel()
                        _zsi_app_instance.initRobot(local_ip, local_port, dog_ip)
            self.zsi_client = _zsi_app_instance
            
            self.sdk_initialized = True
            self.get_logger().info(f"[ZSI-1 SDK] 已初始化，连接 {local_ip}:{local_port} -> {dog_ip}")
            
            # 速度限制参数（与 dog_device 一致）
            self._zsi_deadzone_vx = 0.05
            self._zsi_deadzone_vy = 0.05
            self._zsi_deadzone_wz = 0.02
            self._zsi_max_vx = 2.0
            self._zsi_max_vy = 2.0
            self._zsi_max_wz = 3.0
            self._zsi_move_lock = threading.Lock()
            
            self.get_logger().info("[ZSI-1 SDK] SDK 初始化完成")
            
        except ImportError as e:
            self.get_logger().error(f"[ZSI-1 SDK] 导入失败: {e}")
            raise RuntimeError(
                "[ZSI-1 SDK] 必须安装并可导入 mc_sdk_zsl_1_py；当前已按要求禁止回退到 cmd_vel。"
                "请确认 ZSI_SDK_ROOT=../zsibot_sdk，且该目录下存在 lib/zsl-1/<arch>/mc_sdk_zsl_1_py*。"
                "参考: https://zsibot.github.io/zsibot_sdk"
            ) from e
        except Exception as e:
            self.get_logger().error(f"[ZSI-1 SDK] 初始化失败: {e}")
            raise RuntimeError("[ZSI-1 SDK] 初始化失败；当前已按要求禁止回退到 cmd_vel。") from e

    def _zsi1_preflight_check(self):
        """ZSI-1 启动预检：最小动作探测，提前暴露控制权/自检异常。"""
        self.get_logger().info("[ZSI-1 SDK] 预检开始：standUp + move(0,0,0) 探测")
        last_err = None
        for i in range(3):
            try:
                self.zsi_client.standUp()
                self._sleep(0.5)
                self.zsi_client.move(0.0, 0.0, 0.0)
                self._sleep(0.2)
                self.get_logger().info(f"[ZSI-1 SDK] 预检通过 (attempt={i + 1})")
                return
            except Exception as e:
                last_err = e
                self.get_logger().warn(f"[ZSI-1 SDK] 预检失败 (attempt={i + 1}): {e}")
                self._sleep(0.5)

        raise RuntimeError(
            "[ZSI-1 SDK] 预检失败：机器人可能处于 passive/自检失败/控制权冲突状态。"
            "请检查遥控器与APP是否占用控制权、机器人是否已完成自检并处于可运动状态。"
        ) from last_err

    def _init_high_level_sdk(self):
        """初始化 Unitree SDK high_level 接口"""
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

            self.get_logger().info("[Go2 SDK] SDK 初始化完成")

        except ImportError as e:
            self.get_logger().error(f"[Go2 SDK] 导入失败: {e}")
            self.get_logger().warn("回退到 cmd_vel 模式")
            self.use_high_level_sdk = False
            self._init_ros()
        except Exception as e:
            self.get_logger().error(f"[Go2 SDK] 初始化失败: {e}")
            self.get_logger().warn("回退到 cmd_vel 模式")
            self.use_high_level_sdk = False
            self._init_ros()

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
        manual_hint = "请手动关闭避障"
        try:
            from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
            client = ObstaclesAvoidClient()
            client.Init()
            client.SwitchSet(False)
            self.get_logger().info("[Go2 SDK] 已关闭避障")
        except ImportError:
            self.get_logger().info(f"[Go2 SDK] {manual_hint}")
        except Exception as e:
            self.get_logger().warn(f"[Go2 SDK] 关闭避障失败: {e}")

    def _set_gentle_gait(self, quiet=False, throttle_interval=1.5):
        """设置柔和步态"""
        if not self.sport_client:
            return
        now = time.time()
        if quiet and (now - self._last_gentle_gait_time) < throttle_interval:
            return
        self._last_gentle_gait_time = now

        self.get_logger().info(f"[Go2 SDK] 设置柔和步态: ClassicWalk({self.use_classic_walk}) + SpeedLevel({self.speed_level})")

        try:
            if self.use_classic_walk and hasattr(self.sport_client, 'ClassicWalk'):
                self.sport_client.ClassicWalk(True)
        except Exception as e:
            self.get_logger().warn(f"[Go2 SDK] 开启经典步态失败: {e}")

        try:
            if hasattr(self.sport_client, 'SpeedLevel'):
                self.sport_client.SpeedLevel(self.speed_level)
        except Exception as e:
            self.get_logger().warn(f"[Go2 SDK] 设置速度档位失败: {e}")

    def _get_pose_subscriber_state(self):
        """获取机器人状态"""
        if not hasattr(self, 'pose_sub') or self.pose_sub is None:
            return None
        has_data = getattr(self.pose_sub, 'HasReceived', None) or getattr(self.pose_sub, 'has_received', None)
        if callable(has_data) and not has_data():
            return None
        getter = (
            getattr(self.pose_sub, 'Get', None) or getattr(self.pose_sub, 'get', None)
            or getattr(self.pose_sub, 'receive', None) or getattr(self.pose_sub, 'read', None)
        )
        if not callable(getter):
            return None
        try:
            return getter()
        except Exception:
            return None

    def _update_robot_state(self):
        """更新机器人状态"""
        state = self._get_pose_subscriber_state()
        if state:
            self.robot_state = state
            self.current_euler = [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]]
            self._check_standing_state(state)

    def _check_standing_state(self, state):
        """检查机器人站立状态"""
        try:
            mode = getattr(state, 'mode', None)
            if mode is None:
                mode = getattr(state, 'mode_request', None)
            if mode is None:
                mode = getattr(state, 'robot_mode', None)

            if mode is None:
                self.is_robot_unlocked = True
                self.is_robot_standing = True
                return

            gait_status = getattr(state, 'gait_status', 0)

            self.is_robot_unlocked = mode == 9
            self.is_robot_standing = (mode == 9 and gait_status > 0)

        except Exception as e:
            self.get_logger().warn(f"检查站立状态失败: {e}")
            self.is_robot_standing = True
            self.is_robot_unlocked = True

    def ensure_robot_ready(self):
        """确保机器狗已站立并解锁"""
        if not self.use_high_level_sdk or not self.sport_client or not self.sdk_initialized:
            return True

        self._update_robot_state()

        if self.robot_state is None:
            if not self._unknown_state_init_done:
                self.get_logger().warn("[Go2 SDK] 无法获取机器人状态，先执行 BalanceStand")
                try:
                    self.sport_client.BalanceStand()
                    self._sleep(2.5)
                    self.sport_client.Move(0.0, 0.0, 0.0)
                    self._sleep(1.0)
                    self._set_gentle_gait(quiet=False)
                    self._unknown_state_init_done = True
                except Exception as e:
                    self.get_logger().warn(f"初始化失败: {e}")
                    self._unknown_state_init_done = True
            return True

        if not self.is_robot_unlocked:
            self.get_logger().warn("[Go2 SDK] 机器人未进入运动模式")
            try:
                self.sport_client.BalanceStand()
                self._sleep(2.0)
                self.sport_client.Move(0.0, 0.0, 0.0)
                self._sleep(1.0)
                self._set_gentle_gait(quiet=False)
                self._update_robot_state()
            except Exception as e:
                self.get_logger().warn(f"切换运动模式失败: {e}")

        return True

    def _update_current_pose(self):
        """更新当前姿态"""
        state = self._get_pose_subscriber_state()
        if state:
            self.current_euler = [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]]
            self._check_standing_state(state)

    def _sleep(self, seconds):
        """ROS2 兼容的 sleep"""
        time.sleep(seconds)

    def compute_control(self, pose, bbox_ratio, center_offset):
        """计算控制指令"""
        if not pose.get('success', False):
            self.get_logger().warn("位姿解算失败，停止运动")
            if self.use_high_level_sdk and self.sport_client:
                self.sport_client.StopMove()
            self._reset_motion_state()
            return Twist()

        distance = pose['distance']
        yaw = pose['yaw']
        offset_x, offset_y = center_offset

        if self.use_high_level_sdk:
            return self._compute_high_level_control(distance, yaw, offset_x)
        else:
            return self._compute_cmd_vel_control(distance, yaw, offset_x)

    def _reset_motion_state(self):
        """重置运动状态"""
        self.motion_state = MotionState.IDLE
        self.motion_type = None
        self._waiting_vx = 0.0
        self._waiting_vy = 0.0
        self._waiting_vyaw = 0.0

    def _check_wait_timeout(self):
        """检查等待超时"""
        current_time = time.time()
        elapsed = current_time - self.motion_start_time

        if elapsed > self.wait_timeout:
            self.get_logger().warn(f"[运动控制] 等待超时 ({elapsed:.1f}s)")
            self._reset_motion_state()
            return True
        return False

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
        duration = np.clip(duration, 0.5, 3.0)
        return duration

    def _compute_high_level_control(self, distance, yaw, offset_x):
        """使用 SDK 计算控制指令"""
        if not self.ensure_robot_ready():
            return None
        
        self._update_current_pose()
        
        distance_error = distance - self.target_distance
        angle_error = yaw + offset_x * 30
        
        distance_ok = abs(distance_error) <= self.distance_tolerance
        angle_ok = abs(angle_error) <= self.angle_tolerance
        center_ok = abs(offset_x) <= self.center_tolerance
        
        if distance_ok and angle_ok and center_ok:
            self.get_logger().info("[精确控制] 目标已到位")
            self.is_positioned = True
            if self.sport_client:
                self.sport_client.StopMove()
            self._reset_motion_state()
            return None
        
        if not angle_ok:
            turn_deg = angle_error
            if abs(turn_deg) > self.step_angle:
                turn_deg = self.step_angle if turn_deg > 0 else -self.step_angle
            
            self.get_logger().info(f"[精确控制] 旋转 {turn_deg:.1f}度")
            self.turn_angle(turn_deg, timeout=3.0)
            self.is_positioned = False
            return None
        
        if distance_error > 0:
            move_dist = distance_error
            if move_dist > self.step_distance:
                move_dist = self.step_distance
            self.get_logger().info(f"[精确控制] 前进 {move_dist*100:.1f}cm")
            self.move_forward_distance(move_dist, timeout=5.0)
            self.is_positioned = False
            return None
        else:
            move_dist = distance_error
            if abs(move_dist) > self.step_distance:
                move_dist = -self.step_distance
            self.get_logger().info(f"[精确控制] 后退 {abs(move_dist)*100:.1f}cm")
            self.move_forward_distance(move_dist, timeout=5.0)
            self.is_positioned = False
            return None
        
        return None

    def _compute_cmd_vel_control(self, distance, yaw, offset_x):
        """使用 cmd_vel 计算控制指令"""
        distance_error = distance - self.target_distance
        angle_error = yaw + offset_x * 30

        distance_ok = abs(distance_error) <= self.distance_tolerance
        angle_ok = abs(angle_error) <= self.angle_tolerance
        center_ok = abs(offset_x) <= self.center_tolerance

        self._check_positioned(distance_error, angle_error, offset_x)

        if self.motion_state == MotionState.WAITING:
            if self._check_wait_timeout():
                self.get_logger().warn("[cmd_vel] 等待超时，重置状态")
            else:
                elapsed = time.time() - self.motion_start_time
                self.get_logger().info_throttle(
                    0.5,
                    f"[cmd_vel] 等待运动完成... ({elapsed:.1f}/{self.estimated_duration:.1f}s)"
                )
                return None

        if distance_ok and angle_ok and center_ok:
            if self.motion_state != MotionState.IDLE:
                self.get_logger().info("[cmd_vel] 目标已到位，停止运动")
                self._reset_motion_state()
            return None

        cmd = Twist()
        self.estimated_duration = self._estimate_motion_duration(distance_error, angle_error)

        if not distance_ok:
            linear_vel = self.kp_linear * distance_error
            linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
            cmd.linear.x = linear_vel
            self.motion_type = "move_forward" if linear_vel > 0 else "move_backward"
            self.get_logger().info(
                f"[cmd_vel] 距离控制: error={distance_error:.3f}m, vx={linear_vel:.3f}m/s"
            )
            self.motion_state = MotionState.WAITING
            self.motion_start_time = time.time()

        elif not angle_ok:
            angular_vel = -self.kp_angular * np.radians(angle_error)
            angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
            cmd.angular.z = angular_vel
            self.motion_type = "rotate_right" if angular_vel > 0 else "rotate_left"
            self.get_logger().info(
                f"[cmd_vel] 角度控制: error={angle_error:.2f}deg, wz={angular_vel:.3f}rad/s"
            )
            self.motion_state = MotionState.WAITING
            self.motion_start_time = time.time()

        return cmd

    def _check_positioned(self, distance_error, angle_error, offset_x):
        """检查是否到位"""
        self._last_distance_error = distance_error
        
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
            pass
        elif self.cmd_vel_pub:
            self.cmd_vel_pub.publish(cmd)

    def stop(self):
        """停止运动"""
        self.is_running = False
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            with self._zsi_move_lock:
                self.zsi_client.move(0, 0, 0)
            self.get_logger().info("停止运动 (ZSI-1)")
        elif self.use_high_level_sdk and self.sport_client:
            self.sport_client.StopMove()
            self.get_logger().info("停止运动 (SDK)")
        else:
            cmd = Twist()
            self.publish_control(cmd)
            self.get_logger().info("停止运动 (cmd_vel)")

    def stand_up(self):
        """站立"""
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            self.zsi_client.standUp()
            self.get_logger().info("站立 (ZSI-1)")
            self.is_robot_standing = True
            self.is_robot_unlocked = True
        elif self.use_high_level_sdk and self.sport_client:
            self.sport_client.StandUp()
            self.get_logger().info("站立 (SDK)")
            self.is_robot_standing = True
            self.is_robot_unlocked = True

    def stand_down(self):
        """趴下"""
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            self.zsi_client.lieDown()
            self.get_logger().info("趴下 (ZSI-1)")
            self.is_robot_standing = False
        elif self.use_high_level_sdk and self.sport_client:
            self.sport_client.StandDown()
            self.get_logger().info("趴下 (SDK)")
            self.is_robot_standing = False

    def balance_stand(self):
        """平衡站立"""
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            self.zsi_client.standUp()
            self.get_logger().info("平衡站立 (ZSI-1)")
        elif self.use_high_level_sdk and self.sport_client:
            self.sport_client.BalanceStand()
            self.get_logger().info("平衡站立 (SDK)")

    def move_forward_distance(self, distance_m, timeout=10.0):
        """前进指定距离"""
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            return self._zsi1_move_forward(distance_m, timeout)
        
        if not self.use_high_level_sdk or not self.sport_client:
            return True
        
        direction = "前进" if distance_m > 0 else "后退"
        self.get_logger().info(f"[精确控制] {direction} {abs(distance_m)*100:.1f}cm")
        
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

    def _zsi_clamp_speed(self, vx, vy, wz):
        """速度限制（与 dog_device 一致）"""
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
    
    def _zsi1_move_forward(self, distance_m, timeout=10.0):
        """ZSI-1 前进指定距离"""
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
        vx, vy, wz = self._zsi_clamp_speed(vx, 0.0, 0.0)
        
        with self._zsi_move_lock:
            self.zsi_client.move(vx, vy, wz)
            self._sleep(duration)
            self.zsi_client.move(0, 0, 0)
        
        return True

    def turn_angle(self, angle_deg, timeout=10.0):
        """旋转指定角度"""
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            return self._zsi1_turn_angle(angle_deg, timeout)
        
        if not self.use_high_level_sdk or not self.sport_client:
            return True
        
        direction = "左转" if angle_deg > 0 else "右转"
        self.get_logger().info(f"[精确控制] {direction} {abs(angle_deg):.1f}度")
        
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

    def _zsi1_turn_angle(self, angle_deg, timeout=10.0):
        """ZSI-1 旋转指定角度"""
        direction = "左转" if angle_deg > 0 else "右转"
        self.get_logger().info(f"[ZSI-1] {direction} {abs(angle_deg):.1f}度")
        
        if not self.is_robot_standing:
            self.zsi_client.standUp()
            self._sleep(2)
            self.is_robot_standing = True
        
        angular_speed = min(self.max_angular_speed, 0.2)
        duration = abs(np.radians(angle_deg)) / angular_speed
        duration = min(duration, timeout)
        
        wz = angular_speed if angle_deg > 0 else -angular_speed
        vx, vy, wz = self._zsi_clamp_speed(0.0, 0.0, wz)
        
        with self._zsi_move_lock:
            self.zsi_client.move(vx, vy, wz)
            self._sleep(duration)
            self.zsi_client.move(0, 0, 0)
        
        return True

    def wait_for_motion_complete(self, timeout=None):
        """等待运动执行完成"""
        if timeout is None:
            timeout = self.wait_timeout

        if self.motion_state != MotionState.WAITING:
            return True

        self.get_logger().info(f"[Motion] 开始等待运动完成，超时: {timeout}s")
        start_time = time.time()
        last_diag_time = 0

        while self.motion_state == MotionState.WAITING and self.is_running:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().warn(f"[Motion] 等待超时 ({elapsed:.1f}s)")
                self._reset_motion_state()
                return False

            if elapsed - last_diag_time > 2.0:
                last_diag_time = elapsed
                self._update_robot_state()

            if self.is_positioned:
                self.get_logger().info(f"[Motion] 运动完成，耗时: {elapsed:.1f}s")
                self._reset_motion_state()
                return True

            time.sleep(0.1)

        return True

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
