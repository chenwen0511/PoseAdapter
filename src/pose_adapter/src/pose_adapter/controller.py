#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
运动控制器 - 生成控制指令实现机器狗位姿调整

支持三种模式：
1. cmd_vel 模式：通过 ROS cmd_vel 控制（原有方式）
2. high_level 模式：通过 Unitree SDK high_level 接口控制（Go2）
3. ZSI-1 模式：通过 zsibot_sdk 控制（zsl-1 机器狗）
"""

import os
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time


class MotionState:
    """运动状态机"""
    IDLE = "idle"           # 空闲，等待控制
    MOVING = "moving"       # 正在前进/后退
    ROTATING = "rotating"   # 正在旋转
    WAITING = "waiting"     # 等待运动执行完成


class MotionController:
    """
    运动控制器

    根据位姿误差生成控制指令，实现闭环控制
    支持 cmd_vel 和 high_level SDK 两种模式
    """

    def __init__(self,
                 target_distance=0.3,      # 目标距离（米）
                 target_ratio=0.5,         # 目标画面占比
                 distance_tolerance=0.05,   # 距离容差（米）
                 angle_tolerance=2.0,       # 角度容差（度）
                 center_tolerance=0.05,     # 中心偏差容差
                 max_linear_speed=0.12,     # 最大线速度（m/s）
                 min_linear_speed=0.12,     # 最小线速度（m/s）；必须>0.1才能避免固件死区
                 max_angular_speed=0.25,    # 最大角速度（rad/s）
                 step_distance=0.2,         # 每次移动步长（米），默认20cm
                 step_angle=5.0,            # 每次旋转步长（度），默认5度
                 use_high_level_sdk=False,  # 是否使用 high_level SDK
                 interface_name="eth0",    # 网络接口名称
                 disable_obstacle_avoidance_on_start=True,  # 启动时是否尝试关闭避障
                 use_classic_walk=False,    # 默认步态便于走步
                 speed_level=0,            # 普通速度
                 body_type=None):          # 机器狗类型: None (自动检测), "GO2", "ZSI-1"
        """
        初始化控制器

        Args:
            target_distance: 目标距离（米）
            target_ratio: 目标画面占比（0-1）
            distance_tolerance: 距离容差
            angle_tolerance: 角度容差（度）
            center_tolerance: 中心偏差容差
            max_linear_speed: 最大线速度
            min_linear_speed: 最小线速度（0=不限制；若狗不动可试 0.12~0.15 排除死区）
            max_angular_speed: 最大角速度
            use_high_level_sdk: 是否使用 Unitree SDK high_level 接口
            interface_name: 网络接口名称（如 eth0, enp2s0 等）
            disable_obstacle_avoidance_on_start: 启动时是否尝试关闭避障（整个 adapter 运动控制需在关闭避障+经典步态下运行）
            body_type: 机器狗类型 ("GO2", "ZSI-1")，默认从环境变量 BODY 读取
        """
        # 自动检测 body_type
        if body_type is None:
            body_type = os.environ.get('BODY', '').strip()
        
        self.body_type = body_type.upper() if body_type else None
        rospy.loginfo(f"[运动控制器] body_type: {self.body_type}")
        
        # ZSI-1 模式优先级高于 use_high_level_sdk
        self.use_zsi1_sdk = self.body_type == 'ZSI-1'
        
        self.target_distance = target_distance
        self.target_ratio = target_ratio
        self.distance_tolerance = distance_tolerance
        self.angle_tolerance = angle_tolerance
        self.center_tolerance = center_tolerance
        # 默认降低最大速度，使步态更柔和（可经 launch 覆盖）
        self.max_linear_speed = max_linear_speed
        self.min_linear_speed = min_linear_speed
        self.max_angular_speed = max_angular_speed
        # 步长参数（每次移动/旋转的步长）
        self.step_distance = step_distance  # 米，默认0.2m=20cm
        self.step_angle = step_angle        # 度，默认5度
        self.use_high_level_sdk = use_high_level_sdk
        self.interface_name = interface_name
        self.disable_obstacle_avoidance_on_start = disable_obstacle_avoidance_on_start
        self.use_classic_walk = use_classic_walk
        self.speed_level = speed_level

        # PID 参数（偏柔和，避免步态激进僵硬）
        self.kp_linear = 0.35
        self.ki_linear = 0.0
        self.kd_linear = 0.05

        self.kp_angular = 0.5
        self.ki_angular = 0.0
        self.kd_angular = 0.05

        # 误差积分和微分
        self.error_int_linear = 0
        self.error_int_angular = 0
        self.error_prev_linear = 0
        self.error_prev_angular = 0

        # 状态
        self.is_positioned = False
        self.positioned_count = 0
        self.positioned_threshold = 10  # 连续多少帧达标才算到位
        self._last_distance_error = 0.0  # 用于诊断

        # 运动状态机（解决控制频率问题）
        self.motion_state = MotionState.IDLE
        self.motion_start_time = 0.0      # 运动开始时间
        self.estimated_duration = 0.0      # 预估运动持续时间(秒)
        self.wait_timeout = 6.0            # 等待超时时间(秒)，需大于预估时长否则会频繁超时导致停→抖→再走
        self.motion_type = None            # 当前运动类型: "move_forward", "move_backward", "rotate_left", "rotate_right"

        # 运行状态（用于 wait_for_motion_complete）
        self.is_running = True

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
        self._unknown_state_init_done = False  # 无法获取状态时是否已做过一次 BalanceStand 初始化
        self._waiting_vx = 0.0  # 等待期间持续发送的速度（保持走步）
        self._waiting_vy = 0.0
        self._waiting_vyaw = 0.0
        self._last_gentle_gait_time = 0.0  # 上次施加柔和步态的时间（用于节流）

        # 初始化
        if self.use_zsi1_sdk:
            self._init_zsi1_sdk()
        elif self.use_high_level_sdk:
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

    def _init_zsi1_sdk(self):
        """
        初始化 zsibot_sdk (ZSI-1) 接口
        
        ZSI-1 机器狗控制接口:
        - move(vx, vy, yaw_rate): 速度控制
        - attitudeControl(roll_vel, pitch_vel, yaw_vel, height_vel): 姿态控制
        """
        try:
            import sys
            import platform
            
            # 设置库路径
            arch = platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64')
            _default_zsi_root = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'zsibot_sdk')
            )
            sdk_path = os.path.join(os.environ.get('ZSI_SDK_ROOT', _default_zsi_root), f'lib/zsl-1/{arch}')
            
            if sdk_path not in sys.path:
                sys.path.insert(0, sdk_path)
            
            import mc_sdk_zsl_1_py
            
            # 创建 HighLevel 客户端
            self.zsi_client = mc_sdk_zsl_1_py.HighLevel()
            
            # 初始化连接
            local_ip = os.environ.get('ZSI_LOCAL_IP', '192.168.234.15')
            local_port = int(os.environ.get('ZSI_LOCAL_PORT', 43988))
            dog_ip = os.environ.get('ZSI_DOG_IP', '192.168.234.1')
            
            self.zsi_client.initRobot(local_ip, local_port, dog_ip)
            
            self.sdk_initialized = True
            rospy.loginfo(f"[ZSI-1 SDK] ✓ 已初始化，连接 {local_ip}:{local_port} -> {dog_ip}")
            
            # 站立
            rospy.loginfo("[ZSI-1 SDK] 站立...")
            self.zsi_client.standUp()
            rospy.sleep(2)
            
            rospy.loginfo("[ZSI-1 SDK] ===== SDK 初始化完成 =====")
            
        except ImportError as e:
            rospy.logerr(f"[ZSI-1 SDK] ✗ 导入失败: {e}")
            rospy.logwarn("回退到 cmd_vel 模式")
            self.use_zsi1_sdk = False
            self._init_ros()
        except Exception as e:
            rospy.logerr(f"[ZSI-1 SDK] ✗ 初始化失败: {e}")
            rospy.logwarn("回退到 cmd_vel 模式")
            self.use_zsi1_sdk = False
            self._init_ros()

    def _init_high_level_sdk(self):
        """
        初始化 Unitree SDK high_level 接口

        业界最佳实践初始化流程:
        1. 初始化 DDS 通道
        2. 创建 SportClient
        3. 订阅机器人状态 (rt/SportModeState)
        4. 关闭避障
        5. 设置柔和步态
        """
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
            from unitree_sdk2py.core.channel import ChannelSubscriber

            # Step 1: 初始化 DDS 通道（与手动运行 go2_sport_client.py eth1 一致）
            rospy.loginfo("[Go2 SDK] Step 1: 初始化 DDS 通道 (ChannelFactoryInitialize)")
            ChannelFactoryInitialize(0, self.interface_name)
            rospy.loginfo(f"[Go2 SDK] ✓ DDS 通道已初始化（接口: {self.interface_name}）")

            # Step 2: 创建 SportClient
            rospy.loginfo("[Go2 SDK] Step 2: 创建 SportClient")
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()

            self.sdk_initialized = True
            rospy.loginfo("[Go2 SDK] ✓ SportClient 已初始化（high_level 模式）")

            # Step 3: 订阅机器人状态
            rospy.loginfo("[Go2 SDK] Step 3: 订阅机器人状态 (rt/SportModeState)")
            self._subscribe_pose()

            # Step 4: 关闭避障（运动控制需在关闭避障下运行）
            rospy.loginfo("[Go2 SDK] Step 4: 关闭避障")
            if self.disable_obstacle_avoidance_on_start:
                self._try_disable_obstacle_avoidance()

            # Step 5: 设置柔和步态
            rospy.loginfo("[Go2 SDK] Step 5: 设置柔和步态")
            self._set_gentle_gait()

            rospy.loginfo("[Go2 SDK] ===== SDK 初始化完成 =====")

        except ImportError as e:
            rospy.logerr(f"[Go2 SDK] ✗ Unitree SDK 导入失败: {e}")
            rospy.logwarn("回退到 cmd_vel 模式")
            self.use_high_level_sdk = False
            self._init_ros()
        except Exception as e:
            rospy.logerr(f"[Go2 SDK] ✗ Unitree SDK 初始化失败: {e}")
            rospy.logwarn("回退到 cmd_vel 模式")
            self.use_high_level_sdk = False
            self._init_ros()

    def _subscribe_pose(self):
        """订阅机器人姿态状态。ChannelSubscriber 需要 IDL 类型（类），不能是 default 里的工厂函数。"""
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            # 使用 unitree_go.msg.dds_ 中的 SportModeState_ 类（IDL 类型），而非 idl.default 的 unitree_go_msg_dds__SportModeState_ 函数
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            self.pose_sub = ChannelSubscriber("rt/SportModeState", SportModeState_)
            self.pose_sub.Init()
            rospy.loginfo("已订阅 SportModeState 主题")
        except ImportError as e:
            try:
                from unitree_sdk2py.core.channel import ChannelSubscriber
                from unitree_sdk2py.idl.unitree_go.msg import dds_ as go_dds
                SportModeState_ = getattr(go_dds, "SportModeState_", None)
                if SportModeState_ is not None and isinstance(SportModeState_, type):
                    self.pose_sub = ChannelSubscriber("rt/SportModeState", SportModeState_)
                    self.pose_sub.Init()
                    rospy.loginfo("已订阅 SportModeState 主题 (unitree_go.msg.dds_)")
                else:
                    rospy.logwarn("姿态订阅失败: 未找到 SportModeState_ IDL 类型: %s，将无法获取机器人状态", e)
                    self.pose_sub = None
            except Exception as e2:
                rospy.logwarn("姿态订阅失败: %s，将无法获取机器人状态", e2)
                self.pose_sub = None
        except Exception as e:
            rospy.logwarn("姿态订阅失败: %s，将无法获取机器人状态", e)
            self.pose_sub = None

    def _try_disable_obstacle_avoidance(self):
        """
        尝试通过 SDK 关闭避障 - 业界最佳实践

        根据 Unitree 官方文档:
        整个 adapter 运动控制需在关闭避障、经典步态（稀碎步）下运行。

        若当前 unitree_sdk2py 未提供避障接口，仅打日志提示用户手动在 APP 或 ros2 关闭。
        """
        manual_hint = "请手动关闭避障：APP 中关闭，或在 Go2 上执行 ros2 go2 obstacles_avoidance stop"

        rospy.loginfo("[Go2 SDK] 尝试关闭避障...")

        try:
            from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
            client = ObstaclesAvoidClient()
            client.Init()
            client.SwitchSet(False)  # False = 关闭避障
            rospy.loginfo("[Go2 SDK] ✓ 已通过 ObstaclesAvoidClient.SwitchSet(False) 关闭避障")
        except ImportError:
            rospy.loginfo("[Go2 SDK] ℹ 当前 SDK 未提供避障关闭接口，%s", manual_hint)
        except Exception as e:
            rospy.logwarn("[Go2 SDK] ✗ 关闭避障失败: %s，%s", e, manual_hint)

    def _set_gentle_gait(self, quiet=False, throttle_interval=1.5):
        """
        设置柔和步态 - 业界最佳实践

        根据 Unitree 官方文档:
        - ClassicWalk(True): 经典步态（稀碎步），更柔和
        - SpeedLevel(-1): 慢速档，减轻激进、僵硬感

        发现目标后首次发送 Move 时机器人可能切回激进步态，
        故在每次开始运动前可重新调用。

        Args:
            quiet: True 时不打日志（用于运动前重复施加）
            throttle_interval: 重复调用时的最小间隔（秒），避免刷屏与频繁发指令
        """
        if not self.sport_client:
            return
        now = time.time()
        if quiet and (now - self._last_gentle_gait_time) < throttle_interval:
            return
        self._last_gentle_gait_time = now

        rospy.loginfo("[Go2 SDK] 设置柔和步态: ClassicWalk(True) + SpeedLevel(%d)", self.speed_level)

        # ===== 业界最佳实践: 经典步态 + 慢速档 =====
        try:
            if self.use_classic_walk and hasattr(self.sport_client, 'ClassicWalk'):
                self.sport_client.ClassicWalk(True)
                if not quiet:
                    rospy.loginfo("[Go2 SDK] ✓ 已开启经典步态（稀碎步），步态更柔和")
        except Exception as e:
            if not quiet:
                rospy.logwarn("[Go2 SDK] ✗ 开启经典步态失败: %s", e)

        try:
            if hasattr(self.sport_client, 'SpeedLevel'):
                self.sport_client.SpeedLevel(self.speed_level)  # -1=慢 0=普通 1=快
                if not quiet:
                    rospy.loginfo("[Go2 SDK] ✓ 已设置速度档位 SpeedLevel(%d)", self.speed_level)
        except Exception as e:
            if not quiet:
                rospy.logwarn("[Go2 SDK] ✗ 设置速度档位失败: %s", e)

    def _get_pose_subscriber_state(self):
        """从 pose_sub 取一次状态，兼容不同 SDK 的 Get/get/receive 等接口。无数据或不可用时返回 None。"""
        if not hasattr(self, 'pose_sub') or self.pose_sub is None:
            return None
        # 兼容：HasReceived / has_received（有则先判是否有数据）
        has_data = getattr(self.pose_sub, 'HasReceived', None) or getattr(self.pose_sub, 'has_received', None)
        if callable(has_data) and not has_data():
            return None
        # 兼容：Get / get / receive / read
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
            # 从状态中获取欧拉角 (roll, pitch, yaw)
            self.current_euler = [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]]
            # 检查机器人是否处于站立状态
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

        根据 Unitree 官方文档:
        1. 调用 BalanceStand() 进入平衡站立状态
        2. 调用 Move(0,0,0) 触发进入运动模式 (mode 9)

        Returns:
            bool: True 表示机器人已就绪
        """
        if not self.use_high_level_sdk or not self.sport_client or not self.sdk_initialized:
            return True  # 非 SDK 模式直接返回

        # 更新状态
        self._update_robot_state()

        # 如果无法获取状态（SportModeState 未收到或订阅未建立），需先让机器狗进入可走步状态
        # 否则 Move(vx,0,0) 可能只被解释为身体前倾而不迈步
        if self.robot_state is None:
            if not self._unknown_state_init_done:
                rospy.logwarn(
                    "[Go2 SDK] 无法获取机器人状态，先执行 BalanceStand 进入可走步状态再发送运动指令..."
                )
                try:
                    # Step 1: 进入平衡站立状态
                    rospy.loginfo("[Go2 SDK] Step 1: 调用 BalanceStand() 进入平衡站立状态")
                    self.sport_client.BalanceStand()
                    rospy.sleep(2.5)  # 等待平衡站立完成

                    # Step 2: 触发进入运动模式
                    rospy.loginfo("[Go2 SDK] Step 2: 调用 Move(0,0,0) 触发进入运动模式 (mode 9)")
                    self.sport_client.Move(0.0, 0.0, 0.0)
                    rospy.sleep(1.0)  # 进入运动模式

                    # Step 3: 设置柔和步态
                    rospy.loginfo("[Go2 SDK] Step 3: 调用 ClassicWalk(True) 设置经典步态")
                    self._set_gentle_gait(quiet=False)

                    self._unknown_state_init_done = True
                    rospy.loginfo("[Go2 SDK] 未知状态初始化完成，可接受运动指令")
                except Exception as e:
                    rospy.logwarn(f"[Go2 SDK] 初始化失败: {e}，尝试直接发送运动指令...")
                    try:
                        self.sport_client.Move(0.0, 0.0, 0.0)
                        rospy.sleep(0.5)
                    except Exception as e2:
                        rospy.logwarn(f"发送运动指令失败: {e2}")
                    self._unknown_state_init_done = True
            return True

        if not self.is_robot_unlocked:
            rospy.logwarn("[Go2 SDK] 机器人未进入运动模式 (mode 9)，尝试平衡站立...")

            # ===== Unitree 官方文档: 确保机器狗就绪 =====
            # Step 1: 平衡站立
            rospy.loginfo("[Go2 SDK] Step 1: 调用 BalanceStand() 进入平衡站立状态")
            self.balance_stand()
            rospy.sleep(2.0)  # 等待平衡站立完成

            # Step 2: 触发运动模式
            rospy.loginfo("[Go2 SDK] Step 2: 调用 Move(0,0,0) 触发进入运动模式 (mode 9)")
            try:
                self.sport_client.Move(0.0, 0.0, 0.0)
                rospy.sleep(1.0)  # 等待模式切换
            except Exception as e:
                rospy.logwarn(f"切换运动模式失败: {e}")

            # Step 3: 设置柔和步态
            rospy.loginfo("[Go2 SDK] Step 3: 调用 ClassicWalk(True) 设置经典步态")
            self._set_gentle_gait(quiet=False)

            self._update_robot_state()

        if self.is_robot_standing and self.is_robot_unlocked:
            rospy.loginfo("[Go2 SDK] 机器人已就绪 (mode 9)，可以接受运动指令")
            return True
        else:
            # 再次尝试
            current_mode = getattr(self.robot_state, 'mode', 'unknown')
            rospy.logwarn(f"[Go2 SDK] 当前 mode={current_mode}, 再次尝试进入运动模式...")

            # ===== 重试: 确保机器狗就绪 =====
            rospy.loginfo("[Go2 SDK] Retry Step 1: BalanceStand()")
            try:
                self.sport_client.BalanceStand()
                rospy.sleep(1.5)
            except:
                pass

            rospy.loginfo("[Go2 SDK] Retry Step 2: Move(0,0,0)")
            try:
                self.sport_client.Move(0.0, 0.0, 0.0)
                rospy.sleep(1.0)
                self._update_robot_state()
            except:
                pass

            # 重试设置步态
            self._set_gentle_gait(quiet=False)

            if self.is_robot_standing and self.is_robot_unlocked:
                rospy.loginfo("[Go2 SDK] 机器人已就绪 (mode 9)")
                return True
            else:
                # 多次尝试失败后，放行运动指令（让用户手动确认状态）
                rospy.logerr("[Go2 SDK] 机器人未能进入运动模式，放行运动指令...")
                return True

    def _update_current_pose(self):
        """更新当前姿态"""
        state = self._get_pose_subscriber_state()
        if state:
            # 从状态中获取欧拉角 (roll, pitch, yaw)
            self.current_euler = [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]]
            # 同时检查站立状态
            self._check_standing_state(state)

    def compute_control(self, pose, bbox_ratio, center_offset):
        """
        计算控制指令

        使用预估时间方案：
        1. 如果正在等待运动完成，检查是否超时
        2. 如果超时或空闲，计算控制量并预估执行时间
        3. 下发命令后进入等待状态

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
        """检查等待是否超时"""
        current_time = time.time()
        elapsed = current_time - self.motion_start_time

        if elapsed > self.wait_timeout:
            rospy.logwarn(f"[运动控制] 等待超时 ({elapsed:.1f}s > {self.wait_timeout}s)，重置状态")
            self._reset_motion_state()
            return True
        return False

    def _estimate_motion_duration(self, distance_error, angle_error):
        """
        预估运动持续时间

        Args:
            distance_error: 距离误差（米）
            angle_error: 角度误差（度）

        Returns:
            预估时间（秒）
        """
        # 距离预估时间
        if abs(distance_error) > self.distance_tolerance:
            # 假设平均速度是最大速度的 60%（考虑加减速）
            avg_linear_speed = self.max_linear_speed * 0.6
            distance_duration = abs(distance_error) / avg_linear_speed
        else:
            distance_duration = 0

        # 角度预估时间
        if abs(angle_error) > self.angle_tolerance:
            # 假设平均角速度是最大角速度的 60%
            avg_angular_speed = self.max_angular_speed * 0.6
            angle_duration = abs(np.radians(angle_error)) / avg_angular_speed
        else:
            angle_duration = 0

        # 取较大值 + 0.5秒余量
        duration = max(distance_duration, angle_duration) + 0.5

        # 限制在合理范围
        duration = np.clip(duration, 0.5, 3.0)

        return duration

    def _compute_high_level_control(self, distance, yaw, offset_x):
        """
        使用精确控制方法计算控制指令
        
        使用 move_forward_distance 和 turn_angle 实现精确控制：
        - 每次只移动一小步（20cm）
        - 每次只转一小角度（5度）
        - 执行完成后返回，不持续发送命令
        """
        # 确保机器人就绪
        if not self.ensure_robot_ready():
            rospy.logwarn_throttle(2.0, "[精确控制] 机器狗未就绪")
            return None
        
        # 更新当前姿态
        self._update_current_pose()
        
        # 计算误差
        distance_error = distance - self.target_distance
        angle_error = yaw + offset_x * 30  # 中心偏差贡献角度误差
        
        # 判断是否到位
        distance_ok = abs(distance_error) <= self.distance_tolerance
        angle_ok = abs(angle_error) <= self.angle_tolerance
        center_ok = abs(offset_x) <= self.center_tolerance
        
        # 如果已到位，停止运动
        if distance_ok and angle_ok and center_ok:
            rospy.loginfo("[精确控制] 目标已到位，停止运动")
            self.is_positioned = True
            if self.sport_client:
                self.sport_client.StopMove()
            self._reset_motion_state()
            return None
        
        # 优先处理角度偏差
        if not angle_ok:
            # 根据实际误差计算旋转角度（限制在最大步长内）
            turn_deg = angle_error
            if abs(turn_deg) > self.step_angle:
                turn_deg = self.step_angle if turn_deg > 0 else -self.step_angle
            
            rospy.loginfo(f"[精确控制] 旋转 {turn_deg:.1f}度 (误差{angle_error:.1f}度)")
            self.turn_angle(turn_deg, timeout=3.0)
            self.is_positioned = False
            return None
        
        # 处理距离偏差
        if distance_error > 0:
            # 距离太远，需要前进
            move_dist = distance_error
            # 限制最大步长
            if move_dist > self.step_distance:
                move_dist = self.step_distance
            rospy.loginfo(f"[精确控制] 前进 {move_dist*100:.1f}cm (误差{distance_error*100:.1f}cm, 当前距离{distance:.3f}m)")
            self.move_forward_distance(move_dist, timeout=5.0)
            self.is_positioned = False
            return None
        else:
            # 距离太近，需要后退
            move_dist = distance_error  # 已经是负数
            # 限制最大步长
            if abs(move_dist) > self.step_distance:
                move_dist = -self.step_distance
            rospy.loginfo(f"[精确控制] 后退 {abs(move_dist)*100:.1f}cm (误差{abs(distance_error)*100:.1f}cm, 当前距离{distance:.3f}m)")
            self.move_forward_distance(move_dist, timeout=5.0)
            self.is_positioned = False
            return None
        
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

        预估时间方案：
        1. 如果正在等待运动完成，不重复下发命令
        2. 计算预估执行时间，下发命令后进入等待状态
        """
        # 计算误差
        distance_error = distance - self.target_distance
        angle_error = yaw + offset_x * 30

        # 判断是否到位
        distance_ok = abs(distance_error) <= self.distance_tolerance
        angle_ok = abs(angle_error) <= self.angle_tolerance
        center_ok = abs(offset_x) <= self.center_tolerance

        self._check_positioned(distance_error, angle_error, offset_x)

        # 检查等待超时
        if self.motion_state == MotionState.WAITING:
            if self._check_wait_timeout():
                rospy.logwarn("[cmd_vel] 等待超时，重置状态继续控制")
            else:
                # 正在等待中，不重复下发命令
                elapsed = time.time() - self.motion_start_time
                rospy.loginfo_throttle(
                    0.5,
                    f"[cmd_vel] 等待运动完成... ({elapsed:.1f}/{self.estimated_duration:.1f}s)"
                )
                return None

        # 如果已到位，停止等待状态
        if distance_ok and angle_ok and center_ok:
            if self.motion_state != MotionState.IDLE:
                rospy.loginfo("[cmd_vel] 目标已到位，停止运动")
                self._reset_motion_state()
            return None

        cmd = Twist()

        # 预估运动持续时间
        self.estimated_duration = self._estimate_motion_duration(distance_error, angle_error)

        # 距离控制
        # 注意：cmd_vel中 x > 0 是前进，x < 0 是后退
        # 当 distance > target_distance (太远) 时，应该前进 (正速度)
        if not distance_ok:
            linear_vel = self.kp_linear * distance_error
            linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
            cmd.linear.x = linear_vel

            self.motion_type = "move_forward" if linear_vel > 0 else "move_backward"
            rospy.loginfo(
                f"[cmd_vel] 距离控制: error={distance_error:.3f}m, vx={linear_vel:.3f}m/s, "
                f"预估持续{self.estimated_duration:.1f}s"
            )

            # 进入等待状态
            self.motion_state = MotionState.WAITING
            self.motion_start_time = time.time()

        # 角度控制（只在距离到位时处理角度）
        elif not angle_ok:
            angular_vel = -self.kp_angular * np.radians(angle_error)
            angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
            cmd.angular.z = angular_vel

            self.motion_type = "rotate_right" if angular_vel > 0 else "rotate_left"
            rospy.loginfo(
                f"[cmd_vel] 角度控制: error={angle_error:.2f}deg, wz={angular_vel:.3f}rad/s, "
                f"预估持续{self.estimated_duration:.1f}s"
            )

            # 进入等待状态
            self.motion_state = MotionState.WAITING
            self.motion_start_time = time.time()

        return cmd

    def _check_positioned(self, distance_error, angle_error, offset_x):
        """检查是否已经到位"""
        # 保存用于诊断
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
            # SDK 模式下，指令已在 compute_control 中发送
            pass
        elif self.cmd_vel_pub:
            self.cmd_vel_pub.publish(cmd)

    def stop(self):
        """停止运动"""
        self.is_running = False
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            self.zsi_client.move(0, 0, 0)
            rospy.loginfo("停止运动 (ZSI-1)")
        elif self.use_high_level_sdk and self.sport_client:
            self.sport_client.StopMove()
            rospy.loginfo("停止运动 (SDK)")
        else:
            cmd = Twist()
            self.publish_control(cmd)
            rospy.loginfo("停止运动 (cmd_vel)")

    def stand_up(self):
        """站立"""
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            self.zsi_client.standUp()
            rospy.loginfo("站立 (ZSI-1)")
            self.is_robot_standing = True
            self.is_robot_unlocked = True
        elif self.use_high_level_sdk and self.sport_client:
            self.sport_client.StandUp()
            rospy.loginfo("站立 (SDK)")
            # 更新状态标志
            self.is_robot_standing = True
            self.is_robot_unlocked = True

    def stand_down(self):
        """趴下"""
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            self.zsi_client.lieDown()
            rospy.loginfo("趴下 (ZSI-1)")
            self.is_robot_standing = False
        elif self.use_high_level_sdk and self.sport_client:
            self.sport_client.StandDown()
            rospy.loginfo("趴下 (SDK)")
            # 更新状态标志
            self.is_robot_standing = False

    def balance_stand(self):
        """平衡站立"""
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            self.zsi_client.standUp()
            rospy.loginfo("平衡站立 (ZSI-1)")
        elif self.use_high_level_sdk and self.sport_client:
            self.sport_client.BalanceStand()
            rospy.loginfo("平衡站立 (SDK)")

    def attitude_control(self, roll_vel=0.0, pitch_vel=0.0, yaw_vel=0.0, height_vel=0.0):
        """
        姿态控制
        
        Args:
            roll_vel: 翻滚角速度 (rad/s)
            pitch_vel: 俯仰角速度 (rad/s)
            yaw_vel: 偏航角速度 (rad/s)
            height_vel: 高度变化速度 (m/s)
        """
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            self.zsi_client.attitudeControl(roll_vel, pitch_vel, yaw_vel, height_vel)
            rospy.loginfo(f"[ZSI-1] 姿态控制: roll={roll_vel}, pitch={pitch_vel}, yaw={yaw_vel}, height={height_vel}")
        elif self.use_high_level_sdk and self.sport_client:
            # Unitree SDK 可能没有直接的 attitudeControl 接口
            rospy.logwarn("[SDK] attitudeControl 暂不支持")

    def move_forward_distance(self, distance_m, timeout=10.0):
        """
        前进指定距离（米）- 基于速度+时间控制
        
        业界通用做法（参考Unitree官方SDK和autonomy_stack_go2）：
        1. 先确保机器狗处于运动模式（BalanceStand + Move(0,0,0)）
        2. 使用固定速度发送Move命令
        3. 根据距离和速度计算移动时间
        4. 等待移动完成
        
        Args:
            distance_m: 前进距离（米），正值前进，负值后退
            timeout: 超时时间（秒）
            
        Returns:
            bool: True 表示运动完成
        """
        # ZSI-1 模式
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            return self._zsi1_move_forward(distance_m, timeout)
        
        if not self.use_high_level_sdk or not self.sport_client:
            return True
        
        direction = "前进" if distance_m > 0 else "后退"
        rospy.loginfo(f"[精确控制] {direction} {abs(distance_m)*100:.1f}cm")
        
        # 确保机器人就绪
        if not self.ensure_robot_ready():
            rospy.logwarn("[精确控制] 机器人未就绪，无法移动")
            return False
        
        # 使用保守的固定速度（参考业界做法）
        speed = max(0.1, min(self.max_linear_speed, 0.15))  # 0.1-0.15 m/s
        duration = abs(distance_m) / speed
        
        # 限制最大时间
        duration = min(duration, timeout)
        
        # 确定方向
        vx = speed if distance_m > 0 else -speed
        
        rospy.loginfo(f"[精确控制] 速度 {vx:.3f}m/s, 预计时间 {duration:.2f}s")
        
        # 发送运动命令
        self.sport_client.Move(vx, 0.0, 0.0)
        
        # 等待指定时间（基于速度计算）
        rospy.sleep(duration)
        
        # 停止
        self.sport_client.StopMove()
        actual_distance = speed * duration
        rospy.loginfo(f"[精确控制] {direction}完成，实际移动约 {actual_distance*100:.1f}cm")
        
        return True

    def _zsi1_move_forward(self, distance_m, timeout=10.0):
        """
        ZSI-1 前进指定距离（米）
        
        使用 zsibot_sdk 的 move(vx, vy, yaw_rate) 接口
        """
        direction = "前进" if distance_m > 0 else "后退"
        rospy.loginfo(f"[ZSI-1] {direction} {abs(distance_m)*100:.1f}cm")
        
        # 确保机器人已站立
        if not self.is_robot_standing:
            rospy.loginfo("[ZSI-1] 站立...")
            self.zsi_client.standUp()
            rospy.sleep(2)
            self.is_robot_standing = True
        
        # 使用保守的固定速度
        speed = max(0.1, min(self.max_linear_speed, 0.15))
        duration = abs(distance_m) / speed
        duration = min(duration, timeout)
        
        # 确定方向
        vx = speed if distance_m > 0 else -speed
        
        rospy.loginfo(f"[ZSI-1] 速度 {vx:.3f}m/s, 预计时间 {duration:.2f}s")
        
        # 发送运动命令
        self.zsi_client.move(vx, 0.0, 0.0)
        
        # 等待指定时间
        rospy.sleep(duration)
        
        # 停止
        self.zsi_client.move(0, 0, 0)
        actual_distance = speed * duration
        rospy.loginfo(f"[ZSI-1] {direction}完成，实际移动约 {actual_distance*100:.1f}cm")
        
        return True

    def turn_angle(self, angle_deg, timeout=10.0):
        """
        旋转指定角度（度）- 基于角速度+时间控制
        
        业界通用做法：
        1. 使用固定角速度发送Move命令
        2. 根据角度和角速度计算旋转时间
        3. 等待旋转完成
        
        Args:
            angle_deg: 旋转角度（度），正值左转，负值右转
            timeout: 超时时间（秒）
            
        Returns:
            bool: True 表示旋转完成
        """
        # ZSI-1 模式
        if self.use_zsi1_sdk and hasattr(self, 'zsi_client') and self.zsi_client:
            return self._zsi1_turn_angle(angle_deg, timeout)
        
        if not self.use_high_level_sdk or not self.sport_client:
            return True
        
        direction = "左转" if angle_deg > 0 else "右转"
        rospy.loginfo(f"[精确控制] {direction} {abs(angle_deg):.1f}度")
        
        # 确保机器人就绪
        if not self.ensure_robot_ready():
            rospy.logwarn("[精确控制] 机器人未就绪，无法旋转")
            return False
        
        # 使用固定的角速度
        angular_speed = min(self.max_angular_speed, 0.2)  # 0.2 rad/s
        duration = abs(np.radians(angle_deg)) / angular_speed
        
        # 限制最大时间
        duration = min(duration, timeout)
        
        # 确定方向
        vyaw = angular_speed if angle_deg > 0 else -angular_speed
        
        rospy.loginfo(f"[精确控制] 角速度 {vyaw:.3f}rad/s, 预计时间 {duration:.2f}s")
        
        # 发送旋转命令
        self.sport_client.Move(0.0, 0.0, vyaw)
        
        # 等待指定时间
        rospy.sleep(duration)
        
        # 停止
        self.sport_client.StopMove()
        actual_angle = np.degrees(angular_speed * duration)
        rospy.loginfo(f"[精确控制] {direction}完成，实际旋转约 {actual_angle:.1f}度")
        
        return True

    def _zsi1_turn_angle(self, angle_deg, timeout=10.0):
        """
        ZSI-1 旋转指定角度（度）
        
        使用 zsibot_sdk 的 move(vx, vy, yaw_rate) 接口
        """
        direction = "左转" if angle_deg > 0 else "右转"
        rospy.loginfo(f"[ZSI-1] {direction} {abs(angle_deg):.1f}度")
        
        # 确保机器人已站立
        if not self.is_robot_standing:
            rospy.loginfo("[ZSI-1] 站立...")
            self.zsi_client.standUp()
            rospy.sleep(2)
            self.is_robot_standing = True
        
        # 使用固定的角速度
        angular_speed = min(self.max_angular_speed, 0.2)  # 0.2 rad/s
        duration = abs(np.radians(angle_deg)) / angular_speed
        duration = min(duration, timeout)
        
        # 确定方向
        vyaw = angular_speed if angle_deg > 0 else -angular_speed
        
        rospy.loginfo(f"[ZSI-1] 角速度 {vyaw:.3f}rad/s, 预计时间 {duration:.2f}s")
        
        # 发送旋转命令
        self.zsi_client.move(0.0, 0.0, vyaw)
        
        # 等待指定时间
        rospy.sleep(duration)
        
        # 停止
        self.zsi_client.move(0, 0, 0)
        actual_angle = np.degrees(angular_speed * duration)
        rospy.loginfo(f"[ZSI-1] {direction}完成，实际旋转约 {actual_angle:.1f}度")
        
        return True

    def wait_for_motion_complete(self, timeout=None):
        """
        等待运动执行完成（同步阻塞）

        脚踏实地方案：每次控制指令下发后，等待狗执行完成再进行下一轮检测

        Args:
            timeout: 超时时间（秒），默认使用 wait_timeout

        Returns:
            bool: True 表示运动完成，False 表示超时
        """
        if timeout is None:
            timeout = self.wait_timeout

        if self.motion_state != MotionState.WAITING:
            # 没有在等待运动，直接返回
            rospy.logwarn(f"[Motion] motion_state={self.motion_state}, 不是WAITING, 直接返回")
            return True

        rospy.loginfo(f"[Motion] 开始等待运动完成，超时: {timeout}s, motion_state={self.motion_state}")
        start_time = time.time()
        last_diag_time = 0

        while self.motion_state == MotionState.WAITING and self.is_running:
            # 检查超时
            elapsed = time.time() - start_time
            if elapsed > timeout:
                rospy.logwarn(f"[Motion] 等待运动完成超时 ({elapsed:.1f}s), distance_error={self._last_distance_error:.3f}, is_positioned={self.is_positioned}, positioned_count={self.positioned_count}")
                self._reset_motion_state()
                return False

            # 每2秒打印一次诊断信息
            if elapsed - last_diag_time > 2.0:
                last_diag_time = elapsed
                self._update_robot_state()
                rospy.loginfo(f"[Motion] 诊断: mode={getattr(self.robot_state, 'mode', 'N/A')}, gait={getattr(self.robot_state, 'gait_status', 'N/A')}, is_standing={self.is_robot_standing}, is_unlocked={self.is_robot_unlocked}")

            # 检查是否到位
            if self.is_positioned:
                rospy.loginfo(f"[Motion] 运动完成，耗时: {elapsed:.1f}s")
                self._reset_motion_state()
                return True

            # 短暂休眠，避免 busy wait
            rospy.sleep(0.1)

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
