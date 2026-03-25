#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pose Adapter 主节点 - ROS2 版本 (Humble)
电表巡检核心逻辑：检测 + 追踪 + PnP 位姿 + 控制闭环 + OCR
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from PIL import Image as PILImage, ImageDraw, ImageFont
import queue
import threading
import time
import numpy as np
import yaml

# 添加 src 路径以便导入模块
pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(pkg_dir, 'src'))

from pose_adapter.detector import MeterDetector
from pose_adapter.tracker import DeepSORTTracker
from pose_adapter.pose_solver import PoseSolver
from pose_adapter.controller import MotionController
from pose_adapter.ocr import MeterOCR
from pose_adapter.camera import CameraHandler


class PoseAdapterNode(Node):
    """
    Pose Adapter 主节点 (ROS2)
    
    整合检测、追踪、位姿解算、控制、OCR 的完整流程
    """
    
    def __init__(self):
        """初始化节点"""
        super().__init__('pose_adapter')
        
        self.get_logger().info(f"节点运行 Python: {sys.executable}")
        
        # OpenCV 视频捕获
        self.cap = None
        self.frame_queue = queue.Queue(maxsize=1)  # 保持最新一帧
        self.vision_thread = None
        self.vision_running = False
        
        # 参数
        self._load_params()

        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_shape = None
        self._load_calib_from_file()
        
        # 模块
        self.detector = None
        self.tracker = None
        self.pose_solver = None
        self.controller = None
        self.ocr = None

        # 相机
        self.camera_handler = None
        self.latest_ros_image = None

        # 状态
        self.current_detections = []
        self.current_tracks = []
        self.target_track_id = None
        self.is_running = True
        self.ocr_result = None
        self._frame_count = 0
        self._throttle_ts = {}
        
        # 防死循环
        self._consecutive_timeouts = 0
        self._max_consecutive_timeouts = 3
        self._pause_duration = 2.0
        
        # 新增：调试可视化状态
        self.current_edge_image = None  # 边缘检测结果
        self.current_keypoints = None   # 4个角点坐标
        self.current_pose = None        # PnP解算结果 (distance, yaw)
        self.current_control_cmd = None # 控制命令描述
        
        # RTMP 推流（稍后初始化）
        self.rtmp_writer = None
        # `_init_rtmp_stream()` 在 rtmp_url 为空时会提前 return，
        # 这里先初始化，避免后续 pipeline 访问 `self.rtmp_process` 引发 AttributeError。
        self.rtmp_process = None
        
        # 使用 Go2 相机 SDK
        if self.use_go2_camera:
            self._init_camera()
        else:
            self.get_logger().info("已禁用 Go2 相机，仅使用 camera_image_topic")

        # ROS2
        self._init_ros()
        
        # 定时器替代 ROS1 的 timer
        self._image_timer = None
        self._control_timer = None
        
        self.get_logger().info("Pose Adapter 节点初始化完成")
    
    def _load_params(self):
        """加载参数"""
        # 相机参数
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        
        # 电表尺寸
        self.declare_parameter('meter_width', 0.2)
        self.declare_parameter('meter_height', 0.3)
        
        # 控制参数
        self.declare_parameter('target_distance', 1.7)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('target_ratio', 0.65)
        self.declare_parameter('angle_tolerance', 2.0)
        self.declare_parameter('center_tolerance', 0.05)
        self.declare_parameter('loop_hz', 5)
        
        # 模型
        self.declare_parameter('yolo_model_path', '')
        self.declare_parameter('use_paddle_ocr', False)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('detection_iou_threshold', 0.45)
        self.declare_parameter('min_bbox_area_ratio', 0.05)
        self.declare_parameter('keypoint_method', 'bbox')
        self.declare_parameter('use_gpu', True)
        
        # 标定
        self.declare_parameter('calib_file', '')
        
        # 网络
        self.declare_parameter('network_interface', '')
        self.declare_parameter('use_go2_camera', False)
        
        # 机器狗
        self.declare_parameter('body_type', os.environ.get('BODY', ''))
        
        # SDK 控制
        self.declare_parameter('use_high_level_sdk', True)
        self.declare_parameter('disable_obstacle_avoidance_on_start', True)
        self.declare_parameter('use_classic_walk', False)
        self.declare_parameter('speed_level', 0)
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('min_linear_speed', 0.0)
        self.declare_parameter('max_angular_speed', 0.25)
        self.declare_parameter('step_distance', 0.2)
        self.declare_parameter('step_angle', 5.0)

        # 追踪参数（用于 DeepSORTTracker）
        self.declare_parameter('tracking_max_age', 30)
        self.declare_parameter('tracking_min_hits', 3)
        self.declare_parameter('tracking_iou_threshold', 0.3)
        
        # 图像话题
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        
        # 保存路径
        self.declare_parameter('image_save_path', '/tmp/pose_adapter_images')
        
        # RTSP 流
        self.declare_parameter('rtsp_url', 'rtsp://192.168.234.1:8554/test')

        # 获取参数值
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.meter_width = self.get_parameter('meter_width').value
        self.meter_height = self.get_parameter('meter_height').value
        self.target_distance = self.get_parameter('target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.target_ratio = self.get_parameter('target_ratio').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.loop_hz = self.get_parameter('loop_hz').value
        self.yolo_model_path = self.get_parameter('yolo_model_path').value
        if not self.yolo_model_path:
            raise RuntimeError("未设置 yolo_model_path，当前要求必须提供有效模型路径。")
        if not os.path.isabs(self.yolo_model_path):
            # 相对路径按工作区根目录解析
            self.yolo_model_path = os.path.abspath(os.path.join(os.getcwd(), self.yolo_model_path))
        if not os.path.isfile(self.yolo_model_path):
            raise FileNotFoundError(
                f"模型文件不存在: {self.yolo_model_path}。"
                "请确认路径（例如 /home/nvidia/stephen/PoseAdapter/model/best.pt）并重新启动。"
            )
        self.use_paddle_ocr = self.get_parameter('use_paddle_ocr').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.detection_iou_threshold = self.get_parameter('detection_iou_threshold').value
        self.min_bbox_area_ratio = self.get_parameter('min_bbox_area_ratio').value
        self.keypoint_method = self.get_parameter('keypoint_method').value
        self.use_gpu = self.get_parameter('use_gpu').value
        self.calib_file = self.get_parameter('calib_file').value
        self.network_interface = self.get_parameter('network_interface').value
        self.use_go2_camera = self.get_parameter('use_go2_camera').value
        self.body_type = self.get_parameter('body_type').value
        self.use_high_level_sdk = self.get_parameter('use_high_level_sdk').value
        self.disable_obstacle_avoidance_on_start = self.get_parameter('disable_obstacle_avoidance_on_start').value
        self.use_classic_walk = self.get_parameter('use_classic_walk').value
        self.speed_level = self.get_parameter('speed_level').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.step_distance = self.get_parameter('step_distance').value
        self.step_angle = self.get_parameter('step_angle').value
        self.camera_image_topic = self.get_parameter('camera_image_topic').value
        self.image_save_path = self.get_parameter('image_save_path').value
        self.rtsp_url = self.get_parameter('rtsp_url').value

        # tracking
        self.tracking_max_age = self.get_parameter('tracking_max_age').value
        self.tracking_min_hits = self.get_parameter('tracking_min_hits').value
        self.tracking_iou_threshold = self.get_parameter('tracking_iou_threshold').value
        
        # 调试可视化参数
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('rtmp_url', '')
        
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.rtmp_url = self.get_parameter('rtmp_url').value
        
        # 初始化 RTMP 推流
        self._apply_config_yaml_overrides()
        self._init_rtmp_stream()

        # 性能统计
        self._total_loop_time = 0.0
        self._loop_count = 0
        self._debug_image_interval = 5

    def _apply_config_yaml_overrides(self):
        """
        从仓库根目录的 `config/params.yaml` 读取“业务配置”，覆盖当前 ROS2 参数值。

        这样你可以继续使用：
          camera.width / meter.width / control.* / unitree_sdk.* / detection.*
          tracking.*
          display.publish_debug_image / display.rtmp_url
        这种嵌套格式，而不必强制改成 ROS2 扁平参数名。
        """
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
        candidates = [
            os.path.join(os.getcwd(), 'config', 'params.yaml'),
            os.path.join(repo_root, 'config', 'params.yaml'),
        ]
        cfg_path = next((p for p in candidates if os.path.isfile(p)), None)
        if not cfg_path:
            return

        try:
            with open(cfg_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f"加载 config/params.yaml 失败：{e}")
            return

        if not isinstance(cfg, dict):
            return

        # camera
        cam = cfg.get('camera', {})
        if isinstance(cam, dict):
            self.camera_width = cam.get('width', self.camera_width)
            self.camera_height = cam.get('height', self.camera_height)

        # meter
        meter = cfg.get('meter', {})
        if isinstance(meter, dict):
            self.meter_width = meter.get('width', self.meter_width)
            self.meter_height = meter.get('height', self.meter_height)

        # control
        control = cfg.get('control', {})
        if isinstance(control, dict):
            self.target_distance = control.get('target_distance', self.target_distance)
            self.target_ratio = control.get('target_ratio', self.target_ratio)
            self.distance_tolerance = control.get('distance_tolerance', self.distance_tolerance)
            self.angle_tolerance = control.get('angle_tolerance', self.angle_tolerance)
            self.center_tolerance = control.get('center_tolerance', self.center_tolerance)
            self.max_linear_speed = control.get('max_linear_speed', self.max_linear_speed)
            self.min_linear_speed = control.get('min_linear_speed', self.min_linear_speed)
            self.max_angular_speed = control.get('max_angular_speed', self.max_angular_speed)
            # step_distance / step_angle 不在该文件时保持不变
            self.step_distance = control.get('step_distance', self.step_distance)
            self.step_angle = control.get('step_angle', self.step_angle)

        # unitree_sdk
        unit = cfg.get('unitree_sdk', {})
        if isinstance(unit, dict):
            self.use_high_level_sdk = unit.get('enabled', self.use_high_level_sdk)
            self.network_interface = unit.get('interface', self.network_interface)
            self.disable_obstacle_avoidance_on_start = unit.get(
                'disable_obstacle_avoidance_on_start',
                self.disable_obstacle_avoidance_on_start
            )
            self.use_classic_walk = unit.get('use_classic_walk', self.use_classic_walk)
            self.speed_level = unit.get('speed_level', self.speed_level)

        # detection
        detection = cfg.get('detection', {})
        if isinstance(detection, dict):
            self.conf_threshold = detection.get('conf_threshold', self.conf_threshold)
            self.detection_iou_threshold = detection.get('iou_threshold', self.detection_iou_threshold)

        # tracking
        tracking = cfg.get('tracking', {})
        if isinstance(tracking, dict):
            self.tracking_max_age = tracking.get('max_age', self.tracking_max_age)
            self.tracking_min_hits = tracking.get('min_hits', self.tracking_min_hits)
            self.tracking_iou_threshold = tracking.get('iou_threshold', self.tracking_iou_threshold)

        # display（兼容你当前的 display: {publish_debug_image, rtmp_url}）
        # 注意：不能用 cfg.get('display', {})，否则 display 键缺失时会走到空 dict 分支，
        # 从而跳过对顶层 publish_debug_image / rtmp_url 的兼容读取。
        display = cfg.get('display', None)
        if isinstance(display, dict):
            self.publish_debug_image = display.get('publish_debug_image', self.publish_debug_image)
            self.rtmp_url = display.get('rtmp_url', self.rtmp_url)
        else:
            # 兼容旧版：publish_debug_image / rtmp_url 位于顶层
            if 'publish_debug_image' in cfg:
                self.publish_debug_image = cfg.get('publish_debug_image', self.publish_debug_image)
            if 'rtmp_url' in cfg:
                self.rtmp_url = cfg.get('rtmp_url', self.rtmp_url)

        # 只打印“是否设置”，避免在日志里泄露签名类参数
        self.get_logger().info(
            f"[Config] 覆盖参数生效(来自{cfg_path})：publish_debug_image={self.publish_debug_image}, "
            f"rtmp_url_set={bool(self.rtmp_url)}"
        )

    def _init_rtsp_stream(self):
        """初始化 RTSP 视频流并启动生产者线程"""
        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开 RTSP 流: {self.rtsp_url}")
            self.cap = None
            return
        
        # 设置低延迟
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.get_logger().info(f"RTSP 流已打开: {self.rtsp_url}")
        
        # 启动生产者线程
        self.vision_running = True
        self.vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
        self.vision_thread.start()
        self.get_logger().info("视觉生产者线程已启动")
    
    def _vision_loop(self):
        """生产者线程：持续从 RTSP 拉流"""
        while self.vision_running:
            if self.cap is None or not self.cap.isOpened():
                self.get_logger().warn("RTSP 流断开，尝试重连...")
                self.cap = cv2.VideoCapture(self.rtsp_url)
                if self.cap:
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                time.sleep(1)
                continue
            
            ret, frame = self.cap.read()
            if ret and frame is not None:
                # 入队，丢弃旧帧
                try:
                    if self.frame_queue.full():
                        self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
                self.frame_queue.put(frame)
            else:
                self.get_logger().warn("RTSP 读取失败，尝试重连...")
                self.cap = cv2.VideoCapture(self.rtsp_url)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                time.sleep(0.5)
    
    def _init_rtmp_stream(self):
        """初始化 RTMP 推流"""
        self.get_logger().info(f"[RTMP] 初始化检查... rtmp_url='{self.rtmp_url}'")

        # 若已有 ffmpeg 正在运行，则不重复启动
        if getattr(self, "rtmp_process", None) is not None:
            try:
                if self.rtmp_process.poll() is None:
                    return
            except Exception:
                pass
        
        # RTMP 推流初始化（独立于 publish_debug_image）
        rtmp_url = self.rtmp_url
        if not rtmp_url:
            self.get_logger().info("未设置 rtmp_url，跳过推流初始化")
            return
        
        self.get_logger().info(f"[RTMP] rtmp_url 已配置，准备启动推流...")
        
        try:
            # 检查 ffmpeg 是否可用
            import subprocess
            result = subprocess.run(['which', 'ffmpeg'], capture_output=True)
            if result.returncode != 0:
                self.get_logger().warn("[RTMP] ffmpeg 未安装，无法推流")
                return
            
            # 使用 ffmpeg 推流
            cmd = [
                'ffmpeg',
                '-f', 'rawvideo',
                '-pix_fmt', 'bgr24',
                '-s', f'{self.camera_width}x{self.camera_height}',
                '-r', '10',
                '-i', '-',
                '-an',
                '-c:v', 'libx264',
                # 雪花屏常见原因之一是编码输出格式/档位不兼容（例如 yuv444p + High）。
                # 这里强制输出 yuv420p + baseline，并关掉 B 帧，且定期插入关键帧头信息，提升播放器兼容性。
                '-pix_fmt', 'yuv420p',
                '-profile:v', 'baseline',
                '-level', '3.1',
                # 关键帧越频繁，播放器越容易快速“对齐”解码（减少雪花/黑屏）。
                '-g', '10',
                '-keyint_min', '10',
                '-sc_threshold', '0',
                '-bf', '0',
                '-preset', 'ultrafast',
                '-tune', 'zerolatency',
                '-b', '1000k',
                '-x264-params', 'repeat-headers=1',
                '-f', 'flv',
                rtmp_url
            ]

            # 关键：不要吞掉 ffmpeg 的 stderr，便于定位 RTMP 是否连接成功/是否鉴权失败。
            log_dir = os.path.join(os.getcwd(), 'data', 'logs')
            os.makedirs(log_dir, exist_ok=True)
            rtmp_ffmpeg_log_path = os.path.join(log_dir, 'pose_adapter_rtmp_ffmpeg.log')
            self._rtmp_ffmpeg_log_fp = open(rtmp_ffmpeg_log_path, 'a', buffering=1)
            
            self.rtmp_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stderr=self._rtmp_ffmpeg_log_fp
            )
            self.get_logger().info(f"RTMP 推流已启动: {rtmp_url}")
        except Exception as e:
            self.get_logger().warn(f"RTMP 推流初始化失败: {e}")
            self.rtmp_process = None
    
    def _push_rtmp_frame(self, cv_image):
        """推送帧到 RTMP"""
        if self.rtmp_process is None or self.rtmp_process.stdin is None:
            # 兜底：若 ffmpeg 尚未就绪/已崩，则尝试重新初始化（避免写入无效 stdin）
            now = time.time()
            if not hasattr(self, "_rtmp_reinit_ts"):
                self._rtmp_reinit_ts = 0.0
            if now - self._rtmp_reinit_ts > 5.0:
                self._rtmp_reinit_ts = now
                self.get_logger().info(f"[RTMP] 重试初始化 ffmpeg（frame={getattr(self,'_frame_count',None)}）")
                try:
                    self._init_rtmp_stream()
                except Exception:
                    pass

            if self.rtmp_process is None or self.rtmp_process.stdin is None:
                self._log_throttle(
                    "warn",
                    "rtmp_push_skip",
                    2.0,
                    f"[RTMP] 跳过推流：rtmp_process={self.rtmp_process is not None}, stdin={getattr(self.rtmp_process,'stdin',None) is not None}, frame={getattr(self,'_frame_count',None)}"
                )
                return
        
        try:
            # rawvideo(bgr24) 需要严格的帧字节数，否则会导致“雪花/马赛克/Packet corrupt”
            expected_size = int(self.camera_width) * int(self.camera_height) * 3
            img = cv_image
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif img.ndim == 3:
                if img.shape[2] == 4:
                    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                elif img.shape[2] != 3:
                    img = img[:, :, :3]
            else:
                # 兜底：无法识别通道，直接跳过
                self._log_throttle("warn", "rtmp_bad_ndim", 2.0, f"[RTMP] 跳过推流：ndim={getattr(img,'ndim',None)}")
                return

            if img.shape[0] != self.camera_height or img.shape[1] != self.camera_width:
                img = cv2.resize(img, (int(self.camera_width), int(self.camera_height)), interpolation=cv2.INTER_LINEAR)

            if img.dtype != np.uint8:
                img = img.astype(np.uint8, copy=False)

            data = img.tobytes()
            if len(data) != expected_size:
                self._log_throttle(
                    "warn",
                    "rtmp_frame_size_mismatch",
                    2.0,
                    f"[RTMP] 帧大小不匹配，len(data)={len(data)} expected={expected_size} frame={getattr(self,'_frame_count',None)}"
                )
                return

            # 关键：保证写入不发生“部分写入”。使用 os.write 循环直到写完整帧。
            import os
            fd = self.rtmp_process.stdin.fileno()
            buf = memoryview(data)
            off = 0
            while off < len(buf):
                n = os.write(fd, buf[off:])
                if n <= 0:
                    raise RuntimeError("ffmpeg stdin write returned 0")
                off += n
            # 用节流方式确认：_push_rtmp_frame 确实被调用
            self._log_throttle(
                "info",
                "rtmp_write_called",
                1.0,
                f"[RTMP] 向 ffmpeg 写入触发（frame={self._frame_count}）"
            )
        except Exception as e:
            self.get_logger().warn(f"RTMP 推流失败: {e}")
    
    def _load_calib_from_file(self):
        """从标定文件加载相机内参"""
        if not self.calib_file:
            raise RuntimeError(
                "未设置 calib_file。请在 launch 中指定有效的标定文件路径，例如："
                "/home/nvidia/stephen/PoseAdapter/src/calibrate/calibration_results/rtsp_camera_calib.yaml"
            )

        if not os.path.isfile(self.calib_file):
            raise FileNotFoundError(
                f"标定文件不存在: {self.calib_file}。"
                "请检查路径是否正确并确认文件已生成。"
            )

        try:
            with open(self.calib_file, 'r') as f:
                calib_data = yaml.safe_load(f)

            cam_matrix = calib_data['camera_matrix']['data']
            self.camera_matrix = np.array([
                [cam_matrix[0], cam_matrix[1], cam_matrix[2]],
                [cam_matrix[3], cam_matrix[4], cam_matrix[5]],
                [cam_matrix[6], cam_matrix[7], cam_matrix[8]]
            ], dtype=np.float64)

            dist_coeffs = calib_data['distortion_coefficients']['data']
            self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64)

            w = calib_data.get('image_width', self.camera_width)
            h = calib_data.get('image_height', self.camera_height)
            self.image_shape = (h, w)

            self.get_logger().info(f"从标定文件加载相机内参: {self.calib_file}")
        except Exception as e:
            raise RuntimeError(f"标定文件加载失败: {self.calib_file}，错误: {e}") from e

    def _init_camera(self):
        """使用 Unitree SDK 初始化 Go2 前置相机"""
        config = {
            'camera': {'image_save_path': self.image_save_path},
            'robot': {'network_interface': self.network_interface}
        }

        self.camera_handler = CameraHandler(config)
        if not self.camera_handler.is_available():
            self.get_logger().warn("Go2 相机不可用")
    
    def _init_ros(self):
        """初始化 ROS2 发布/订阅"""
        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 发布
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.debug_image_pub = self.create_publisher(Image, '/pose_adapter/debug_image', qos)

        # RTSP 模式：不再订阅 ROS 话题
        # 图像通过 RTSP 流直接获取
        self.get_logger().info(f"图像来源: RTSP ({self.rtsp_url})")

        self.get_logger().info("使用同步顺序执行：取图 → 检测 → 追踪 → PnP → 控制(等待完成) → 循环")
        
        # 初始化 RTSP 视频流
        self._init_rtsp_stream()

    def _init_modules(self):
        """初始化各模块"""
        if self.camera_matrix is None:
            self.get_logger().warn("使用默认相机参数，建议进行标定")
            fx = 700.0
            fy = 700.0
            cx = self.camera_width / 2
            cy = self.camera_height / 2
            self.camera_matrix = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ], dtype=np.float64)
            self.dist_coeffs = np.array([0.1, -0.2, 0, 0, 0.05], dtype=np.float64)
            self.image_shape = (self.camera_height, self.camera_width)
        
        self.detector = MeterDetector(
            model_path=self.yolo_model_path if self.yolo_model_path else None,
            conf_threshold=self.conf_threshold,
            min_area_ratio=self.min_bbox_area_ratio,
            use_gpu=self.use_gpu,
            iou_threshold=self.detection_iou_threshold,
            keypoint_method=self.keypoint_method,
            logger=self.get_logger(),
        )
        
        self.get_logger().info(f"检测器 GPU: {self.use_gpu}, 关键点方式: {self.keypoint_method}")
        self.tracker = DeepSORTTracker(
            max_age=self.tracking_max_age,
            min_hits=self.tracking_min_hits,
            iou_threshold=self.tracking_iou_threshold,
        )
        self.pose_solver = PoseSolver(
            self.camera_matrix,
            self.dist_coeffs,
            (self.meter_width, self.meter_height),
            logger=self.get_logger(),
        )
        
        # 确定网络接口
        interface = self.network_interface if self.network_interface else "eth1"
        if not interface:
            interface = "eth0"
        
        self.controller = MotionController(
            target_distance=self.target_distance,
            target_ratio=self.target_ratio,
            distance_tolerance=self.distance_tolerance,
            angle_tolerance=self.angle_tolerance,
            center_tolerance=self.center_tolerance,
            max_linear_speed=self.max_linear_speed,
            min_linear_speed=self.min_linear_speed,
            max_angular_speed=self.max_angular_speed,
            step_distance=self.step_distance,
            step_angle=self.step_angle,
            use_high_level_sdk=self.use_high_level_sdk,
            interface_name=interface,
            disable_obstacle_avoidance_on_start=self.disable_obstacle_avoidance_on_start,
            use_classic_walk=self.use_classic_walk,
            speed_level=self.speed_level,
            body_type=self.body_type,
        )
        self.ocr = None
        
        self.get_logger().info("所有模块初始化完成")

    def _ros_image_callback(self, msg):
        """ROS 图像话题回调"""
        try:
            cv_image = self._ros_image_to_bgr(msg)
            self.latest_ros_image = cv_image
        except Exception as e:
            self.get_logger().error(f"ROS 图像转换失败: {e}")

    @staticmethod
    def _ros_image_to_bgr(msg):
        if msg.encoding not in ("bgr8", "rgb8"):
            raise ValueError(f"暂不支持图像编码: {msg.encoding}")
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        if msg.encoding == "rgb8":
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr

    @staticmethod
    def _bgr_to_ros_image(frame):
        msg = Image()
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.tobytes()
        return msg
    
    @staticmethod
    def _draw_chinese_text(img, text, position, font_size=24, text_color=(255, 255, 0)):
        """在图像上绘制中文文本"""
        # 转换为 PIL 图像
        pil_img = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil_img)
        
        # 尝试加载中文字体
        font_path = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
        try:
            font = ImageFont.truetype(font_path, font_size)
        except:
            font = ImageFont.load_default()
        
        # 绘制文本
        draw.text(position, text, font=font, fill=text_color + (255,))
        
        # 转回 OpenCV 图像
        return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

    def _get_image(self):
        """消费者：优先从队列消费最新 RTSP 帧"""
        try:
            frame = self.frame_queue.get_nowait()
            return frame
        except queue.Empty:
            pass
        
        # 回退到 ROS 话题
        if self.camera_image_topic and self.latest_ros_image is not None:
            return self.latest_ros_image.copy()
        
        return None

    def _log_throttle(self, level, key, interval_sec, message):
        now = time.time()
        last = self._throttle_ts.get(key, 0.0)
        if now - last < interval_sec:
            return
        self._throttle_ts[key] = now
        if level == "warn":
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _select_target(self, tracks):
        """选择追踪目标"""
        if not tracks:
            return None
        
        best_track = None
        min_offset = float('inf')
        
        for track_id, bbox, conf in tracks:
            cx = (bbox[0] + bbox[2]) / 2
            cy = (bbox[1] + bbox[3]) / 2
            image_cx = self.camera_width / 2
            image_cy = self.camera_height / 2
            
            offset = abs(cx - image_cx) + abs(cy - image_cy)
            if offset < min_offset:
                min_offset = offset
                best_track = track_id
        
        self.get_logger().info(f"选择目标 ID: {best_track}")
        return best_track

    def _trigger_ocr(self, bbox):
        """触发 OCR"""
        if self.ocr_result is not None:
            return

        if self.ocr is None:
            self.ocr = MeterOCR(use_paddle=self.use_paddle_ocr, logger=self.get_logger())
        self.get_logger().info("触发 OCR 识别...")

    def _add_debug_overlay(self, debug_image):
        """发布调试图像（带完整可视化）"""
        # 添加日志：记录当前有哪些数据可用于绘制
        det_count = len(self.current_detections) if self.current_detections else 0
        track_count = len(self.current_tracks) if self.current_tracks else 0
        has_edge = self.current_edge_image is not None
        has_keypts = self.current_keypoints is not None and len(self.current_keypoints) == 4
        has_pose = self.current_pose is not None and self.current_pose.get('success')
        has_cmd = self.current_control_cmd is not None
        
        self.get_logger().info(
            f"[Overlay] draws: detections={det_count}, tracks={track_count}, "
            f"edge={has_edge}, keypoints={has_keypts}, pose={has_pose}, cmd={has_cmd}"
        )
        
        debug_image = debug_image.copy()
        
        # ========== 1. 边缘检测可视化 ==========
        if self.current_edge_image is not None:
            # 将边缘图像叠加到右上角小图
            h, w = debug_image.shape[:2]
            edge_h, edge_w = self.current_edge_image.shape[:2]
            # 缩放边缘图
            scale = min(200 / edge_h, 300 / edge_w)
            edge_small = cv2.resize(self.current_edge_image, (int(edge_w * scale), int(edge_h * scale)))
            # 叠加到左上角
            roi_y, roi_x = 10, 10
            roi_h, roi_w = edge_small.shape[:2]
            if roi_y + roi_h < h and roi_x + roi_w < w:
                debug_image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w] = edge_small
            # 边框
            cv2.rectangle(debug_image, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (0, 255, 255), 1)
            cv2.putText(debug_image, "Edge", (roi_x, roi_y-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # ========== 2. YOLO 检测框 ==========
        for x1, y1, x2, y2, conf, cls in self.current_detections:
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug_image, f"YOLO:{conf:.2f}", (x1, y1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # ========== 3. 追踪框 ==========
        for track_id, bbox, conf in self.current_tracks:
            x1, y1, x2, y2 = bbox
            color = (0, 0, 255) if track_id == self.target_track_id else (255, 0, 0)
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(debug_image, f"ID:{track_id}", (x1, y1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # ========== 4. 关键点（4个角点） ==========
        if self.current_keypoints is not None and len(self.current_keypoints) == 4:
            for idx, (cx, cy) in enumerate(self.current_keypoints):
                cv2.circle(debug_image, (int(cx), int(cy)), 8, (0, 255, 255), -1)
                cv2.putText(debug_image, str(idx), (int(cx)+8, int(cy)-8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            # 连接角点形成矩形
            pts = np.array(self.current_keypoints, dtype=np.int32)
            cv2.polylines(debug_image, [pts], True, (0, 255, 255), 2)
        
        # ========== 5. PnP 结果（距离 + yaw） ==========
        if self.current_pose is not None and self.current_pose.get('success'):
            distance = self.current_pose.get('distance', 0)
            yaw = self.current_pose.get('yaw', 0)
            pose_text = f"Dist: {distance:.2f}m | Yaw: {yaw:.1f}deg"
            # 绘制在右上角
            cv2.rectangle(debug_image, (debug_image.shape[1]-250, 20), (debug_image.shape[1]-10, 50), (0, 0, 0), -1)
            debug_image = self._draw_chinese_text(debug_image, pose_text, (debug_image.shape[1]-240, 25), 22, (0, 255, 0))
        
        # ========== 6. 控制命令（中文）==========
        if self.current_control_cmd:
            cmd_text = self.current_control_cmd
            # 绘制在底部中间，先用英文估算位置
            text_y = debug_image.shape[0] - 30
            # 背景条
            cv2.rectangle(debug_image, (100, text_y-25), (debug_image.shape[1]-100, text_y+10), (0, 0, 0), -1)
            # 使用 PIL 绘制中文
            text_x = (debug_image.shape[1] - 200) // 2
            debug_image = self._draw_chinese_text(debug_image, cmd_text, (text_x, text_y-5), 28, (255, 255, 0))
        
        # ========== 7. 状态栏 ==========
        status_text = f"Tracks: {len(self.current_tracks)}"
        if self.target_track_id:
            status_text += f" | Target: {self.target_track_id}"
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    def _execute_pipeline(self):
        """执行完整检测→追踪→PnP→控制流程"""
        import time
        pipeline_start = time.time()
        
        # 1. 取图
        cv_image = self._get_image()
        if cv_image is None:
            self._log_throttle("warn", "wait_image", 2.0, "[Pipeline] 等待图像...")
            return

        # 2. 尺寸对齐：ffmpeg 的 -s 参数依赖 camera_width/height
        # 如果 RTSP 帧的分辨率与之不一致，不做 resize 会导致客户端雪花屏（字节错位）。
        h, w = cv_image.shape[:2]
        if (w != self.camera_width) or (h != self.camera_height):
            self._log_throttle(
                "warn",
                "rtsp_size_mismatch",
                2.0,
                f"[Pipeline] 图像尺寸不匹配，将 resize: got={w}x{h} expect={self.camera_width}x{self.camera_height}"
            )
            cv_image = cv2.resize(cv_image, (self.camera_width, self.camera_height), interpolation=cv2.INTER_LINEAR)

        # 3. 格式对齐：保证推流输入严格是 uint8 的 BGR 三通道
        # 否则 rawvideo(BGR24) 的字节边界会错，导致“雪花/马赛克/Packet corrupt”
        if cv_image.ndim == 2:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        elif cv_image.ndim == 3:
            if cv_image.shape[2] == 4:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            elif cv_image.shape[2] != 3:
                cv_image = cv_image[:, :, :3]

        if cv_image.dtype != np.uint8:
            cv_image = cv_image.astype(np.uint8, copy=False)

        self.image_shape = cv_image.shape[:2]
        
        if self.detector is None:
            self._init_modules()
        
        self.get_logger().info("[Pipeline] === 步骤1: 取图完成 ===")
        
        # 2. YOLO检测
        detections = self.detector.detect(cv_image)
        self.get_logger().info(f"[Pipeline] === 步骤2: YOLO检测完成，检测到 {len(detections)} 个目标 ===")
        
        # 过滤异常检测框
        h, w = self.image_shape
        valid_detections = []
        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            bw = max(0, x2 - x1)
            bh = max(0, y2 - y1)
            if bw == 0 or bh == 0:
                continue
            area_ratio = float(bw * bh) / float(w * h)
            if area_ratio >= 0.9:
                self.get_logger().warn(f"[Pipeline] 检测框过大({area_ratio:.2f})，丢弃")
                continue
            valid_detections.append(det)
        
        self.current_detections = valid_detections
        
        # 3. 追踪
        if valid_detections:
            tracks = self.tracker.update(valid_detections)
            self.current_tracks = tracks
            
            if self.target_track_id is None and tracks:
                self.target_track_id = self._select_target(tracks)
        else:
            self.current_tracks = []
        
        self.get_logger().info(f"[Pipeline] === 步骤3: 追踪完成，{len(self.current_tracks)} 个追踪 ===")
        
        # 无目标时：清空姿态/控制的上一次结果，并且仍继续做推流（背景也要推）。
        if len(self.current_tracks) == 0:
            self.target_track_id = None
            self.current_edge_image = None
            self.current_keypoints = None
            self.current_pose = None
            self.current_control_cmd = None
            self._log_throttle("warn", "no_target_idle", 2.0, "[Pipeline] 无目标，不下发任何控制指令")
        
        # 图像可视化/推流：RTMP 每个循环都推，保证无检测时也能维持流畅性
        self._frame_count += 1
        debug_image = cv_image.copy()
        
        # ROS topic 发布（可选节流）
        if self.publish_debug_image and self._frame_count % self._debug_image_interval == 0:
            try:
                debug_msg = self._bgr_to_ros_image(debug_image)
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = "camera"
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"调试图像发布失败: {e}")
        
        # RTMP 推流（不节流，维持直播不断帧）
        self._push_rtmp_frame(debug_image)
        
        # 4. PnP位姿解算 + 5. 控制 + 绘制 overlay
        if self.target_track_id is not None:
            target_track = None
            for track_id, bbox, conf in self.current_tracks:
                if track_id == self.target_track_id:
                    target_track = (track_id, bbox, conf)
                    break
            
            if target_track is None:
                self.get_logger().warn("[Pipeline] 丢失目标")
                self.target_track_id = None
                return
            
            _, bbox, conf = target_track
            
            # 检查bbox异常
            x1, y1, x2, y2 = bbox
            bw = max(0, x2 - x1)
            bh = max(0, y2 - y1)
            if bw > 0 and bh > 0:
                area_ratio = float(bw * bh) / float(w * h)
                if area_ratio >= 0.9:
                    self.get_logger().warn(f"[Pipeline] 目标框过大({area_ratio:.2f})")
                    return
            
            # PnP
            keypoints = self.detector.extract_corners(cv_image, bbox)
            # 保存关键点用于可视化
            self.current_keypoints = keypoints
            # 获取边缘检测结果
            self.current_edge_image = self.detector.get_edge_image(cv_image, bbox)
            
            pose = self.pose_solver.solve(bbox, self.image_shape, keypoints=keypoints)
            # 保存PnP结果用于可视化
            self.current_pose = pose
            self.get_logger().info("[Pipeline] === 步骤4: PnP位姿解算完成 ===")
            
            # 控制
            ratio = self.pose_solver.get_target_bbox_ratio(bbox, self.image_shape)
            offset = self.pose_solver.get_center_offset(bbox, self.image_shape)
            cmd = self.controller.compute_control(pose, ratio, offset)
            
            # 保存控制命令描述用于可视化
            if pose.get('success'):
                distance = pose.get('distance', 0)
                yaw = pose.get('yaw', 0)
                dist_diff = abs(distance - self.target_distance)
                if abs(yaw) > self.controller.angle_tolerance:
                    self.current_control_cmd = f"ROTATE {yaw:.1f}deg"
                elif dist_diff > self.distance_tolerance:
                    direction = "FORWARD" if distance > self.target_distance else "BACKWARD"
                    self.current_control_cmd = f"{direction} {dist_diff:.2f}m"
                else:
                    self.current_control_cmd = "IN_POSITION"
            else:
                self.current_control_cmd = "STOP"
            
            if cmd is not None:
                self.cmd_vel_pub.publish(cmd)
            
            self.get_logger().info("[Pipeline] === 步骤5: 控制指令已下发，等待执行完成 ===")
            
            # 等待运动完成
            motion_complete = self.controller.wait_for_motion_complete()
            if motion_complete:
                self.get_logger().info("[Pipeline] 运动执行完成")
                self._consecutive_timeouts = 0
            else:
                self.get_logger().warn("[Pipeline] 运动执行超时")
                self._consecutive_timeouts += 1
                
                if self._consecutive_timeouts >= self._max_consecutive_timeouts:
                    self.get_logger().error(f"[Pipeline] 连续超时{self._max_consecutive_timeouts}次")
                    self.controller.stop()
                    self._consecutive_timeouts = 0
            
            # OCR
            if self.controller.is_ready_for_ocr():
                self._trigger_ocr(bbox)
            
            # 绘制 overlay（在 PnP 和控制计算完成后）
            if len(self.current_detections) > 0:
                self._add_debug_overlay(debug_image)
                # 推流带 overlay 的帧
                self._push_rtmp_frame(debug_image)
        else:
            self._log_throttle("info", "no_target_idle", 2.0, "[Pipeline] 无目标，不下发任何控制指令")
        
        # 统计耗时
        pipeline_elapsed = (time.time() - pipeline_start) * 1000
        self._total_loop_time += pipeline_elapsed
        self._loop_count += 1
        avg_time = self._total_loop_time / self._loop_count if self._loop_count > 0 else 0
        
        self.get_logger().info(f"[Pipeline] 本轮耗时: {pipeline_elapsed:.1f}ms, 平均: {avg_time:.1f}ms")

    def run(self):
        """主循环"""
        self.get_logger().info("启动同步主循环")
        
        # 创建定时器 (ROS2)
        timer_period = 1.0 / self.loop_hz
        self._timer = self.create_timer(timer_period, self._execute_pipeline)
        
        # spin 阻塞
        rclpy.spin(self)

    def shutdown(self):
        """关闭节点"""
        self.is_running = False
        
        # 停止 RTMP 推流
        if hasattr(self, 'rtmp_process') and self.rtmp_process is not None:
            try:
                self.rtmp_process.stdin.close()
                self.rtmp_process.wait(timeout=2)
            except:
                self.rtmp_process.terminate()
            self.get_logger().info("RTMP 推流已关闭")
        
        if hasattr(self, '_rtmp_ffmpeg_log_fp') and self._rtmp_ffmpeg_log_fp:
            try:
                self._rtmp_ffmpeg_log_fp.close()
            except:
                pass
        
        # 停止视觉生产者线程
        self.vision_running = False
        if self.vision_thread:
            self.vision_thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
        
        if self.controller:
            self.controller.is_running = False
            self.controller.stop()
        self.get_logger().info("Pose Adapter 节点已关闭")


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = PoseAdapterNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
