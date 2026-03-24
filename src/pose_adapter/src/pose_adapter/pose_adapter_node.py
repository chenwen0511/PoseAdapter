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
# from sensor_msgs.msg import Image  # RTSP 模式不需要
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
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
        
        # 参数
        self._load_params()

        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_shape = None
        self._load_calib_from_file()
        
        # CV 桥接
        self.bridge = CvBridge()
        
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
        
        # 防死循环
        self._consecutive_timeouts = 0
        self._max_consecutive_timeouts = 3
        self._pause_duration = 2.0
        
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
        self.declare_parameter('loop_hz', 5)
        
        # 模型
        self.declare_parameter('yolo_model_path', '')
        self.declare_parameter('use_paddle_ocr', False)
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
        self.loop_hz = self.get_parameter('loop_hz').value
        self.yolo_model_path = self.get_parameter('yolo_model_path').value
        self.use_paddle_ocr = self.get_parameter('use_paddle_ocr').value
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

        # 性能统计
        self._total_loop_time = 0.0
        self._loop_count = 0
        self._debug_image_interval = 5

    def _init_rtsp_stream(self):
        """初始化 RTSP 视频流"""
        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开 RTSP 流: {self.rtsp_url}")
            self.cap = None
        else:
            # 设置低延迟
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.get_logger().info(f"RTSP 流已打开: {self.rtsp_url}")
    
    def _load_calib_from_file(self):
        """从标定文件加载相机内参"""
        if not self.calib_file:
            return

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
            self.get_logger().warn(f"标定文件加载失败，将使用默认内参: {e}")

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
            min_area_ratio=self.min_bbox_area_ratio,
            use_gpu=self.use_gpu,
            keypoint_method=self.keypoint_method,
            logger=self.get_logger(),
        )
        
        self.get_logger().info(f"检测器 GPU: {self.use_gpu}, 关键点方式: {self.keypoint_method}")
        self.tracker = DeepSORTTracker()
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
            distance_tolerance=self.distance_tolerance,
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
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_ros_image = cv_image
        except Exception as e:
            self.get_logger().error(f"ROS 图像转换失败: {e}")

    def _get_image(self):
        """获取图像 - 优先使用 RTSP 流"""
        # 优先使用 RTSP 流
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                return frame
            else:
                # 重连
                self.get_logger().warn("RTSP 流断开，尝试重连...")
                self.cap = cv2.VideoCapture(self.rtsp_url)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    return frame
        
        # 回退到 ROS 话题
        if self.camera_image_topic and self.latest_ros_image is not None:
            return self.latest_ros_image.copy()
        
        # 可选：使用 Go2 SDK 取流
        if self.camera_handler and self.camera_handler.is_available():
            frame = self.camera_handler.get_image_frame()
            if frame is not None:
                return frame
        
        return None

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

    def _publish_debug_image(self, cv_image):
        """发布调试图像"""
        debug_image = cv_image.copy()
        
        # 绘制检测框
        for x1, y1, x2, y2, conf, cls in self.current_detections:
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug_image, f"{conf:.2f}", (x1, y1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 绘制追踪框
        for track_id, bbox, conf in self.current_tracks:
            x1, y1, x2, y2 = bbox
            color = (0, 0, 255) if track_id == self.target_track_id else (255, 0, 0)
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(debug_image, f"ID:{track_id}", (x1, y1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # 绘制角点
        if self.target_track_id is not None:
            for track_id, bbox, conf in self.current_tracks:
                if track_id == self.target_track_id:
                    x1, y1, x2, y2 = bbox
                    corners = [(x1, y2), (x2, y2), (x2, y1), (x1, y1)]
                    for idx, (cx, cy) in enumerate(corners):
                        cv2.circle(debug_image, (int(cx), int(cy)), 5, (0, 255, 255), -1)
                        cv2.putText(debug_image, str(idx), (int(cx)+3, int(cy)-3),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    break
        
        # 状态信息
        status_text = f"Tracks: {len(self.current_tracks)}"
        if self.target_track_id:
            status_text += f" | Target: {self.target_track_id}"
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"调试图像发布失败: {e}")

    def _execute_pipeline(self):
        """执行完整检测→追踪→PnP→控制流程"""
        import time
        pipeline_start = time.time()
        
        # 1. 取图
        cv_image = self._get_image()
        if cv_image is None:
            self.get_logger().warn_throttle(2.0, "[Pipeline] 等待图像...")
            return
        
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
        
        # 无目标时停止
        if len(self.current_tracks) == 0:
            self.get_logger().warn_throttle(2.0, "[Pipeline] 无目标，停止运动")
            self.controller.stop()
            return
        
        # 发布调试图像
        self._frame_count += 1
        if self._frame_count % self._debug_image_interval == 0:
            self._publish_debug_image(cv_image)
        
        # 4. PnP位姿解算 + 5. 控制
        if self.target_track_id is not None:
            target_track = None
            for track_id, bbox, conf in self.current_tracks:
                if track_id == self.target_track_id:
                    target_track = (track_id, bbox, conf)
                    break
            
            if target_track is None:
                self.get_logger().warn("[Pipeline] 丢失目标")
                self.target_track_id = None
                self.controller.stop()
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
                    self.controller.stop()
                    return
            
            # PnP
            keypoints = self.detector.extract_corners(cv_image, bbox)
            pose = self.pose_solver.solve(bbox, self.image_shape, keypoints=keypoints)
            self.get_logger().info("[Pipeline] === 步骤4: PnP位姿解算完成 ===")
            
            # 控制
            ratio = self.pose_solver.get_target_bbox_ratio(bbox, self.image_shape)
            offset = self.pose_solver.get_center_offset(bbox, self.image_shape)
            cmd = self.controller.compute_control(pose, ratio, offset)
            
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
        else:
            self.get_logger().info_throttle(2.0, "[Pipeline] 无目标，保持静止")
            self.controller.stop()
        
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
