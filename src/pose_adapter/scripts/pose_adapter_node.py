#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pose Adapter 主节点 - 电表巡检核心逻辑
"""

import sys
import rospy
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

from pose_adapter.detector import MeterDetector
from pose_adapter.tracker import DeepSORTTracker
from pose_adapter.pose_solver import PoseSolver
from pose_adapter.controller import MotionController
from pose_adapter.ocr import MeterOCR
from pose_adapter.camera import CameraHandler


class PoseAdapterNode:
    """
    Pose Adapter 主节点
    
    整合检测、追踪、位姿解算、控制、OCR 的完整流程
    """
    
    def __init__(self):
        """初始化节点"""
        rospy.init_node('pose_adapter', anonymous=True)
        rospy.loginfo("节点运行 Python: %s" % sys.executable)
        
        # 参数
        self._load_params()

        # 相机内参：先置空，再由标定文件填充（若提供）
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
        self.latest_ros_image = None  # ROS 话题回退用的最新帧

        # 状态
        self.current_detections = []
        self.current_tracks = []
        self.target_track_id = None
        self.is_running = True
        self.ocr_result = None
        self._frame_count = 0  # 用于节流调试图像
        
        # 防死循环计数器
        self._consecutive_timeouts = 0  # 连续超时次数
        self._max_consecutive_timeouts = 3  # 最大连续超时次数，超过后暂停
        self._pause_duration = 2.0  # 超时时暂停秒数
        
        # 使用 Unitree SDK2 VideoClient 初始化 Go2 前置相机
        if self.use_go2_camera:
            self._init_camera()
        else:
            rospy.loginfo("已禁用 Go2 相机，仅使用 camera_image_topic")

        # ROS
        self._init_ros()
        
        rospy.loginfo("Pose Adapter 节点初始化完成")
    
    def _load_params(self):
        """加载参数"""
        # 相机参数
        self.camera_width = rospy.get_param('~camera_width', 1280)
        self.camera_height = rospy.get_param('~camera_height', 720)
        
        # 电表尺寸（米）
        self.meter_width = rospy.get_param('~meter_width', 0.2)
        self.meter_height = rospy.get_param('~meter_height', 0.3)
        
        # 控制参数
        self.target_distance = rospy.get_param('~target_distance', 1.7)
        # 距离容差（米）：|distance - target_distance| 小于该值时认为距离已 OK，不再前后挪
        self.distance_tolerance = rospy.get_param('~distance_tolerance', 0.05)
        self.target_ratio = rospy.get_param('~target_ratio', 0.65)

        # 低功耗：主循环频率 Hz（默认 5，低算力设备建议 3-5，高算力可 10-20）
        self.loop_hz = rospy.get_param('~loop_hz', 5)
        
        # 模型路径与检测参数
        self.yolo_model_path = rospy.get_param('~yolo_model_path', None)
        self.use_paddle_ocr = rospy.get_param('~use_paddle_ocr', False)
        # 备用检测最小 bbox 面积占比（0-1），默认 0.05（5%）
        self.min_bbox_area_ratio = rospy.get_param('~min_bbox_area_ratio', 0.05)
        # 关键点提取方式: bbox(默认) / contour / keypoint
        self.keypoint_method = rospy.get_param('~keypoint_method', 'bbox')
        
        # GPU 加速（默认 True，使用 NVIDIA Jetson 的 GPU）
        self.use_gpu = rospy.get_param('~use_gpu', True)
        
        # 标定文件
        self.calib_file = rospy.get_param('~calib_file', None)

        # Go2 网络接口（用于 Unitree SDK）
        self.network_interface = rospy.get_param('~network_interface', '')

        # 是否使用 Go2 相机 SDK 取流（默认 False，从 camera_image_topic 取图）
        self.use_go2_camera = rospy.get_param('~use_go2_camera', False)

        # 机器狗类型: "GO2" (Unitree), "ZSI-1" (zsibot_sdk)，默认从环境变量 BODY 读取
        self.body_type = rospy.get_param('~body_type', None)

        # 是否使用 Unitree SDK high_level 接口控制（默认 True，优先使用 SDK）
        self.use_high_level_sdk = rospy.get_param('~use_high_level_sdk', True)
        # 运动前提：关闭避障。SDK 支持时是否在启动时尝试关闭避障
        self.disable_obstacle_avoidance_on_start = rospy.get_param('~disable_obstacle_avoidance_on_start', True)
        # 步态：use_classic_walk=false 时默认步态易走步，true 时稀碎步(部分固件下可能不迈步)
        self.use_classic_walk = rospy.get_param('~use_classic_walk', False)
        self.speed_level = rospy.get_param('~speed_level', 0)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.12)
        self.min_linear_speed = rospy.get_param('~min_linear_speed', 0.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.25)
        # 步长参数
        self.step_distance = rospy.get_param('~step_distance', 0.2)  # 米
        self.step_angle = rospy.get_param('~step_angle', 5.0)       # 度

        # 图像话题（与 calibrate 一致默认 /camera/image_raw；由其他节点发布相机 raw）
        self.camera_image_topic = rospy.get_param('~camera_image_topic', '/camera/image_raw')

        # 本地图片保存路径（拍照/调试）
        self.image_save_path = rospy.get_param('~image_save_path', '/tmp/pose_adapter_images')
        
        # 性能统计
        self._total_loop_time = 0.0
        self._loop_count = 0
        
        # 内存优化：减少调试图像发布频率
        self._debug_image_interval = 5  # 每5帧发布一次调试图像（减少内存占用）

    def _load_calib_from_file(self):
        """从标定文件加载相机内参（如果提供了 calib_file）"""
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

            rospy.loginfo(f"从标定文件加载相机内参: {self.calib_file}")
        except Exception as e:
            rospy.logwarn(f"标定文件加载失败，将使用默认内参: {e}")

    def _init_camera(self):
        """使用 Unitree SDK 初始化 Go2 前置相机"""
        config = {
            'camera': {
                'image_save_path': self.image_save_path
            },
            'robot': {
                'network_interface': self.network_interface
            }
        }

        self.camera_handler = CameraHandler(config)
        if not self.camera_handler.is_available():
            rospy.logwarn("Go2 相机不可用，无法从 Unitree SDK 获取图像")
    
    def _init_ros(self):
        """初始化 ROS"""
        # 发布
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/pose_adapter/debug_image', Image, queue_size=1)

        # 订阅图像话题：默认从 /camera/image_raw 取帧（与 calibrate 一致，需其他节点发布）
        if self.camera_image_topic:
            self.image_sub = rospy.Subscriber(
                self.camera_image_topic, Image, self._ros_image_callback, queue_size=1
            )
            rospy.loginfo(f"图像来源: {self.camera_image_topic}")
        else:
            self.image_sub = None

        # 定时器已移除，改用同步顺序执行
        # 取图 → YOLO检测 → 追踪 → PnP → 控制(等待执行完成) → 循环
        rospy.loginfo("使用同步顺序执行：取图 → 检测 → 追踪 → PnP → 控制(等待完成) → 循环")

    def _init_modules(self):
        """初始化各模块（需要相机参数）"""
        if self.camera_matrix is None:
            # 未从标定文件加载到时使用默认参数
            rospy.logwarn("使用默认相机参数，建议进行标定")
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
        else:
            rospy.loginfo("使用标定文件内参进行 PnP 解算")
        
        # 初始化模块（OCR 延迟至首次触发时加载，避免 PaddleOCR 导致段错误）
        self.detector = MeterDetector(
            model_path=self.yolo_model_path,
            min_area_ratio=self.min_bbox_area_ratio,
            use_gpu=self.use_gpu,
            keypoint_method=self.keypoint_method,
        )
        
        rospy.loginfo(f"检测器 GPU 加速: {self.use_gpu}, 关键点方式: {self.keypoint_method}")
        self.tracker = DeepSORTTracker()
        self.pose_solver = PoseSolver(
            self.camera_matrix,
            self.dist_coeffs,
            (self.meter_width, self.meter_height)
        )
        self.controller = MotionController(
            target_distance=self.target_distance,
            distance_tolerance=self.distance_tolerance,
            max_linear_speed=self.max_linear_speed,
            min_linear_speed=self.min_linear_speed,
            max_angular_speed=self.max_angular_speed,
            step_distance=self.step_distance,
            step_angle=self.step_angle,
            use_high_level_sdk=self.use_high_level_sdk,
            interface_name=self.network_interface if self.network_interface else "eth1",
            disable_obstacle_avoidance_on_start=self.disable_obstacle_avoidance_on_start,
            use_classic_walk=self.use_classic_walk,
            speed_level=self.speed_level,
            body_type=self.body_type,
        )
        self.ocr = None  # 延迟初始化
        
        rospy.loginfo("所有模块初始化完成")

    def _ros_image_callback(self, msg):
        """ROS 图像话题回调，保存最新帧供 _image_loop 使用"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_ros_image = cv_image
        except Exception as e:
            rospy.logerr_throttle(5, f"ROS 图像转换失败: {e}")

    def _image_loop(self, event):
        """图像循环：从 ROS 话题 /camera/image_raw 或 Go2 SDK 获取图像并进行检测/追踪"""
        loop_start = time.time()
        
        cv_image = None

        # 优先从话题取帧（默认，与 calibrate 一致）
        if self.camera_image_topic and self.latest_ros_image is not None:
            cv_image = self.latest_ros_image.copy()

        # 可选：使用 Go2 SDK 取流（use_go2_camera=true 时）
        if cv_image is None and self.camera_handler and self.camera_handler.is_available():
            frame = self.camera_handler.get_image_frame()
            if frame is not None:
                cv_image = frame

        if cv_image is None:
            return

        self.image_shape = cv_image.shape[:2]
        
        # 延迟初始化
        if self.detector is None:
            self._init_modules()
        
        # 1. 检测
        detections = self.detector.detect(cv_image)

        # 过滤异常尺寸的检测框（例如几乎占满整幅图像的 bbox）
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
                rospy.logwarn_throttle(
                    2.0,
                    f"检测框过大(占比 {area_ratio:.2f})，丢弃该结果: "
                    f"bbox=({x1}, {y1}, {x2}, {y2})"
                )
                continue
            valid_detections.append(det)

        self.current_detections = valid_detections
        
        # 2. 追踪
        if valid_detections:
            tracks = self.tracker.update(valid_detections)
            self.current_tracks = tracks
            
            # 选择目标（最近/最居中）
            if self.target_track_id is None and tracks:
                self.target_track_id = self._select_target(tracks)
        else:
            self.current_tracks = []
        
        # 整体耗时统计
        loop_elapsed = (time.time() - loop_start) * 1000  # ms
        self._total_loop_time += loop_elapsed
        self._loop_count += 1
        avg_loop_time = self._total_loop_time / self._loop_count if self._loop_count > 0 else 0
        
        rospy.loginfo_throttle(
            2.0,
            f"[Pipeline] 总耗时: {loop_elapsed:.1f}ms, 平均: {avg_loop_time:.1f}ms, "
            f"FPS: {1000.0/avg_loop_time:.1f}"
        )
        
        # 发布调试图像（每N帧发布一次以降低CPU和内存）
        self._frame_count += 1
        if self._frame_count % self._debug_image_interval == 0:
            self._publish_debug_image(cv_image)
    
    def _select_target(self, tracks):
        """选择追踪目标"""
        if not tracks:
            return None
        
        # 选择最居中的目标
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
        
        rospy.loginfo(f"选择目标 ID: {best_track}")
        return best_track
    
    def _control_loop(self, event):
        """控制循环"""
        if not self.is_running or self.target_track_id is None:
            return
        
        # 查找目标追踪
        target_track = None
        for track_id, bbox, conf in self.current_tracks:
            if track_id == self.target_track_id:
                target_track = (track_id, bbox, conf)
                break
        
        if target_track is None:
            rospy.logwarn_throttle(5, "丢失目标，尝试重新检测...")
            self.target_track_id = None
            # 不调用 controller.stop()，避免频繁 StopMove 导致原地抖；仅清空目标，下一帧重新选目标后再发 Move
            return
        
        _, bbox, conf = target_track

        # 若当前 bbox 明显异常（几乎占满整幅图像），则跳过控制，避免 PnP 得到不合理位姿
        h, w = self.image_shape if self.image_shape is not None else (0, 0)
        if h > 0 and w > 0:
            x1, y1, x2, y2 = bbox
            bw = max(0, x2 - x1)
            bh = max(0, y2 - y1)
            if bw > 0 and bh > 0:
                area_ratio = float(bw * bh) / float(w * h)
                if area_ratio >= 0.9:
                    rospy.logwarn_throttle(
                        2.0,
                        f"目标框占比过大(占比 {area_ratio:.2f})，跳过本次控制: "
                        f"bbox=({x1}, {y1}, {x2}, {y2})"
                    )
                    self.controller.stop()
                    return
        
        # 位姿解算
        # 提取角点（根据配置使用 bbox 或 contour）
        keypoints = self.detector.extract_corners(cv_image, bbox)
        pose = self.pose_solver.solve(bbox, self.image_shape, keypoints=keypoints)
        
        # 计算画面占比和中心偏差
        ratio = self.pose_solver.get_target_bbox_ratio(bbox, self.image_shape)
        offset = self.pose_solver.get_center_offset(bbox, self.image_shape)
        
        # 计算控制指令
        cmd = self.controller.compute_control(pose, ratio, offset)
        
        # SDK 模式下指令已在内部通过 Move 发送，无需发布到 cmd_vel 话题
        if cmd is not None:
            self.cmd_vel_pub.publish(cmd)
        
        # 检查是否到位
        if self.controller.is_ready_for_ocr():
            self._trigger_ocr(bbox)
    
    def _trigger_ocr(self, bbox):
        """触发 OCR"""
        if self.ocr_result is not None:
            return

        if self.ocr is None:
            self.ocr = MeterOCR(use_paddle=self.use_paddle_ocr)
        rospy.loginfo("触发 OCR 识别...")
    
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
        
        # 若已选中目标，在 debug 图中高亮 PnP 使用的四个角点，便于核对电表读数区域
        if self.target_track_id is not None:
            for track_id, bbox, conf in self.current_tracks:
                if track_id == self.target_track_id:
                    x1, y1, x2, y2 = bbox
                    # 顺序与 PoseSolver 中的 image_points 保持一致：
                    # 左下、右下、右上、左上
                    corners = [
                        (x1, y2),
                        (x2, y2),
                        (x2, y1),
                        (x1, y1),
                    ]
                    for idx, (cx, cy) in enumerate(corners):
                        cv2.circle(debug_image, (int(cx), int(cy)), 5, (0, 255, 255), -1)
                        cv2.putText(
                            debug_image,
                            str(idx),
                            (int(cx) + 3, int(cy) - 3),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 255),
                            1,
                        )
                    break
        
        # 绘制状态信息
        status_text = f"Tracks: {len(self.current_tracks)}"
        if self.target_track_id:
            status_text += f" | Target: {self.target_track_id}"
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"调试图像发布失败: {e}")
    
    def run(self):
        """同步顺序执行主循环"""
        rospy.loginfo("启动同步主循环：脚踏实地一步一步执行")
        
        while self.is_running and not rospy.is_shutdown():
            try:
                # 执行一次完整的检测→追踪→PnP→控制流程
                self._execute_pipeline()
                
                # 检查是否需要退出
                if not self.is_running:
                    break
                    
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"主循环异常: {e}")
                rospy.sleep(0.5)
        
        rospy.loginfo("主循环已退出")
    
    def _execute_pipeline(self):
        """
        同步顺序执行完整流程：
        1. 取图
        2. YOLO检测
        3. 追踪
        4. PnP位姿解算
        5. 控制(等待执行完成)
        """
        pipeline_start = time.time()
        
        # ========== 1. 取图 ==========
        cv_image = self._get_image()
        if cv_image is None:
            rospy.logwarn_throttle(2.0, "[Pipeline] 等待图像...")
            rospy.sleep(0.1)
            return
        
        self.image_shape = cv_image.shape[:2]
        
        # 延迟初始化模块
        if self.detector is None:
            self._init_modules()
        
        rospy.loginfo("[Pipeline] === 步骤1: 取图完成 ===")
        
        # ========== 2. YOLO检测 ==========
        detections = self.detector.detect(cv_image)
        rospy.loginfo(f"[Pipeline] === 步骤2: YOLO检测完成，检测到 {len(detections)} 个目标 ===")
        
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
                rospy.logwarn(f"[Pipeline] 检测框过大({area_ratio:.2f})，丢弃: bbox=({x1}, {y1}, {x2}, {y2})")
                continue
            valid_detections.append(det)
        
        self.current_detections = valid_detections
        
        # ========== 3. 追踪 ==========
        if valid_detections:
            tracks = self.tracker.update(valid_detections)
            self.current_tracks = tracks
            
            # 选择目标
            if self.target_track_id is None and tracks:
                self.target_track_id = self._select_target(tracks)
        else:
            self.current_tracks = []
        
        rospy.loginfo(f"[Pipeline] === 步骤3: 追踪完成，{len(self.current_tracks)} 个追踪 ===")
        
        # 无目标时只停止，不下发控制
        if len(self.current_tracks) == 0:
            rospy.logwarn_throttle(2.0, "[Pipeline] 无目标，停止运动")
            self.controller.stop()
            rospy.sleep(0.5)  # 暂停降低频率
            return
        
        # 发布调试图像
        self._publish_debug_image(cv_image)
        
        # ========== 4. PnP位姿解算 + 5. 控制 ==========
        if self.target_track_id is not None:
            # 查找目标追踪
            target_track = None
            for track_id, bbox, conf in self.current_tracks:
                if track_id == self.target_track_id:
                    target_track = (track_id, bbox, conf)
                    break
            
            if target_track is None:
                rospy.logwarn("[Pipeline] 丢失目标，停止运动")
                self.target_track_id = None
                self.controller.stop()
                rospy.sleep(0.5)
                return
            
            _, bbox, conf = target_track
            
            # 检查bbox是否异常
            x1, y1, x2, y2 = bbox
            bw = max(0, x2 - x1)
            bh = max(0, y2 - y1)
            if bw > 0 and bh > 0:
                area_ratio = float(bw * bh) / float(w * h)
                if area_ratio >= 0.9:
                    rospy.logwarn(f"[Pipeline] 目标框过大({area_ratio:.2f})，跳过控制")
                    self.controller.stop()
                    return
            
            # PnP位姿解算
            # 提取角点（根据配置使用 bbox 或 contour）
            keypoints = self.detector.extract_corners(cv_image, bbox)
            pose = self.pose_solver.solve(bbox, self.image_shape, keypoints=keypoints)
            rospy.loginfo("[Pipeline] === 步骤4: PnP位姿解算完成 ===")
            
            # 计算控制指令
            ratio = self.pose_solver.get_target_bbox_ratio(bbox, self.image_shape)
            offset = self.pose_solver.get_center_offset(bbox, self.image_shape)
            cmd = self.controller.compute_control(pose, ratio, offset)
            
            if cmd is not None:
                self.cmd_vel_pub.publish(cmd)
            
            rospy.loginfo("[Pipeline] === 步骤5: 控制指令已下发，等待执行完成 ===")
            
            # 等待运动执行完成（核心：脚踏实地）
            motion_complete = self.controller.wait_for_motion_complete()
            if motion_complete:
                rospy.loginfo("[Pipeline] 运动执行完成")
                self._consecutive_timeouts = 0  # 成功后重置计数器
            else:
                rospy.logwarn("[Pipeline] 运动执行超时")
                self._consecutive_timeouts += 1
                rospy.logwarn(f"[Pipeline] 连续超时次数: {self._consecutive_timeouts}/{self._max_consecutive_timeouts}")
                
                # 超过最大连续超时次数，暂停并重置
                if self._consecutive_timeouts >= self._max_consecutive_timeouts:
                    rospy.logerr(f"[Pipeline] 连续超时{self._max_consecutive_timeouts}次，暂停{self._pause_duration}秒并停止运动")
                    self.controller.stop()
                    rospy.sleep(self._pause_duration)
                    self._consecutive_timeouts = 0  # 重置计数器
            
            # 检查是否到位，准备OCR
            if self.controller.is_ready_for_ocr():
                self._trigger_ocr(bbox)
        else:
            rospy.loginfo_throttle(2.0, "[Pipeline] 无目标，保持静止")
            self.controller.stop()
        
        # 统计耗时
        pipeline_elapsed = (time.time() - pipeline_start) * 1000
        self._total_loop_time += pipeline_elapsed
        self._loop_count += 1
        avg_time = self._total_loop_time / self._loop_count if self._loop_count > 0 else 0
        
        rospy.loginfo(f"[Pipeline] 本轮耗时: {pipeline_elapsed:.1f}ms, 平均: {avg_time:.1f}ms")
    
    def _get_image(self):
        """获取图像"""
        # 优先从ROS话题取帧
        if self.camera_image_topic and self.latest_ros_image is not None:
            return self.latest_ros_image.copy()
        
        # 可选：使用Go2 SDK取流
        if self.camera_handler and self.camera_handler.is_available():
            frame = self.camera_handler.get_image_frame()
            if frame is not None:
                return frame
        
        return None
    
    def shutdown(self):
        """关闭节点"""
        self.is_running = False
        if self.controller:
            self.controller.is_running = False
            self.controller.stop()
        rospy.loginfo("Pose Adapter 节点已关闭")


def main():
    """主函数"""
    node = None
    try:
        node = PoseAdapterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node:
            node.shutdown()


if __name__ == '__main__':
    main()
