#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pose Adapter 主节点 - 电表巡检核心逻辑
"""

import rospy
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
        
        # 参数
        self._load_params()

        # 优先从标定文件加载相机内参（如果提供）
        self._load_calib_from_file()
        
        # CV 桥接
        self.bridge = CvBridge()
        
        # 相机参数
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_shape = None
        
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
        self.target_ratio = rospy.get_param('~target_ratio', 0.65)

        # 低功耗：主循环频率 Hz（默认 5，低算力设备建议 3-5，高算力可 10-20）
        self.loop_hz = rospy.get_param('~loop_hz', 5)
        
        # 模型路径
        self.yolo_model_path = rospy.get_param('~yolo_model_path', None)
        self.use_paddle_ocr = rospy.get_param('~use_paddle_ocr', False)
        
        # 标定文件
        self.calib_file = rospy.get_param('~calib_file', None)

        # Go2 网络接口（用于 Unitree SDK）
        self.network_interface = rospy.get_param('~network_interface', '')

        # 是否使用 Go2 相机 SDK 取流（默认 False，从 camera_image_topic 取图）
        self.use_go2_camera = rospy.get_param('~use_go2_camera', False)

        # 图像话题（与 calibrate 一致默认 /camera/image_raw；由其他节点发布相机 raw）
        self.camera_image_topic = rospy.get_param('~camera_image_topic', '/camera/image_raw')

        # 本地图片保存路径（拍照/调试）
        self.image_save_path = rospy.get_param('~image_save_path', '/tmp/pose_adapter_images')

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

        # 定时器（低算力设备降低 loop_hz 以节省 CPU）
        period = 1.0 / max(1, min(30, self.loop_hz))
        self.control_timer = rospy.Timer(rospy.Duration(period), self._control_loop)
        self.image_timer = rospy.Timer(rospy.Duration(period), self._image_loop)
        rospy.loginfo(f"主循环频率: {self.loop_hz} Hz")

    def _init_modules(self):
        """初始化各模块（需要相机参数）"""
        if self.camera_matrix is None:
            # 使用默认参数
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
        
        # 初始化模块（OCR 延迟至首次触发时加载，避免 PaddleOCR 导致段错误）
        self.detector = MeterDetector(model_path=self.yolo_model_path)
        self.tracker = DeepSORTTracker()
        self.pose_solver = PoseSolver(
            self.camera_matrix,
            self.dist_coeffs,
            (self.meter_width, self.meter_height)
        )
        self.controller = MotionController(target_distance=self.target_distance)
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
        self.current_detections = detections
        
        # 2. 追踪
        if detections:
            tracks = self.tracker.update(detections)
            self.current_tracks = tracks
            
            # 选择目标（最近/最居中）
            if self.target_track_id is None and tracks:
                self.target_track_id = self._select_target(tracks)
        else:
            self.current_tracks = []
        
        # 发布调试图像（每 2 帧发布一次以降低 CPU）
        self._frame_count += 1
        if self._frame_count % 2 == 0:
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
            self.controller.stop()
            return
        
        _, bbox, conf = target_track
        
        # 位姿解算
        pose = self.pose_solver.solve(bbox, self.image_shape)
        
        # 计算画面占比和中心偏差
        ratio = self.pose_solver.get_target_bbox_ratio(bbox, self.image_shape)
        offset = self.pose_solver.get_center_offset(bbox, self.image_shape)
        
        # 计算控制指令
        cmd = self.controller.compute_control(pose, ratio, offset)
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
        """运行节点"""
        rospy.spin()
    
    def shutdown(self):
        """关闭节点"""
        self.is_running = False
        if self.controller:
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
