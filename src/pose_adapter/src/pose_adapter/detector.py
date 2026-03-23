#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
电表检测器 - 基于 YOLOv8 的目标检测

支持三种关键点提取方式:
- bbox: 使用检测框的四个角点（现有方式）
- contour: 使用轮廓检测 + 角点提取（新增）
- keypoint: 使用关键点检测模型（预留接口）
"""

import cv2
import numpy as np
import rospy
import time


class MeterDetector:
    """
    电表检测器类
    
    使用 YOLOv8 模型检测电表，输出 bbox 和关键点（可选）
    """
    
    def __init__(self, model_path=None, conf_threshold=0.5, iou_threshold=0.45, min_area_ratio=0.05, use_gpu=True, keypoint_method='bbox'):
        """
        初始化检测器
        
        Args:
            model_path: YOLOv8 模型路径，None 时使用默认参数
            conf_threshold: 置信度阈值
            iou_threshold: NMS IoU 阈值
            min_area_ratio: 备用检测时允许的最小 bbox 面积占比（0-1）
            use_gpu: 是否使用 GPU 加速（默认 True）
            keypoint_method: 关键点提取方式: 'bbox'(默认) / 'contour' / 'keypoint'
        """
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        # 备用检测时使用的最小面积占比（例如 0.05 表示 5%）
        self.min_area_ratio = float(min_area_ratio) if min_area_ratio is not None else 0.05
        self.use_gpu = use_gpu
        self.keypoint_method = keypoint_method
        
        rospy.loginfo(f"[Detector] 关键点提取方式: {keypoint_method}")
        
        # 耗时统计
        self._total_time = 0.0
        self._count = 0
        
        # 尝试加载 YOLOv8
        self.model = None
        self.use_yolo = False
        self._device = 'cpu'
        
        if model_path and str(model_path).strip():
            try:
                from ultralytics import YOLO
                self.model = YOLO(str(model_path).strip())
                
                # GPU 加速配置
                if self.use_gpu:
                    # 尝试使用 CUDA
                    try:
                        import torch
                        if torch.cuda.is_available():
                            self._device = 'cuda:0'
                            rospy.loginfo("检测到 CUDA GPU，使用 GPU 加速")
                        else:
                            rospy.logwarn("未检测到 CUDA GPU，使用 CPU")
                    except ImportError:
                        rospy.logwarn("PyTorch 未安装，使用 CPU")
                
                # 不转 FP16，避免与 ultralytics fuse_conv_and_bn 冲突（expected scalar type Half but found Float）
                self.use_yolo = True
                rospy.loginfo(f"YOLOv8 模型加载成功: {model_path}, 设备: {self._device}")
            except ImportError as e:
                rospy.logwarn("ultralytics 导入失败（将使用备用检测）: %s" % e)
            except Exception as e:
                rospy.logwarn(f"YOLO 模型加载失败: {e}")
        
        # 备用：使用 OpenCV DNN 或传统方法
        if not self.use_yolo:
            rospy.loginfo("使用 OpenCV DNN 备用检测方案")
            self._init_backup_detector()
    
    def _init_backup_detector(self):
        """初始化备用检测器（基于颜色/形状的简单检测）"""
        # 电表通常是矩形，白色/灰色背景，黑色数字
        # 这里使用简单的轮廓检测
        self.backup_detector = True
    
    def detect(self, cv_image):
        """
        检测电表
        
        Args:
            cv_image: OpenCV BGR 图像
            
        Returns:
            list: [(x1, y1, x2, y2, conf, class_id), ...]
        """
        if cv_image is None:
            return []

        start_time = time.time()
        
        if self.use_yolo:
            detections = self._detect_yolo(cv_image)
        else:
            detections = self._detect_backup(cv_image)
        
        # 耗时统计
        elapsed = (time.time() - start_time) * 1000  # ms
        self._total_time += elapsed
        self._count += 1
        avg_time = self._total_time / self._count if self._count > 0 else 0
        
        # 打印耗时日志（每帧或节流）
        rospy.loginfo_throttle(
            1.0,
            f"[Detector] 检测耗时: {elapsed:.1f}ms, 平均: {avg_time:.1f}ms, "
            f"设备: {self._device}, 检测数: {len(detections)}"
        )
        
        return detections
    
    def _detect_yolo(self, cv_image):
        """使用 YOLOv8 检测"""
        # 使用指定设备（GPU/CPU），强制 FP32 避免 Jetson 上 Half/Float 冲突
        results = self.model(cv_image, verbose=False, device=self._device, half=False)
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()
                cls = int(box.cls[0].cpu().numpy())
                
                if conf >= self.conf_threshold:
                    detections.append((int(x1), int(y1), int(x2), int(y2), float(conf), cls))
        
        return detections
    
    def _detect_backup(self, cv_image):
        """
        备用检测方案 - 基于轮廓的矩形检测
        用于测试阶段，实际部署应使用 YOLOv8
        大图先降采样至 640 宽以节省 CPU
        """
        detections = []
        h, w = cv_image.shape[:2]
        scale = 1.0
        work_img = cv_image

        # 低算力优化：宽>640 时降采样
        if w > 640:
            scale = 640.0 / w
            nw, nh = 640, int(h * scale)
            work_img = cv2.resize(cv_image, (nw, nh), interpolation=cv2.INTER_LINEAR)
            h, w = nh, nw

        # 转为灰度
        gray = cv2.cvtColor(work_img, cv2.COLOR_BGR2GRAY)
        
        # 自适应阈值
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                       cv2.THRESH_BINARY_INV, 11, 2)
        
        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 电表在画面中的面积：由 min_area_ratio 控制（默认 5%），上限 80%
        min_area = (w * h) * max(0.0, min(self.min_area_ratio, 0.5))  # 至少占画面 min_area_ratio，防止设置过大
        max_area = (w * h) * 0.8   # 最多占画面 80%
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (min_area < area < max_area):
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect_ratio = bw / float(bh)
            
            # 电表通常是竖直矩形，宽高比在 0.5-2 之间
            if 0.5 < aspect_ratio < 2.0:
                conf = min(0.9, area / (w * h * 0.5))  # 简单的置信度估计
                # 若降采样过，坐标需缩放回原图
                if scale < 1.0:
                    inv = 1.0 / scale
                    x, y, bw, bh = int(x*inv), int(y*inv), int(bw*inv), int(bh*inv)
                detections.append((x, y, x+bw, y+bh, conf, 0))
        
        # 按置信度排序
        detections.sort(key=lambda x: x[4], reverse=True)
        return detections[:3]  # 最多返回 3 个
    
    def get_roi(self, cv_image, bbox):
        """
        从图像中提取 ROI
        
        Args:
            cv_image: OpenCV 图像
            bbox: (x1, y1, x2, y2)
            
        Returns:
            ROI 图像
        """
        x1, y1, x2, y2 = bbox
        # 添加一点边距
        h, w = cv_image.shape[:2]
        margin = 10
        x1 = max(0, x1 - margin)
        y1 = max(0, y1 - margin)
        x2 = min(w, x2 + margin)
        y2 = min(h, y2 + margin)
        return cv_image[y1:y2, x1:x2]
    
    def extract_corners(self, cv_image, bbox):
        """
        提取电表角点
        
        根据 keypoint_method 配置选择提取方式:
        - bbox: 使用检测框的四个角点
        - contour: 使用轮廓检测 + 角点提取
        - keypoint: 使用关键点检测模型（预留）
        
        Args:
            cv_image: OpenCV BGR 图像
            bbox: (x1, y1, x2, y2) 检测框
            
        Returns:
            list: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]] 四个角点坐标（顺时针，从左上开始）
                  或 None（提取失败时返回 bbox 角点）
        """
        if self.keypoint_method == 'bbox':
            return self._get_bbox_corners(bbox)
        elif self.keypoint_method == 'contour':
            return self._extract_contour_corners(cv_image, bbox)
        elif self.keypoint_method == 'keypoint':
            # 预留接口：关键点检测模型
            rospy.logwarn("[Detector] keypoint 方法暂未实现，回退到 bbox")
            return self._get_bbox_corners(bbox)
        else:
            rospy.logwarn(f"[Detector] 未知 keypoint_method: {self.keypoint_method}，使用 bbox")
            return self._get_bbox_corners(bbox)
    
    def _get_bbox_corners(self, bbox):
        """
        使用 bbox 四个角点
        
        Args:
            bbox: (x1, y1, x2, y2)
            
        Returns:
            [[x,y], ...] 四个角点（顺时针）
        """
        x1, y1, x2, y2 = bbox
        # 顺时针: 左上 -> 右上 -> 右下 -> 左下
        return [[int(x1), int(y1)], [int(x2), int(y1)], [int(x2), int(y2)], [int(x1), int(y2)]]
    
    def _extract_contour_corners(self, cv_image, bbox):
        """
        使用轮廓检测 + 角点提取
        
        在 bbox 区域内进行轮廓检测，提取最可能是电表轮廓的四个角点
        
        Args:
            cv_image: OpenCV BGR 图像
            bbox: (x1, y1, x2, y2) 检测框
            
        Returns:
            [[x,y], ...] 四个角点（顺时针）或 None
        """
        x1, y1, x2, y2 = bbox
        
        # 扩展 bbox 区域
        h, w = cv_image.shape[:2]
        margin = 5
        x1 = max(0, x1 - margin)
        y1 = max(0, y1 - margin)
        x2 = min(w, x2 + margin)
        y2 = min(h, y2 + margin)
        
        # 裁剪 ROI
        roi = cv_image[y1:y2, x1:x2]
        if roi.size == 0:
            rospy.logwarn("[Detector] ROI 为空，回退到 bbox")
            return self._get_bbox_corners(bbox)
        
        # 转灰度
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # 预处理：模糊 + 边缘检测
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        # 形态学操作：闭合边缘
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # 找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            rospy.logwarn("[Detector] 未找到轮廓，回退到 bbox")
            return self._get_bbox_corners(bbox)
        
        # 找最大轮廓
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        
        # 面积太小的不要
        roi_area = roi.shape[0] * roi.shape[1]
        if area < roi_area * 0.1:
            rospy.logwarn(f"[Detector] 轮廓面积太小 ({area/roi_area*100:.1f}%)，回退到 bbox")
            return self._get_bbox_corners(bbox)
        
        # 近似多边形，尝试拟合4个角点
        epsilon = 0.02 * cv2.arcLength(max_contour, True)
        approx = cv2.approxPolyDP(max_contour, epsilon, True)
        
        # 如果拟合结果不是4点，尝试调整 epsilon 或直接用凸包
        if len(approx) < 4:
            # 尝试更宽松的 epsilon
            epsilon = 0.04 * cv2.arcLength(max_contour, True)
            approx = cv2.approxPolyDP(max_contour, epsilon, True)
        
        if len(approx) < 4:
            # 用凸包
            hull = cv2.convexHull(max_contour)
            hull_approx = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
            if len(hull_approx) >= 4:
                approx = hull_approx[:4]
            else:
                rospy.logwarn("[Detector] 无法提取4个角点，回退到 bbox")
                return self._get_bbox_corners(bbox)
        
        # 如果点数 > 4，取前4个（按面积排序后的凸包点）
        if len(approx) > 4:
            # 计算凸包并按顺序取4个
            hull = cv2.convexHull(max_contour)
            hull_approx = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
            approx = hull_approx[:4]
        
        # 提取角点坐标并排序（顺时针）
        corners = []
        for pt in approx:
            if len(pt[0]) >= 2:
                corners.append([pt[0][0] + x1, pt[0][1] + y1])
        
        if len(corners) < 4:
            return self._get_bbox_corners(bbox)
        
        # 排序：先按 x+y 和排序得到左上、右下，再用叉积得到其他两点
        corners = self._sort_corners(corners)
        
        rospy.logdebug(f"[Detector] 轮廓角点: {corners}")
        return corners
    
    def _sort_corners(self, corners):
        """
        将4个角点排序为顺时针顺序（左上、右上、右下、左下）
        
        Args:
            corners: [[x,y], ...] 任意顺序的4个点
            
        Returns:
            [[x,y], ...] 顺时针顺序
        """
        if len(corners) != 4:
            return corners
        
        # 按 x+y 和排序，得到左上(最小)和右下(最大)
        corners = sorted(corners, key=lambda p: p[0] + p[1])
        top_left = corners[0]
        bottom_right = corners[3]
        
        # 剩下的两点：按 x-y 排序
        remaining = [corners[1], corners[2]]
        remaining = sorted(remaining, key=lambda p: p[0] - p[1])
        top_right = remaining[0]
        bottom_left = remaining[1]
        
        # 验证：左上、右上、右下、左下
        return [top_left, top_right, bottom_right, bottom_left]