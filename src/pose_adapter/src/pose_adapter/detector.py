#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电表检测器 - 基于 YOLOv8 的目标检测 (ROS2 版本)
"""

import cv2
import numpy as np
import time


def _loginfo(msg, logger):
    if logger:
        logger.info(msg)
    else:
        print(f"[INFO] {msg}")

def _logwarn(msg, logger):
    if logger:
        logger.warn(msg)
    else:
        print(f"[WARN] {msg}")

def _logerr(msg, logger):
    if logger:
        logger.error(msg)
    else:
        print(f"[ERROR] {msg}")

def _logdebug(msg, logger):
    if logger:
        logger.debug(msg)


class MeterDetector:
    """电表检测器类"""
    
    def __init__(self, model_path=None, conf_threshold=0.5, iou_threshold=0.45, min_area_ratio=0.05, use_gpu=True, keypoint_method='bbox', logger=None):
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.min_area_ratio = float(min_area_ratio) if min_area_ratio is not None else 0.05
        self.use_gpu = use_gpu
        self.keypoint_method = keypoint_method
        self.logger = logger
        
        _loginfo(f"[Detector] 关键点提取方式: {keypoint_method}", logger)
        
        self._total_time = 0.0
        self._count = 0
        
        self.model = None
        self.use_yolo = False
        self._device = 'cpu'
        
        if model_path and str(model_path).strip():
            try:
                from ultralytics import YOLO
                self.model = YOLO(str(model_path).strip())
                
                if self.use_gpu:
                    try:
                        import torch
                        if torch.cuda.is_available():
                            self._device = 'cuda:0'
                            _loginfo("检测到 CUDA GPU，使用 GPU 加速", logger)
                        else:
                            _logwarn("未检测到 CUDA GPU，使用 CPU", logger)
                    except ImportError:
                        _logwarn("PyTorch 未安装，使用 CPU", logger)
                
                self.use_yolo = True
                _loginfo(f"YOLOv8 模型加载成功: {model_path}, 设备: {self._device}", logger)
            except ImportError as e:
                _logwarn(f"ultralytics 导入失败（将使用备用检测）: {e}", logger)
            except Exception as e:
                _logwarn(f"YOLO 模型加载失败: {e}", logger)
        
        if not self.use_yolo:
            _loginfo("使用 OpenCV DNN 备用检测方案", logger)
            self.backup_detector = True
    
    def detect(self, cv_image):
        """检测电表"""
        if cv_image is None:
            return []

        start_time = time.time()
        
        if self.use_yolo:
            detections = self._detect_yolo(cv_image)
        else:
            detections = self._detect_backup(cv_image)
        
        elapsed = (time.time() - start_time) * 1000
        self._total_time += elapsed
        self._count += 1
        avg_time = self._total_time / self._count if self._count > 0 else 0
        
        _loginfo(
            f"[Detector] 检测耗时: {elapsed:.1f}ms, 平均: {avg_time:.1f}ms, "
            f"设备: {self._device}, 检测数: {len(detections)}",
            self.logger
        )
        
        return detections
    
    def _detect_yolo(self, cv_image):
        """使用 YOLOv8 检测"""
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
        """备用检测方案"""
        detections = []
        h, w = cv_image.shape[:2]
        scale = 1.0
        work_img = cv_image

        if w > 640:
            scale = 640.0 / w
            nw, nh = 640, int(h * scale)
            work_img = cv2.resize(cv_image, (nw, nh), interpolation=cv2.INTER_LINEAR)
            h, w = nh, nw

        gray = cv2.cvtColor(work_img, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                       cv2.THRESH_BINARY_INV, 11, 2)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        min_area = (w * h) * max(0.0, min(self.min_area_ratio, 0.5))
        max_area = (w * h) * 0.8
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (min_area < area < max_area):
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect_ratio = bw / float(bh)
            
            if 0.5 < aspect_ratio < 2.0:
                conf = min(0.9, area / (w * h * 0.5))
                if scale < 1.0:
                    inv = 1.0 / scale
                    x, y, bw, bh = int(x*inv), int(y*inv), int(bw*inv), int(bh*inv)
                detections.append((x, y, x+bw, y+bh, conf, 0))
        
        detections.sort(key=lambda x: x[4], reverse=True)
        return detections[:3]
    
    def get_roi(self, cv_image, bbox):
        """提取 ROI"""
        x1, y1, x2, y2 = bbox
        h, w = cv_image.shape[:2]
        margin = 10
        x1 = max(0, x1 - margin)
        y1 = max(0, y1 - margin)
        x2 = min(w, x2 + margin)
        y2 = min(h, y2 + margin)
        return cv_image[y1:y2, x1:x2]
    
    def extract_corners(self, cv_image, bbox):
        """提取电表角点"""
        if self.keypoint_method == 'bbox':
            return self._get_bbox_corners(bbox)
        elif self.keypoint_method == 'contour':
            return self._extract_contour_corners(cv_image, bbox)
        else:
            _logwarn(f"[Detector] 未知 keypoint_method: {self.keypoint_method}，使用 bbox", self.logger)
            return self._get_bbox_corners(bbox)
    
    def _get_bbox_corners(self, bbox):
        x1, y1, x2, y2 = bbox
        return [[int(x1), int(y1)], [int(x2), int(y1)], [int(x2), int(y2)], [int(x1), int(y2)]]
    
    def _extract_contour_corners(self, cv_image, bbox):
        """轮廓角点提取"""
        x1, y1, x2, y2 = bbox
        
        h, w = cv_image.shape[:2]
        margin = 5
        x1 = max(0, x1 - margin)
        y1 = max(0, y1 - margin)
        x2 = min(w, x2 + margin)
        y2 = min(h, y2 + margin)
        
        roi = cv_image[y1:y2, x1:x2]
        if roi.size == 0:
            _logwarn("[Detector] ROI 为空，回退到 bbox", self.logger)
            return self._get_bbox_corners(bbox)
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            _logwarn("[Detector] 未找到轮廓，回退到 bbox", self.logger)
            return self._get_bbox_corners(bbox)
        
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        
        roi_area = roi.shape[0] * roi.shape[1]
        if area < roi_area * 0.1:
            _logwarn(f"[Detector] 轮廓面积太小 ({area/roi_area*100:.1f}%)，回退到 bbox", self.logger)
            return self._get_bbox_corners(bbox)
        
        epsilon = 0.02 * cv2.arcLength(max_contour, True)
        approx = cv2.approxPolyDP(max_contour, epsilon, True)
        
        if len(approx) < 4:
            epsilon = 0.04 * cv2.arcLength(max_contour, True)
            approx = cv2.approxPolyDP(max_contour, epsilon, True)
        
        if len(approx) < 4:
            hull = cv2.convexHull(max_contour)
            hull_approx = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
            if len(hull_approx) >= 4:
                approx = hull_approx[:4]
            else:
                _logwarn("[Detector] 无法提取4个角点，回退到 bbox", self.logger)
                return self._get_bbox_corners(bbox)
        
        corners = []
        for pt in approx:
            if len(pt[0]) >= 2:
                corners.append([pt[0][0] + x1, pt[0][1] + y1])
        
        if len(corners) < 4:
            return self._get_bbox_corners(bbox)
        
        return self._sort_corners(corners)
    
    def _sort_corners(self, corners):
        """排序角点"""
        if len(corners) != 4:
            return corners
        
        corners = sorted(corners, key=lambda p: p[0] + p[1])
        top_left = corners[0]
        bottom_right = corners[3]
        
        remaining = [corners[1], corners[2]]
        remaining = sorted(remaining, key=lambda p: p[0] - p[1])
        top_right = remaining[0]
        bottom_left = remaining[1]
        
        return [top_left, top_right, bottom_right, bottom_left]
    
    def get_edge_image(self, cv_image, bbox):
        """获取边缘检测结果图像（Canny边缘 + 轮廓）"""
        x1, y1, x2, y2 = bbox
        
        h, w = cv_image.shape[:2]
        margin = 5
        x1 = max(0, x1 - margin)
        y1 = max(0, y1 - margin)
        x2 = min(w, x2 + margin)
        y2 = min(h, y2 + margin)
        
        roi = cv_image[y1:y2, x1:x2]
        if roi.size == 0:
            return None
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓并绘制
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 创建彩色边缘图
        edge_vis = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        
        # 用红色绘制轮廓
        cv2.drawContours(edge_vis, contours, -1, (0, 0, 255), 2)
        
        return edge_vis
