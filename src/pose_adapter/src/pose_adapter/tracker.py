#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标追踪器 - 基于 DeepSORT 或简单 IOU 追踪 (ROS2 版本)
"""

import numpy as np
from collections import deque


class Track:
    """单个追踪目标"""
    
    _id_counter = 0
    
    def __init__(self, bbox, conf, keypoints=None):
        Track._id_counter += 1
        self.id = Track._id_counter
        self.bbox = bbox
        self.conf = conf
        self.keypoints = keypoints  # 保存关键点
        self.age = 0
        self.hits = 1
        self.time_since_update = 0
        self.history = deque(maxlen=30)
        self.history.append(bbox)
        self.kf_state = self._init_kalman(bbox)
    
    def _init_kalman(self, bbox):
        """初始化卡尔曼滤波器状态"""
        x1, y1, x2, y2 = bbox
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        w, h = x2 - x1, y2 - y1
        return np.array([cx, cy, w, h, 0, 0, 0, 0], dtype=np.float32)
    
    def predict(self):
        """预测下一帧位置"""
        self.kf_state[0] += self.kf_state[4]
        self.kf_state[1] += self.kf_state[5]
        self.age += 1
        self.time_since_update += 1
        return self
    
    def update(self, bbox, conf, keypoints=None):
        """更新追踪目标"""
        x1, y1, x2, y2 = bbox
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        w, h = x2 - x1, y2 - y1
        
        # 简化的卡尔曼更新
        self.kf_state[0] = cx
        self.kf_state[1] = cy
        self.kf_state[2] = w
        self.kf_state[3] = h
        
        self.bbox = bbox
        self.conf = conf
        self.keypoints = keypoints  # 更新关键点
        self.hits += 1
        self.time_since_update = 0
        self.history.append(bbox)
        return self
    
    def to_bbox(self):
        """从状态获取 bbox"""
        cx, cy, w, h = self.kf_state[:4]
        x1 = cx - w / 2
        y1 = cy - h / 2
        x2 = cx + w / 2
        y2 = cy + h / 2
        return [x1, y1, x2, y2]


class DeepSORTTracker:
    """DeepSORT 追踪器（简化版）"""
    
    def __init__(self, max_age=30, min_hits=3, iou_threshold=0.3):
        """
        初始化追踪器
        
        Args:
            max_age: 最大丢失帧数
            min_hits: 最小命中次数
            iou_threshold: IOU 匹配阈值
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.tracks = []
    
    def update(self, detections):
        """
        更新追踪
        
        Args:
            detections: [(x1, y1, x2, y2, conf, cls), ...]
            
        Returns:
            [(track_id, bbox, conf), ...]
        """
        # 预测
        for track in self.tracks:
            track.predict()
        
        # 匹配
        matched, unmatched_dets, unmatched_tracks = self._associate(detections)
        
        # 更新已匹配
        for det_idx, track_idx in matched:
            det = detections[det_idx]
            keypoints = det[6] if len(det) > 6 else None  # 提取关键点
            self.tracks[track_idx].update(det[:4], det[4], keypoints)
        
        # 删除丢失跟踪
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
        
        # 添加新跟踪
        for det_idx in unmatched_dets:
            det = detections[det_idx]
            keypoints = det[6] if len(det) > 6 else None
            self.tracks.append(Track(det[:4], det[4], keypoints))
        
        # 返回活跃跟踪
        result = []
        for track in self.tracks:
            if track.time_since_update == 0 and track.hits >= self.min_hits:
                # 返回格式: (track_id, bbox, conf, keypoints)
                result.append((track.id, track.bbox, track.conf, track.keypoints))
        
        return result
    
    def _associate(self, detections):
        """IOU 匹配"""
        if not self.tracks:
            return [], list(range(len(detections))), []
        
        if not detections:
            return [], [], list(range(len(self.tracks)))
        
        iou_matrix = np.zeros((len(detections), len(self.tracks)))
        for d, det in enumerate(detections):
            for t, track in enumerate(self.tracks):
                iou_matrix[d, t] = self._iou(det[:4], track.to_bbox())
        
        matched = []
        unmatched_dets = list(range(len(detections)))
        unmatched_tracks = list(range(len(self.tracks)))
        
        # 贪婪匹配
        for t in range(len(self.tracks)):
            if len(unmatched_dets) == 0:
                break
            best_iou = self.iou_threshold
            best_d = -1
            for d in unmatched_dets:
                if iou_matrix[d, t] > best_iou:
                    best_iou = iou_matrix[d, t]
                    best_d = d
            if best_d >= 0:
                matched.append((best_d, t))
                unmatched_dets.remove(best_d)
                if t in unmatched_tracks:
                    unmatched_tracks.remove(t)
        
        return matched, unmatched_dets, unmatched_tracks
    
    def _iou(self, bbox1, bbox2):
        """计算 IOU"""
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])
        
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        
        area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        
        union = area1 + area2 - intersection
        
        if union <= 0:
            return 0
        
        return intersection / union
