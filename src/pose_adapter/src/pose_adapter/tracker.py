#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
目标追踪器 - 基于 DeepSORT 或简单 IOU 追踪
"""

import numpy as np
import rospy
from collections import deque


class Track:
    """单个追踪目标"""
    
    _id_counter = 0
    
    def __init__(self, bbox, conf):
        """
        初始化追踪目标
        
        Args:
            bbox: (x1, y1, x2, y2)
            conf: 置信度
        """
        Track._id_counter += 1
        self.id = Track._id_counter
        self.bbox = bbox
        self.conf = conf
        self.age = 0
        self.hits = 1
        self.time_since_update = 0
        self.history = deque(maxlen=30)  # 保存历史位置
        self.history.append(bbox)
        
        # 卡尔曼滤波器状态（简化版）
        self.kf_state = self._init_kalman(bbox)
    
    def _init_kalman(self, bbox):
        """初始化卡尔曼滤波器状态"""
        x1, y1, x2, y2 = bbox
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        w, h = x2 - x1, y2 - y1
        return np.array([cx, cy, w, h, 0, 0, 0, 0], dtype=np.float32)
    
    def predict(self):
        """预测下一帧位置"""
        # 简化的恒定速度模型
        self.kf_state[0] += self.kf_state[4]  # cx += vx
        self.kf_state[1] += self.kf_state[5]  # cy += vy
        self.age += 1
        self.time_since_update += 1
        return self.get_bbox()
    
    def update(self, bbox):
        """更新追踪状态"""
        x1, y1, x2, y2 = bbox
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        w, h = x2 - x1, y2 - y1
        
        # 更新速度
        old_cx, old_cy = self.kf_state[0], self.kf_state[1]
        self.kf_state[4] = 0.7 * self.kf_state[4] + 0.3 * (cx - old_cx)  # vx
        self.kf_state[5] = 0.7 * self.kf_state[5] + 0.3 * (cy - old_cy)  # vy
        
        # 更新位置
        self.kf_state[0] = cx
        self.kf_state[1] = cy
        self.kf_state[2] = w
        self.kf_state[3] = h
        
        self.bbox = bbox
        self.history.append(bbox)
        self.hits += 1
        self.time_since_update = 0
    
    def get_bbox(self):
        """获取当前 bbox"""
        cx, cy, w, h = self.kf_state[0], self.kf_state[1], self.kf_state[2], self.kf_state[3]
        x1 = cx - w / 2
        y1 = cy - h / 2
        x2 = cx + w / 2
        y2 = cy + h / 2
        return (int(x1), int(y1), int(x2), int(y2))


class DeepSORTTracker:
    """
    简化版 DeepSORT 追踪器
    
    使用 IOU 匹配 + 卡尔曼滤波
    """
    
    def __init__(self, max_age=30, min_hits=3, iou_threshold=0.3):
        """
        初始化追踪器
        
        Args:
            max_age: 最大丢失帧数
            min_hits: 最小确认帧数
            iou_threshold: IOU 匹配阈值
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.tracks = []
        self.frame_count = 0
    
    def update(self, detections):
        """
        更新追踪器
        
        Args:
            detections: [(x1, y1, x2, y2, conf, class_id), ...]
            
        Returns:
            list: [(track_id, bbox, conf), ...] 确认的追踪结果
        """
        self.frame_count += 1
        
        # 预测已有追踪位置
        for track in self.tracks:
            track.predict()
        
        # 匹配检测和追踪
        matched, unmatched_dets, unmatched_tracks = self._match(detections)
        
        # 更新匹配的追踪
        for track_idx, det_idx in matched:
            self.tracks[track_idx].update(detections[det_idx][:4])
        
        # 为未匹配的检测创建新追踪
        for det_idx in unmatched_dets:
            bbox = detections[det_idx][:4]
            conf = detections[det_idx][4]
            self.tracks.append(Track(bbox, conf))
        
        # 标记未匹配的追踪
        for track_idx in unmatched_tracks:
            self.tracks[track_idx].time_since_update += 1
        
        # 删除过期追踪
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
        
        # 返回确认的追踪结果
        results = []
        for track in self.tracks:
            if track.hits >= self.min_hits or self.frame_count <= self.min_hits:
                results.append((track.id, track.get_bbox(), track.conf))
        
        return results
    
    def _match(self, detections):
        """
        匈牙利算法匹配检测和追踪
        
        Returns:
            matched: [(track_idx, det_idx), ...]
            unmatched_dets: [det_idx, ...]
            unmatched_tracks: [track_idx, ...]
        """
        if len(self.tracks) == 0:
            return [], list(range(len(detections))), []
        
        if len(detections) == 0:
            return [], [], list(range(len(self.tracks)))
        
        # 计算 IOU 矩阵
        iou_matrix = np.zeros((len(self.tracks), len(detections)))
        for i, track in enumerate(self.tracks):
            for j, det in enumerate(detections):
                iou_matrix[i, j] = self._iou(track.get_bbox(), det[:4])
        
        # 简单贪心匹配（实际应用可用 scipy.optimize.linear_sum_assignment）
        matched = []
        unmatched_dets = set(range(len(detections)))
        unmatched_tracks = set(range(len(self.tracks)))
        
        # 按 IOU 降序排序
        indices = np.dstack(np.unravel_index(np.argsort(-iou_matrix.ravel()), iou_matrix.shape))[0]
        
        for track_idx, det_idx in indices:
            if track_idx in unmatched_tracks and det_idx in unmatched_dets:
                if iou_matrix[track_idx, det_idx] >= self.iou_threshold:
                    matched.append((track_idx, det_idx))
                    unmatched_dets.remove(det_idx)
                    unmatched_tracks.remove(track_idx)
        
        return matched, list(unmatched_dets), list(unmatched_tracks)
    
    def _iou(self, bbox1, bbox2):
        """计算两个 bbox 的 IOU"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        xi1 = max(x1_1, x1_2)
        yi1 = max(y1_1, y1_2)
        xi2 = min(x2_1, x2_2)
        yi2 = min(y2_1, y2_2)
        
        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
        box1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
        box2_area = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = box1_area + box2_area - inter_area
        
        return inter_area / union_area if union_area > 0 else 0
    
    def get_track(self, track_id):
        """获取指定 ID 的追踪"""
        for track in self.tracks:
            if track.id == track_id:
                return track
        return None