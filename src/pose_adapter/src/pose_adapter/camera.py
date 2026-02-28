#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机控制模块
处理拍照、云台控制等功能
"""

import os
import time
import uuid
import logging
from typing import Dict, Optional
from datetime import datetime

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("警告: OpenCV不可用，相机功能将被禁用")

# 导入Unitree SDK
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    UNITREE_VIDEO_AVAILABLE = True
except ImportError:
    UNITREE_VIDEO_AVAILABLE = False
    VideoClient = None
    print("警告: Unitree Video SDK不可用，相机功能将被禁用")


class CameraHandler:
    """相机处理器"""
    
    def __init__(self, config: Dict):
        """
        初始化相机处理器
        
        Args:
            config: 配置文件字典
        """
        self.config = config
        camera_config = config.get('camera', {})
        
        self.image_save_path = camera_config.get('image_save_path', './data/images')
        self.image_upload_url = camera_config.get('image_upload_url', '')
        
        # 创建保存目录
        os.makedirs(self.image_save_path, exist_ok=True)
        
        # 云台配置
        gimbal_config = camera_config.get('gimbal', {})
        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0
        self.max_pitch = gimbal_config.get('max_pitch', 90.0)
        self.min_pitch = gimbal_config.get('min_pitch', -90.0)
        self.max_yaw = gimbal_config.get('max_yaw', 180.0)
        self.min_yaw = gimbal_config.get('min_yaw', -180.0)
        
        # 日志
        self.logger = logging.getLogger(__name__)
        
        # Unitree Video客户端（用于Go2相机）
        self.video_client = None
        
        # 网络接口配置
        self.network_interface = config.get('robot', {}).get('network_interface', '')
        
        # 初始化相机
        self._init_camera()

    def _init_camera(self):
        """初始化相机设备（使用Unitree SDK的VideoClient）"""
        if not UNITREE_VIDEO_AVAILABLE or VideoClient is None:
            self.logger.warning("⚠️ Unitree Video SDK不可用，相机功能将被禁用")
            return
        
        try:
            # 初始化DDS通道（如果还未初始化）
            # 注意：这里假设ChannelFactoryInitialize已经在RobotControl中初始化过了
            # 如果还没有，我们需要初始化
            try:
                # 尝试初始化（如果已经初始化过，这个操作是幂等的）
                if self.network_interface:
                    ChannelFactoryInitialize(0, self.network_interface)
                    self.logger.debug(f"DDS初始化（相机） - 网络接口: {self.network_interface}")
                else:
                    ChannelFactoryInitialize(0)
                    self.logger.debug("DDS初始化（相机） - 自动检测网络接口")
            except Exception as e:
                self.logger.debug(f"DDS可能已初始化: {e}")
            
            # 创建VideoClient
            self.video_client = VideoClient()
            # 设置较短的超时时间，以提高获取帧的速度（30fps时每帧约33ms）
            self.video_client.SetTimeout(0.1)  # 100ms超时，足够获取一帧
            self.video_client.Init()
            self.logger.info("✅ Go2前置相机初始化成功（使用Unitree SDK）")
        except Exception as e:
            self.logger.error(f"❌ 相机初始化失败: {e}", exc_info=True)
            self.video_client = None
    
    def get_image_frame(self) -> Optional[np.ndarray]:
        """
        获取一帧图像（使用Unitree SDK）
        
        Returns:
            np.ndarray: OpenCV图像数组，失败返回None
        """
        if not self.video_client:
            return None
        
        if not CV2_AVAILABLE:
            return None
        
        try:
            code, data = self.video_client.GetImageSample()
            
            if code != 0:
                self.logger.debug(f"获取图像失败，错误码: {code}")
                return None
            
            # 将二进制数据转换为numpy数组
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            
            # 解码为OpenCV图像（抑制JPEG损坏警告）
            # 使用cv2.IMREAD_COLOR解码，忽略部分损坏的JPEG数据
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            
            # 如果第一次解码失败，尝试不忽略错误
            if image is None:
                # 尝试使用cv2.IMREAD_UNCHANGED
                image = cv2.imdecode(image_data, cv2.IMREAD_UNCHANGED)
                if image is None:
                    # 只有在连续多次失败时才记录警告（避免日志过多）
                    if not hasattr(self, '_decode_fail_count'):
                        self._decode_fail_count = 0
                    self._decode_fail_count += 1
                    # 每100次失败才记录一次警告
                    if self._decode_fail_count % 100 == 0:
                        self.logger.warning(f"图像解码失败（已失败 {self._decode_fail_count} 次），可能数据损坏")
                    return None
                elif len(image.shape) == 2:
                    # 如果是灰度图，转换为BGR
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                elif image.shape[2] == 4:
                    # 如果是RGBA，转换为BGR
                    image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            
            # 解码成功，重置失败计数
            if hasattr(self, '_decode_fail_count'):
                if self._decode_fail_count > 0:
                    self.logger.debug(f"图像解码恢复正常（之前失败 {self._decode_fail_count} 次）")
                self._decode_fail_count = 0
            
            return image
            
        except Exception as e:
            self.logger.error(f"获取图像帧失败: {e}", exc_info=True)
            return None
    
    def is_available(self) -> bool:
        """检查相机是否可用"""
        return self.video_client is not None
    
    def take_photo(self, gimbal_pitch: Optional[float] = None, 
                   gimbal_yaw: Optional[float] = None) -> Optional[Dict]:
        """
        拍照
        
        Args:
            gimbal_pitch: 云台俯仰角（度，可选）
            gimbal_yaw: 云台偏航角（度，可选）
            
        Returns:
            Dict: {"photo_id": str, "image_url": str, ...} 或 None
        """
        try:
            # 设置云台角度（如果有指定）
            if gimbal_pitch is not None or gimbal_yaw is not None:
                self.set_gimbal_angle(
                    gimbal_pitch if gimbal_pitch is not None else self.gimbal_pitch,
                    gimbal_yaw if gimbal_yaw is not None else self.gimbal_yaw
                )
                time.sleep(0.5)  # 等待云台到位
            
            # 生成照片ID和文件名
            photo_id = f"photo_{uuid.uuid4().hex[:8]}"
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{photo_id}_{timestamp}.jpg"
            filepath = os.path.join(self.image_save_path, filename)
            
            # 使用Unitree SDK获取图像
            frame = self.get_image_frame()
            if frame is not None:
                cv2.imwrite(filepath, frame)
                height, width = frame.shape[:2]
            else:
                self.logger.warning("相机拍照失败（无法获取图像帧）")
                return None
            
            # 上传图片（如果有配置上传URL）
            image_url = filepath  # 默认使用本地路径
            if self.image_upload_url:
                # 这里应该调用上传接口
                # image_url = self._upload_image(filepath)
                pass
            
            file_size = os.path.getsize(filepath) if os.path.exists(filepath) else 0
            
            self.logger.info(f"✅ 拍照成功: {photo_id}, 文件: {filepath}")
            
            return {
                "photo_id": photo_id,
                "image_url": image_url,
                "file_size": file_size,
                "width": width,
                "height": height
            }
            
        except Exception as e:
            self.logger.error(f"❌ 拍照失败: {e}", exc_info=True)
            return None
    
    def set_gimbal_angle(self, pitch: float, yaw: float, speed: Optional[float] = None) -> bool:
        """
        设置云台角度
        
        Args:
            pitch: 俯仰角（度）
            yaw: 偏航角（度）
            speed: 转动速度（度/秒，可选）
            
        Returns:
            bool: 是否成功
        """
        try:
            # 角度限制
            pitch = max(self.min_pitch, min(self.max_pitch, pitch))
            yaw = max(self.min_yaw, min(self.max_yaw, yaw))
            
            # 这里应该调用实际的云台控制接口
            # 示例：调用Go2 SDK的云台控制API
            self.gimbal_pitch = pitch
            self.gimbal_yaw = yaw
            
            self.logger.info(f"云台角度设置: pitch={pitch}°, yaw={yaw}°")
            return True
            
        except Exception as e:
            self.logger.error(f"设置云台角度失败: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        # VideoClient不需要显式释放，DDS会自动管理
        self.video_client = None

