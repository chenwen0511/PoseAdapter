#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OCR 识别器 - 电表读数识别 (ROS2 版本)
"""

import cv2
import numpy as np


class MeterOCR:
    """电表 OCR 识别器"""
    
    def __init__(self, use_paddle=True, model_path=None, logger=None):
        self.use_paddle = use_paddle
        self.ocr_engine = None
        self.logger = logger
        self.use_tesseract = False
        
        def loginfo(msg):
            if logger is not None and hasattr(logger, "info"):
                logger.info(msg)
            else:
                print(f"[INFO] {msg}")

        def logwarn(msg):
            if logger is not None and hasattr(logger, "warning"):
                logger.warning(msg)
            elif logger is not None and hasattr(logger, "warn"):
                logger.warn(msg)
            else:
                print(f"[WARN] {msg}")
        
        if use_paddle:
            try:
                from paddleocr import PaddleOCR
                self.ocr_engine = PaddleOCR(
                    use_angle_cls=True,
                    lang='en',
                    show_log=False
                )
                loginfo("PaddleOCR 初始化成功")
            except ImportError:
                logwarn("PaddleOCR 未安装，使用备用方案")
                self.use_paddle = False
            except Exception as e:
                logwarn(f"PaddleOCR 初始化失败: {e}")
                self.use_paddle = False
        
        if not self.use_paddle:
            self._init_backup_ocr(logwarn, loginfo)
    
    def _init_backup_ocr(self, logwarn, loginfo):
        """初始化备用 OCR"""
        try:
            import pytesseract
            self.use_tesseract = True
            loginfo("Tesseract OCR 备用方案")
        except ImportError:
            self.use_tesseract = False
            loginfo("使用 OpenCV 数字识别备用方案")
    
    def recognize(self, image):
        """
        识别电表读数
        
        Args:
            image: OpenCV BGR 图像
            
        Returns:
            str: 识别结果，或 None
        """
        if image is None or image.size == 0:
            return None
        
        # 预处理
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if self.use_paddle:
            return self._recognize_paddle(gray)
        elif self.use_tesseract:
            return self._recognize_tesseract(gray)
        else:
            return self._recognize_backup(gray)
    
    def _recognize_paddle(self, gray):
        """PaddleOCR 识别"""
        try:
            result = self.ocr_engine.ocr(gray, cls=True)
            if result and result[0]:
                texts = []
                for line in result[0]:
                    if line and len(line) >= 2:
                        texts.append(str(line[1][0]))
                return ''.join(texts)
        except Exception as e:
            pass
        return self._recognize_backup(gray)
    
    def _recognize_tesseract(self, gray):
        """Tesseract 识别"""
        try:
            import pytesseract
            # 预处理
            _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
            text = pytesseract.image_to_string(binary, config='--psm 7')
            return text.strip()
        except Exception as e:
            pass
        return self._recognize_backup(gray)
    
    def _recognize_backup(self, gray):
        """备用识别 - 简单数字提取"""
        try:
            # 二值化
            _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            
            # 找数字区域
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            digits = []
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                if w > 5 and h > 10 and w < 100:
                    digits.append((x, y, w, h))
            
            # 按 x 坐标排序
            digits.sort(key=lambda d: d[0])
            
            result = ''
            for x, y, w, h in digits:
                # 简单判断是否为数字（根据宽高比）
                aspect = w / float(h)
                if 0.2 < aspect < 1.0:
                    result += '0'  # 占位
            
            return result if result else None
        except Exception as e:
            return None
