#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
OCR 识别器 - 电表读数识别
"""

import cv2
import numpy as np
import rospy


class MeterOCR:
    """
    电表 OCR 识别器
    
    识别电表读数，支持 PaddleOCR 和备用方案
    """
    
    def __init__(self, use_paddle=True, model_path=None):
        """
        初始化 OCR
        
        Args:
            use_paddle: 是否使用 PaddleOCR
            model_path: 模型路径
        """
        self.use_paddle = use_paddle
        self.ocr_engine = None
        
        if use_paddle:
            try:
                from paddleocr import PaddleOCR
                self.ocr_engine = PaddleOCR(
                    use_angle_cls=True,
                    lang='en',
                    show_log=False
                )
                rospy.loginfo("PaddleOCR 初始化成功")
            except ImportError:
                rospy.logwarn("PaddleOCR 未安装，使用备用方案")
                self.use_paddle = False
            except Exception as e:
                rospy.logwarn(f"PaddleOCR 初始化失败: {e}")
                self.use_paddle = False
        
        if not self.use_paddle:
            self._init_backup_ocr()
    
    def _init_backup_ocr(self):
        """初始化备用 OCR（基于 Tesseract 或简单数字识别）"""
        try:
            import pytesseract
            self.use_tesseract = True
            rospy.loginfo("Tesseract OCR 备用方案")
        except ImportError:
            self.use_tesseract = False
            rospy.loginfo("使用 OpenCV 数字识别备用方案")
    
    def recognize(self, image):
        """
        识别电表读数
        
        Args:
            image: OpenCV 图像 (BGR)
            
        Returns:
            dict: {
                'text': str,        # 识别的文本
                'confidence': float, # 置信度
                'success': bool
            }
        """
        if image is None or image.size == 0:
            return {'text': '', 'confidence': 0, 'success': False}
        
        if self.use_paddle:
            return self._recognize_paddle(image)
        elif self.use_tesseract:
            return self._recognize_tesseract(image)
        else:
            return self._recognize_backup(image)
    
    def _recognize_paddle(self, image):
        """使用 PaddleOCR 识别"""
        try:
            result = self.ocr_engine.ocr(image, cls=True)
            
            if result and result[0]:
                texts = []
                confidences = []
                for line in result[0]:
                    if line:
                        text = line[1][0]
                        conf = line[1][1]
                        texts.append(text)
                        confidences.append(conf)
                
                # 合并文本，提取数字
                full_text = ' '.join(texts)
                numbers = self._extract_numbers(full_text)
                
                avg_conf = np.mean(confidences) if confidences else 0
                
                return {
                    'text': numbers,
                    'raw_text': full_text,
                    'confidence': float(avg_conf),
                    'success': len(numbers) > 0
                }
            
            return {'text': '', 'confidence': 0, 'success': False}
            
        except Exception as e:
            rospy.logerr(f"PaddleOCR 识别失败: {e}")
            return {'text': '', 'confidence': 0, 'success': False}
    
    def _recognize_tesseract(self, image):
        """使用 Tesseract 识别"""
        try:
            import pytesseract
            
            # 预处理
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
            # OCR
            custom_config = r'--oem 3 --psm 6 -c tessedit_char_whitelist=0123456789.'
            text = pytesseract.image_to_string(binary, config=custom_config)
            
            numbers = self._extract_numbers(text)
            
            return {
                'text': numbers,
                'raw_text': text.strip(),
                'confidence': 0.7 if numbers else 0,
                'success': len(numbers) > 0
            }
            
        except Exception as e:
            rospy.logerr(f"Tesseract 识别失败: {e}")
            return {'text': '', 'confidence': 0, 'success': False}
    
    def _recognize_backup(self, image):
        """
        备用识别方案 - 基于轮廓的数字分割
        简化版，仅用于测试
        """
        try:
            # 预处理
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
            
            # CLAHE 增强
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            
            # 二值化
            _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            
            # 形态学操作
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 筛选可能的数字区域
            digit_regions = []
            h, w = binary.shape
            for cnt in contours:
                x, y, bw, bh = cv2.boundingRect(cnt)
                aspect = bw / float(bh)
                area = bw * bh
                
                # 数字通常有一定宽高比
                if 0.2 < aspect < 1.0 and area > (h * w * 0.005):
                    digit_regions.append((x, y, bw, bh))
            
            # 按 x 排序
            digit_regions.sort(key=lambda r: r[0])
            
            # 简单的数字数量作为读数（实际应用需要训练分类器）
            num_digits = len(digit_regions)
            
            return {
                'text': str(num_digits) + '_digits_detected',
                'confidence': 0.5,
                'success': num_digits > 0,
                'num_digits': num_digits
            }
            
        except Exception as e:
            rospy.logerr(f"备用识别失败: {e}")
            return {'text': '', 'confidence': 0, 'success': False}
    
    def _extract_numbers(self, text):
        """从文本中提取数字"""
        import re
        numbers = re.findall(r'\d+\.?\d*', text)
        return ''.join(numbers)
    
    def preprocess_for_ocr(self, image):
        """
        预处理图像以提高 OCR 准确率
        
        Args:
            image: OpenCV 图像
            
        Returns:
            预处理后的图像
        """
        # 调整大小
        h, w = image.shape[:2]
        if w < 200:
            scale = 200 / w
            image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        
        # 灰度
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # CLAHE 增强对比度
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        
        return enhanced