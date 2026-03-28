#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电表读数识别：优先使用 YOLO 检测数字/读数区域（默认 model/eletrict_meter/best.pt），
预处理流程参考 plate_ocr.py；可选回退 PaddleOCR / 简易 OpenCV。
"""

import os
import re
import cv2
import numpy as np


def _repo_root_from_here():
    """ocr.py 位于 src/pose_adapter/src/pose_adapter/ocr.py -> 仓库根为向上 4 级。"""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def _resolve_meter_yolo_path(explicit_path=None):
    """
    默认权重：model/eletrict_meter/best.pt（仓库内实际目录名；亦尝试其它拼写）。
    """
    candidates = []
    if explicit_path:
        candidates.append(os.path.abspath(os.path.expanduser(explicit_path)))
        if not os.path.isabs(explicit_path):
            candidates.append(os.path.abspath(os.path.join(os.getcwd(), explicit_path)))
    root = _repo_root_from_here()
    # 首选 eletrict_meter，其次 electric_meter，最后 letrict_meter（历史拼写）
    for sub in ("eletrict_meter", "electric_meter", "letrict_meter"):
        candidates.append(os.path.join(root, "model", sub, "best.pt"))
        candidates.append(os.path.join(os.getcwd(), "model", sub, "best.pt"))

    for p in candidates:
        if p and os.path.isfile(p):
            return p
    return os.path.join(root, "model", "eletrict_meter", "best.pt")


def preprocess_meter(image):
    """
    预处理电表 ROI（参考 plate_ocr.preprocess_plate）：
    灰度、去噪、CLAHE、Otsu 二值、开运算去小噪点。
    返回单通道 uint8（二值图）。
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()

    denoised = cv2.GaussianBlur(gray, (3, 3), 0)
    denoised = cv2.medianBlur(denoised, 3)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(denoised)

    _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    return cleaned


def preprocess_meter_bgr(image):
    """将 preprocess_meter 结果转为 3 通道，供 YOLO 使用。"""
    cleaned = preprocess_meter(image)
    return cv2.merge([cleaned, cleaned, cleaned])


class MeterOCR:
    """电表读数：YOLO 检测 + 可选 Paddle / 备用方案。"""

    def __init__(
        self,
        use_paddle=True,
        model_path=None,
        meter_yolo_path=None,
        yolo_conf=0.25,
        use_gpu=True,
        logger=None,
    ):
        self.use_paddle = use_paddle
        self.ocr_engine = None
        self.logger = logger
        self.use_tesseract = False
        self.yolo_conf = float(yolo_conf)
        self.use_gpu = use_gpu
        self._yolo = None
        self._yolo_device = "cpu"
        self._meter_weights = _resolve_meter_yolo_path(meter_yolo_path or model_path)

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

        # YOLO 电表读数模型
        if os.path.isfile(self._meter_weights):
            try:
                from ultralytics import YOLO
                import torch

                self._yolo = YOLO(self._meter_weights)
                if use_gpu and torch.cuda.is_available():
                    self._yolo_device = "cuda:0"
                else:
                    self._yolo_device = "cpu"
                loginfo(f"电表 YOLO 模型加载成功: {self._meter_weights}, device={self._yolo_device}")
            except ImportError as e:
                logwarn(f"ultralytics 未安装，无法使用 YOLO 读数: {e}")
                self._yolo = None
            except Exception as e:
                logwarn(f"电表 YOLO 加载失败: {e}")
                self._yolo = None
        else:
            logwarn(f"未找到电表 YOLO 权重: {self._meter_weights}，将仅用 Paddle/备用方案")

        if use_paddle:
            try:
                from paddleocr import PaddleOCR

                self.ocr_engine = PaddleOCR(use_angle_cls=True, lang="en")
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
        try:
            import pytesseract

            self.use_tesseract = True
            loginfo("Tesseract OCR 备用方案")
        except ImportError:
            self.use_tesseract = False
            loginfo("使用 OpenCV 数字识别备用方案")

    def _class_to_char(self, cls_id, names_dict):
        """将 YOLO 类别 id 转为单个数字/字符。"""
        if names_dict is None:
            return str(int(cls_id) % 10) if 0 <= int(cls_id) <= 9 else None
        # ultralytics: names 可能是 {0: '0', ...} 或 list
        if isinstance(names_dict, (list, tuple)) and 0 <= int(cls_id) < len(names_dict):
            name = names_dict[int(cls_id)]
        else:
            name = names_dict.get(int(cls_id), None) if hasattr(names_dict, "get") else None
        if name is None:
            return None
        s = str(name).strip()
        # 类别名可能是 "0".."9" 或 "digit_3"
        if len(s) == 1 and s.isdigit():
            return s
        m = re.search(r"(\d)", s)
        if m:
            return m.group(1)
        return None

    def _recognize_yolo(self, bgr_image):
        """使用 YOLO 在 ROI 上检测数字/读数，按从左到右拼接。"""
        if self._yolo is None or bgr_image is None or bgr_image.size == 0:
            return None

        def run(img_bgr):
            results = self._yolo(
                img_bgr,
                verbose=False,
                device=self._yolo_device,
                conf=self.yolo_conf,
                half=False,
            )
            rows = []
            names = getattr(self._yolo, "names", None)
            if names is None and results:
                names = results[0].names

            for result in results:
                boxes = result.boxes
                if boxes is None or len(boxes) == 0:
                    continue
                rnames = getattr(result, "names", None) or names
                xyxy = boxes.xyxy.cpu().numpy()
                clss = boxes.cls.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                for i in range(len(xyxy)):
                    cx = (float(xyxy[i][0]) + float(xyxy[i][2])) / 2.0
                    cid = int(clss[i])
                    cf = float(confs[i])
                    ch = self._class_to_char(cid, rnames)
                    rows.append((cx, ch, cf, cid))

            if not rows:
                return None

            rows.sort(key=lambda x: x[0])
            chars = []
            for _, ch, _, _ in rows:
                if ch is not None:
                    chars.append(ch)
            if not chars:
                return None
            return "".join(chars)

        # 原图
        text = run(bgr_image)
        if text:
            return text
        # 与 plate_ocr 一致的预处理后三通道
        try:
            proc = preprocess_meter_bgr(bgr_image)
            return run(proc)
        except Exception:
            return None

    def _log_meter_reading(self, text):
        """将最终读数写入日志（构造时传入 logger，ROS2 用 Node.get_logger()；ROS1 可传 rospy 模块）。"""
        if self.logger is None:
            return
        try:
            import rospy as _rospy

            if self.logger is _rospy:
                if text:
                    _rospy.loginfo(f"[电表读数] {text}")
                else:
                    _rospy.logwarn("[电表读数] (无有效结果)")
                return
        except ImportError:
            pass
        if text:
            if hasattr(self.logger, "info"):
                self.logger.info(f"[电表读数] {text}")
        else:
            if hasattr(self.logger, "warning"):
                self.logger.warning("[电表读数] (无有效结果)")

    def recognize(self, image):
        """
        识别电表读数字符串。

        Args:
            image: OpenCV BGR 图像（一般为电表 ROI）

        Returns:
            str 或 None
        """
        if image is None or image.size == 0:
            self._log_meter_reading(None)
            return None

        text = None
        if self._yolo is not None:
            text = self._recognize_yolo(image)
        if not text:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if self.use_paddle:
                text = self._recognize_paddle(image, gray)
            elif self.use_tesseract:
                text = self._recognize_tesseract(gray)
            else:
                text = self._recognize_backup(gray)

        self._log_meter_reading(text)
        return text

    def _recognize_paddle(self, image_bgr, gray):
        try:
            # 与 plate_ocr 类似：原图 + 预处理后的二值图都试一次
            processed_bin = preprocess_meter(image_bgr)
            for img in (image_bgr, processed_bin):
                result = self.ocr_engine.ocr(img, cls=True)
                if result and result[0]:
                    texts = []
                    for line in result[0]:
                        if line and len(line) >= 2:
                            texts.append(str(line[1][0]))
                    joined = "".join(texts)
                    if joined.strip():
                        return re.sub(r"\s+", "", joined)
        except Exception:
            pass
        return self._recognize_backup(gray)

    def _recognize_tesseract(self, gray):
        try:
            import pytesseract

            _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
            text = pytesseract.image_to_string(binary, config="--psm 7")
            return text.strip() or None
        except Exception:
            pass
        return self._recognize_backup(gray)

    def _recognize_backup(self, gray):
        try:
            _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            digits = []
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                if w > 5 and h > 10 and w < 100:
                    digits.append((x, y, w, h))
            digits.sort(key=lambda d: d[0])
            result = ""
            for x, y, w, h in digits:
                aspect = w / float(h)
                if 0.2 < aspect < 1.0:
                    result += "0"
            return result if result else None
        except Exception:
            return None
