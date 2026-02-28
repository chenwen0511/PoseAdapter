#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于 OpenCV 的相机标定脚本（不依赖 ROS cameracalibrator GUI）
订阅 /camera/image_raw，用棋盘格采集多张图后计算内参并保存为 yaml。
用法（需先 source ROS2）：
  python3 calibrate_opencv.py --topic /camera/image_raw --size 8x5 --square 0.025 --out calib_result.yaml
交互：窗口里按 s 采集当前帧，采够约 15 张后按 c 标定并保存，按 q 退出。
若无窗口：加 --headless 自动每隔约 2 秒采一帧，采够 20 张后自动标定保存。
"""
import argparse
import sys
import time
import yaml
import numpy as np
import cv2

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError as e:
    print("请先 source ROS2 环境，例如: source /opt/ros/foxy/setup.bash")
    print("错误:", e)
    sys.exit(1)

# 棋盘格内角点数量（宽 x 高）
GRID_COLS = 8
GRID_ROWS = 5
SQUARE_SIZE = 0.025  # 方格边长，米
OUTPUT_YAML = "calib_result.yaml"
IMAGE_TOPIC = "/camera/image_raw"
HEADLESS = False


def get_object_points(cols, rows, square_size):
    """棋盘格在平面 z=0 上的 3D 角点坐标"""
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size
    return objp


def save_calib_yaml(path, K, dist, width, height, camera_name="camera"):
    """保存为与 read_calib_params / default_camera_calib 兼容的 yaml"""
    data = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": camera_name,
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": K.flatten().tolist(),
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": dist.flatten().tolist()},
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": list(K[0, :]) + [0.0] + list(K[1, :]) + [0.0] + list(K[2, :]) + [0.0],
        },
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=None, allow_unicode=True)
    print(f"已保存: {path}")


class CalibNode(Node):
    def __init__(self, topic, grid_cols, grid_rows, square_size, out_path, headless):
        super().__init__("calibrate_opencv_node")
        self.topic = topic
        self.grid_cols = grid_cols
        self.grid_rows = grid_rows
        self.square_size = square_size
        self.out_path = out_path
        self.headless = headless
        self.bridge = CvBridge()
        self.latest_image = None
        self.obj_points = get_object_points(grid_cols, grid_rows, square_size)
        self.all_object_points = []
        self.all_image_points = []
        self.image_size = None
        self.sub = self.create_subscription(Image, topic, self._callback, 10)
        self.get_logger().info(f"订阅: {topic}")

    def _callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.image_size is None:
                h, w = self.latest_image.shape[:2]
                self.image_size = (w, h)
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")

    def run_with_gui(self):
        win = "calibrate: s=采集 c=标定保存 q=退出"
        needed = 15
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image is None:
                continue
            img = self.latest_image.copy()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (self.grid_cols, self.grid_rows), None)
            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                cv2.drawChessboardCorners(img, (self.grid_cols, self.grid_rows), corners2, ret)
                status = f"检测到棋盘格 | 已采集 {len(self.all_image_points)}/{needed}"
            else:
                status = f"未检测到棋盘格 | 已采集 {len(self.all_image_points)}/{needed}"
            cv2.putText(img, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow(win, img)
            key = cv2.waitKey(100) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s") and ret:
                self.all_object_points.append(self.obj_points)
                self.all_image_points.append(corners2)
                self.get_logger().info(f"采集 {len(self.all_image_points)}/{needed}")
            if key == ord("c"):
                if len(self.all_image_points) < 5:
                    print("至少需要 5 张有效图像，当前:", len(self.all_image_points))
                else:
                    self._do_calibrate_and_save()
                    break
        cv2.destroyAllWindows()

    def run_headless(self):
        needed = 20
        interval = 2.0
        last_capture = 0
        print(f"无头模式：每 {interval} 秒自动采集，共需 {needed} 张。")
        while rclpy.ok() and len(self.all_image_points) < needed:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_image is None:
                continue
            now = time.time()
            if now - last_capture < interval:
                continue
            gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (self.grid_cols, self.grid_rows), None)
            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                self.all_object_points.append(self.obj_points)
                self.all_image_points.append(corners2)
                if self.image_size is None:
                    h, w = self.latest_image.shape[:2]
                    self.image_size = (w, h)
                last_capture = now
                print(f"采集 {len(self.all_image_points)}/{needed}")
        if len(self.all_image_points) >= 5:
            self._do_calibrate_and_save()
        else:
            print("采集数量不足，至少需要 5 张有效图像。")

    def _do_calibrate_and_save(self):
        if self.image_size is None or len(self.all_image_points) < 5:
            print("无法标定：图像数量或尺寸不足")
            return
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.all_object_points,
            self.all_image_points,
            self.image_size,
            None,
            None,
        )
        print("标定完成。内参 K:\n", K)
        print("畸变系数:", dist.ravel())
        save_calib_yaml(
            self.out_path,
            K,
            dist,
            self.image_size[0],
            self.image_size[1],
        )


def main():
    global GRID_COLS, GRID_ROWS, SQUARE_SIZE, OUTPUT_YAML, IMAGE_TOPIC, HEADLESS
    parser = argparse.ArgumentParser(description="OpenCV 相机标定（订阅 ROS2 图像）")
    parser.add_argument("--topic", "-t", default=IMAGE_TOPIC, help="图像话题")
    parser.add_argument("--size", "-s", default="8x5", help="棋盘格内角点 宽x高，如 8x5")
    parser.add_argument("--square", default=0.025, type=float, help="方格边长(米)")
    parser.add_argument("--out", "-o", default=OUTPUT_YAML, help="输出 yaml 路径")
    parser.add_argument("--headless", action="store_true", help="无窗口，自动定时采集后标定")
    args = parser.parse_args()
    w, h = args.size.lower().split("x")
    GRID_COLS, GRID_ROWS = int(w), int(h)
    SQUARE_SIZE = args.square
    OUTPUT_YAML = args.out
    IMAGE_TOPIC = args.topic
    HEADLESS = args.headless

    rclpy.init()
    node = CalibNode(IMAGE_TOPIC, GRID_COLS, GRID_ROWS, SQUARE_SIZE, OUTPUT_YAML, HEADLESS)
    try:
        if HEADLESS:
            node.run_headless()
        else:
            node.run_with_gui()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
