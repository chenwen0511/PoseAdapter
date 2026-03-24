#!/usr/bin/env python3
"""
ROS2 Humble RTSP 相机发布节点（OpenCV 拉流 -> /camera/image_raw）。
"""
import argparse
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

DEFAULT_RTSP_URL = "rtsp://192.168.234.1:8554/test"


class RtspCameraPublisher(Node):
    def __init__(self, rtsp_url: str, topic: str, fps: float, frame_id: str):
        super().__init__("rtsp_camera_publisher")
        self.rtsp_url = rtsp_url
        self.pub = self.create_publisher(Image, topic, 10)
        self.frame_id = frame_id
        self.capture = None
        self.fail_count = 0
        self.last_reopen_ts = 0.0
        self.reopen_interval_sec = 2.0
        self.reopen_fail_threshold = 10

        self._open_capture()

        timer_period = 1.0 / fps if fps > 0 else 0.1
        self.timer = self.create_timer(timer_period, self._on_timer)
        self.get_logger().info(f"RTSP 源: {rtsp_url}")
        self.get_logger().info(f"发布话题: {topic}")

    def _open_capture(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

        # 优先走 TCP，降低网络抖动时的花屏与解码错误概率。
        os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|max_delay;500000")
        self.capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        if not self.capture.isOpened():
            raise RuntimeError(f"无法打开 RTSP 流: {self.rtsp_url}")

        self.fail_count = 0
        self.last_reopen_ts = time.time()
        self.get_logger().info("RTSP 连接已建立。")

    def _maybe_reopen_capture(self):
        now = time.time()
        if now - self.last_reopen_ts < self.reopen_interval_sec:
            return
        try:
            self.get_logger().warn("检测到连续解码失败，尝试重连 RTSP 流。")
            self._open_capture()
        except Exception as e:
            self.last_reopen_ts = now
            self.get_logger().warn(f"RTSP 重连失败: {e}")

    def _on_timer(self):
        if not rclpy.ok():
            return
        if self.capture is None:
            self._maybe_reopen_capture()
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.fail_count += 1
            self.get_logger().warn("读取 RTSP 帧失败，跳过本帧。", throttle_duration_sec=2.0)
            if self.fail_count >= self.reopen_fail_threshold:
                self._maybe_reopen_capture()
            return
        self.fail_count = 0

        msg = Image()
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        try:
            self.pub.publish(msg)
        except Exception as e:
            # 进程退出阶段可能出现 context invalid，忽略即可。
            self.get_logger().debug(f"发布失败（退出阶段可忽略）: {e}")

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="ROS2 RTSP 相机发布器")
    parser.add_argument("--rtsp-url", default=DEFAULT_RTSP_URL, help="RTSP 拉流地址")
    parser.add_argument("--topic", "-t", default="/camera/image_raw", help="发布图像话题")
    parser.add_argument("--fps", type=float, default=15.0, help="发布频率")
    parser.add_argument("--frame-id", default="camera", help="图像帧 frame_id")
    parser.add_argument("--reconnect-threshold", type=int, default=10, help="连续读帧失败多少次后尝试重连")
    parser.add_argument("--reconnect-interval", type=float, default=2.0, help="两次重连尝试最小间隔(秒)")
    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        node = RtspCameraPublisher(
            rtsp_url=args.rtsp_url,
            topic=args.topic,
            fps=args.fps,
            frame_id=args.frame_id,
        )
        node.reopen_fail_threshold = max(1, int(args.reconnect_threshold))
        node.reopen_interval_sec = max(0.5, float(args.reconnect_interval))
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
