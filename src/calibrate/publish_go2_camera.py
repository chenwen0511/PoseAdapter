#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 Go2 前置相机（Unitree SDK）图像发布到 ROS2 话题 /camera/image_raw。
标定或其它节点需要图像时，先在本机运行此节点。

用法（需先 source ROS2，且安装 unitree_sdk2py、opencv、cv_bridge）：
  python3.8 publish_go2_camera.py --no-network-interface
本脚本会令 ROS2 使用 domain 1（Unitree SDK 占 domain 0）。标定或查看话题时需同一 domain：
  export ROS_DOMAIN_ID=1
  ros2 topic hz /camera/image_raw
  ./calibrate_go2_onekey.sh
"""
import os
import sys
import argparse
import threading
import queue

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    import cv2
    import numpy as np
    try:
        cv2.setLogLevel(cv2.LOG_LEVEL_ERROR)  # 抑制 "Corrupt JPEG data" 等解码警告
    except AttributeError:
        pass
except ImportError as e:
    print("请先 source ROS2 并安装依赖。错误:", e)
    sys.exit(1)

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
except ImportError:
    print("未安装 unitree_sdk2py，无法从 Go2 获取图像。请安装宇树 SDK。")
    sys.exit(1)


class Go2CameraPublisher(Node):
    def __init__(self, topic: str):
        super().__init__("go2_camera_publisher")
        self.topic = topic
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        # 使用 RELIABLE 以兼容 ros2 topic hz / 标定脚本等默认订阅
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Image, topic, qos)
        self.video_client = None
        self._init_camera()

    def _init_camera(self):
        try:
            # ChannelFactory 已在 main() 中于 rclpy.init() 之前初始化，此处仅创建 VideoClient
            self.video_client = VideoClient()
            self.video_client.SetTimeout(0.1)
            self.video_client.Init()
            self.get_logger().info("Go2 相机初始化成功，开始发布: %s" % self.topic)
        except Exception as e:
            self.get_logger().error("相机初始化失败: %s" % e)
            self.video_client = None

    def get_frame(self):
        if not self.video_client:
            return None
        try:
            code, data = self.video_client.GetImageSample()
            if code != 0:
                return None
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if image is None:
                image = cv2.imdecode(image_data, cv2.IMREAD_UNCHANGED)
                if image is not None and len(image.shape) == 2:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            return image
        except Exception:
            return None

    def _frame_to_imgmsg(self, frame: np.ndarray) -> Image:
        """将 BGR numpy 转为 sensor_msgs/Image，不依赖 cv_bridge（避免 libffi 等库冲突）"""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.ravel().tobytes()
        return msg

    def _grab_loop(self, frame_queue):
        """单独线程中取图，避免 GetImageSample() 阻塞主循环"""
        while getattr(self, "_grab_running", True):
            frame = self.get_frame()
            try:
                frame_queue.put_nowait(frame)
            except queue.Full:
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    frame_queue.put_nowait(frame)
                except queue.Full:
                    pass

    def run(self):
        frame_queue = queue.Queue(maxsize=1)
        self._grab_running = True
        grab_thread = threading.Thread(target=self._grab_loop, args=(frame_queue,), daemon=True)
        grab_thread.start()
        self.get_logger().info("取图已在后台线程运行，主循环每 0.5 秒检查一次…")
        pub_count = 0
        wait_count = 0
        while rclpy.ok():
            try:
                frame = frame_queue.get(timeout=0.5)
            except queue.Empty:
                wait_count += 1
                if wait_count % 2 == 0:
                    self.get_logger().info("等待图像… (已发布 %d 帧，说明 GetImageSample 可能阻塞或相机无数据)" % pub_count)
                rclpy.spin_once(self, timeout_sec=0)
                continue
            wait_count = 0
            if frame is not None:
                self.pub.publish(self._frame_to_imgmsg(frame))
                pub_count += 1
                if pub_count % 30 == 0:
                    self.get_logger().info("已发布 %d 帧" % pub_count)
            rclpy.spin_once(self, timeout_sec=0)


def main():
    parser = argparse.ArgumentParser(description="发布 Go2 相机图像到 ROS2 话题")
    parser.add_argument("--topic", "-t", default="/camera/image_raw", help="发布的话题名")
    parser.add_argument("--network-interface", "-n", default="", help="DDS 网卡，如 eth0；若报 domain 错误可去掉此参数试 --no-network-interface")
    parser.add_argument("--no-network-interface", action="store_true", help="不指定网卡，仅用 ChannelFactoryInitialize(0)")
    args = parser.parse_args()

    # 必须在 rclpy.init() 之前初始化 Unitree DDS，否则易出现 cyclonedds domain 冲突
    try:
        if args.no_network_interface or not args.network_interface:
            ChannelFactoryInitialize(0)
            print("[DDS] 已初始化: ChannelFactoryInitialize(0)")
        else:
            ChannelFactoryInitialize(0, args.network_interface)
            print("[DDS] 已初始化: ChannelFactoryInitialize(0, %s)" % args.network_interface)
    except Exception as e:
        print("DDS 初始化失败:", e)
        print("请尝试: python3.8 publish_go2_camera.py --no-network-interface")
        sys.exit(1)

    # Unitree SDK 已占用 domain 0，本进程内 ROS2 必须用 domain 1，否则 rmw_create_node 会报错
    os.environ["ROS_DOMAIN_ID"] = "1"
    rclpy.init()
    node = Go2CameraPublisher(args.topic)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
