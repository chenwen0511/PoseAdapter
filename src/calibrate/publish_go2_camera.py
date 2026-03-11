#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 Go2 前置相机（Unitree SDK）图像发布到 ROS1 话题 /camera/image_raw。
与 pose_adapter（ROS1）同机使用时无需桥接，adapter 可直接订阅。

用法（需先 source ROS1 Noetic，且安装 unitree_sdk2py、opencv）：
  在 conda 的 task 环境中可用 python 或 python3.8（若遇 cyclonedds 冲突见文档）。
  export CYCLONEDDS_HOME=/home/unitree/cyclonedds/install  # Unitree SDK 依赖
  source /opt/ros/noetic/setup.bash
  python publish_go2_camera.py --no-network-interface

查看话题：rostopic hz /camera/image_raw
"""
import os
import sys
import argparse
import threading
import queue

# Unitree SDK 依赖 CycloneDDS，需在 import 前设置（可覆盖）
os.environ.setdefault("CYCLONEDDS_HOME", "/home/unitree/cyclonedds/install")

try:
    import rospy
    from sensor_msgs.msg import Image
    import cv2
    import numpy as np
    try:
        cv2.setLogLevel(cv2.LOG_LEVEL_ERROR)  # 抑制 "Corrupt JPEG data" 等解码警告
    except AttributeError:
        pass
except ImportError as e:
    err = str(e)
    if "foxy" in err or "humble" in err or "rosgraph_msgs" in err:
        print("当前为 ROS2 环境，本脚本需 ROS1。请勿 source ROS2，只执行: source /opt/ros/noetic/setup.bash")
    print("请先 source ROS1 并安装依赖。错误:", err)
    sys.exit(1)

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
except ImportError as e:
    err = str(e)
    if "cyclonedds" in err and "undefined symbol" in err:
        print("cyclonedds 与 ROS2 冲突时，请勿 source ROS2；仅 source /opt/ros/noetic/setup.bash")
    else:
        print("未安装 unitree_sdk2py，无法从 Go2 获取图像。请安装宇树 SDK。")
    print("实际错误:", err)
    sys.exit(1)


class Go2CameraPublisher:
    def __init__(self, topic: str):
        self.topic = topic
        self.pub = rospy.Publisher(topic, Image, queue_size=10)
        self.video_client = None
        self._init_camera()

    def _init_camera(self):
        try:
            self.video_client = VideoClient()
            self.video_client.SetTimeout(0.1)
            self.video_client.Init()
            rospy.loginfo("Go2 相机初始化成功，开始发布: %s" % self.topic)
        except Exception as e:
            rospy.logerr("相机初始化失败: %s" % e)
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
        """将 BGR numpy 转为 sensor_msgs/Image"""
        msg = Image()
        msg.header.stamp = rospy.Time.now()
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

    def run(self, hz: float = 10.0):
        frame_queue = queue.Queue(maxsize=1)
        self._grab_running = True
        grab_thread = threading.Thread(target=self._grab_loop, args=(frame_queue,), daemon=True)
        grab_thread.start()
        # 发布频率下限做一个保护，避免 0 或负数
        hz = float(hz) if hz and hz > 0 else 10.0
        rospy.loginfo("取图已在后台线程运行，发布频率约为 %.1f Hz" % hz)
        pub_count = 0
        wait_count = 0
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            try:
                frame = frame_queue.get(timeout=0.5)
            except queue.Empty:
                wait_count += 1
                if wait_count % 2 == 0:
                    rospy.loginfo("等待图像… (已发布 %d 帧)" % pub_count)
                rate.sleep()
                continue
            wait_count = 0
            if frame is not None:
                self.pub.publish(self._frame_to_imgmsg(frame))
                pub_count += 1
                if pub_count % 30 == 0:
                    rospy.loginfo("已发布 %d 帧" % pub_count)
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="发布 Go2 相机图像到 ROS1 话题")
    parser.add_argument("--topic", "-t", default="/camera/image_raw", help="发布的话题名")
    parser.add_argument("--network-interface", "-n", default="", help="DDS 网卡，如 eth0")
    parser.add_argument("--no-network-interface", action="store_true", help="不指定网卡，仅用 ChannelFactoryInitialize(0)")
    parser.add_argument("--hz", type=float, default=10.0, help="发布频率 Hz（默认 10.0）")
    args = parser.parse_args()

    # 必须在 rospy.init_node 之前初始化 Unitree DDS
    try:
        if args.no_network_interface or not args.network_interface:
            ChannelFactoryInitialize(0)
            print("[DDS] 已初始化: ChannelFactoryInitialize(0)")
        else:
            ChannelFactoryInitialize(0, args.network_interface)
            print("[DDS] 已初始化: ChannelFactoryInitialize(0, %s)" % args.network_interface)
    except Exception as e:
        print("DDS 初始化失败:", e)
        print("请尝试: python publish_go2_camera.py --no-network-interface")
        sys.exit(1)

    rospy.init_node("go2_camera_publisher", anonymous=False)
    node = Go2CameraPublisher(args.topic)
    try:
        node.run(hz=args.hz)
    except rospy.ROSInterruptException:
        pass
    finally:
        node._grab_running = False


if __name__ == "__main__":
    main()
