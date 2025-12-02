#!/usr/bin/env python3
# coding=utf-8

"""
ROSイメージトピックを動画として保存するスクリプト

使い方:
  rosrun tc2025 save_image_video.py
  rosrun tc2025 save_image_video.py _topic:=/other/image _output:=output.mp4 _fps:=30
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class ImageVideoSaver:
    def __init__(self):
        rospy.init_node('image_video_saver')

        # パラメータ
        self.topic = rospy.get_param('~topic', '/yolo_detector/image_det')
        self.output_dir = rospy.get_param('~output_dir', '/home/nkb/catkin_ws/src/tc2025/videos')
        self.output_file = rospy.get_param('~output', None)
        self.fps = rospy.get_param('~fps', 30)
        self.codec = rospy.get_param('~codec', 'mp4v')

        # 出力ファイル名（指定がなければタイムスタンプで自動生成）
        if self.output_file is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.output_file = f'yolo_det_{timestamp}.mp4'

        # 出力パス
        os.makedirs(self.output_dir, exist_ok=True)
        self.output_path = os.path.join(self.output_dir, self.output_file)

        # CV Bridge
        self.bridge = CvBridge()

        # VideoWriter（最初のフレーム受信時に初期化）
        self.video_writer = None
        self.frame_count = 0

        # Subscriber
        rospy.Subscriber(self.topic, Image, self.image_callback)

        rospy.loginfo(f"Image Video Saver started")
        rospy.loginfo(f"Topic: {self.topic}")
        rospy.loginfo(f"Output: {self.output_path}")
        rospy.loginfo(f"FPS: {self.fps}")

    def image_callback(self, msg):
        try:
            # ROS ImageをOpenCV形式に変換
            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # 最初のフレームでVideoWriterを初期化
            if self.video_writer is None:
                h, w = cv_image.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*self.codec)
                self.video_writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
                rospy.loginfo(f"Video initialized: {w}x{h} @ {self.fps}fps")

            # フレームを書き込み
            self.video_writer.write(cv_image)
            self.frame_count += 1

            if self.frame_count % 100 == 0:
                rospy.loginfo(f"Saved {self.frame_count} frames")

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def shutdown(self):
        """終了処理"""
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo(f"Video saved: {self.output_path}")
            rospy.loginfo(f"Total frames: {self.frame_count}")

    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ImageVideoSaver()
        node.run()
    except rospy.ROSInterruptException:
        pass
