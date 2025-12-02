#!/usr/bin/env python3
# coding=utf-8

"""
yaw_offset計算ノード

/Odometryと/amcl_poseのヨー角偏差を計算し、
5秒ごとにconfigファイルに保存する

偏差の定義: yaw_offset = amcl_pose - Odometry
"""

import rospy
import numpy as np
import yaml
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
from collections import deque
import threading


class YawOffsetCalculator:
    def __init__(self):
        rospy.init_node('yaw_offset_calculator')

        # パラメータ
        self.output_file = rospy.get_param('~output_file',
            '/home/nkb/catkin_ws/src/tc2025/config/yaw_offset.yaml')
        self.save_interval = rospy.get_param('~save_interval', 5.0)  # 秒
        self.time_window = rospy.get_param('~time_window', 30.0)  # 計算に使う時間窓

        # データ保存用
        self.odom_data = deque()
        self.amcl_data = deque()
        self.lock = threading.Lock()

        # 現在の偏差
        self.current_offset = 0.0
        self.offset_std = 0.0

        # Subscriber
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        # 保存タイマー
        rospy.Timer(rospy.Duration(self.save_interval), self.save_callback)

        rospy.loginfo(f"Yaw offset calculator started")
        rospy.loginfo(f"Output file: {self.output_file}")
        rospy.loginfo(f"Save interval: {self.save_interval} sec")

    def quaternion_to_yaw(self, q):
        """Quaternionからyaw角を取得"""
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

    def normalize_angle(self, angle):
        """角度を-piからpiの範囲に正規化"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def odom_callback(self, msg):
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        t = msg.header.stamp.to_sec()

        with self.lock:
            self.odom_data.append((t, yaw))
            self._cleanup_old_data(t)

    def amcl_callback(self, msg):
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        t = msg.header.stamp.to_sec()

        with self.lock:
            self.amcl_data.append((t, yaw))
            self._cleanup_old_data(t)

    def _cleanup_old_data(self, current_time):
        """古いデータを削除"""
        cutoff_time = current_time - self.time_window

        while self.odom_data and self.odom_data[0][0] < cutoff_time:
            self.odom_data.popleft()

        while self.amcl_data and self.amcl_data[0][0] < cutoff_time:
            self.amcl_data.popleft()

    def calculate_offset(self):
        """偏差を計算"""
        with self.lock:
            odom_list = list(self.odom_data)
            amcl_list = list(self.amcl_data)

        if len(odom_list) < 2 or len(amcl_list) < 2:
            return None, None

        odom_times = np.array([d[0] for d in odom_list])
        odom_yaws = np.array([d[1] for d in odom_list])
        amcl_times = np.array([d[0] for d in amcl_list])
        amcl_yaws = np.array([d[1] for d in amcl_list])

        # amclの時刻でodomを補間
        odom_interp = np.interp(amcl_times, odom_times, odom_yaws)

        # 偏差計算: yaw_offset = amcl - odom
        yaw_diff = np.array([self.normalize_angle(a - o)
                           for a, o in zip(amcl_yaws, odom_interp)])

        mean_offset = np.mean(yaw_diff)
        std_offset = np.std(yaw_diff)

        return mean_offset, std_offset

    def save_callback(self, event):
        """偏差をファイルに保存"""
        mean_offset, std_offset = self.calculate_offset()

        if mean_offset is None:
            rospy.logwarn("Not enough data to calculate offset")
            return

        self.current_offset = mean_offset
        self.offset_std = std_offset

        # YAMLファイルに保存
        data = {
            'yaw_offset': float(mean_offset),
            'yaw_offset_deg': float(np.degrees(mean_offset)),
            'yaw_offset_std': float(std_offset),
            'yaw_offset_std_deg': float(np.degrees(std_offset)),
        }

        # ディレクトリが存在しない場合は作成
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        with open(self.output_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        rospy.loginfo(f"Saved yaw_offset: {mean_offset:.4f} rad ({np.degrees(mean_offset):.2f} deg), "
                     f"std: {std_offset:.4f} rad")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = YawOffsetCalculator()
        node.run()
    except rospy.ROSInterruptException:
        pass
