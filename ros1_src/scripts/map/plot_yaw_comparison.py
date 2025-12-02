#!/usr/bin/env python3
# coding=utf-8

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
from collections import deque
import threading

class YawComparisonPlotter:
    def __init__(self):
        rospy.init_node('yaw_comparison_plotter')

        # データ保存用（時刻、yaw）
        self.odom_data = deque()
        self.amcl_data = deque()

        # 保持時間（秒）
        self.time_window = 60.0

        # 最新のyaw値
        self.odom_yaw = None
        self.amcl_yaw = None

        # ロック
        self.lock = threading.Lock()

        # Subscriber
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        # プロット設定
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('Yaw Comparison: /Odometry vs /amcl_pose')

        rospy.loginfo('Yaw comparison plotter started')
        rospy.loginfo('Subscribing to /Odometry and /amcl_pose')

    def quaternion_to_yaw(self, q):
        """Quaternionからyaw角を取得"""
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]  # yaw

    def odom_callback(self, msg):
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        t = msg.header.stamp.to_sec()

        with self.lock:
            self.odom_yaw = yaw
            self.odom_data.append((t, yaw))
            self._cleanup_old_data(t)

    def amcl_callback(self, msg):
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        t = msg.header.stamp.to_sec()

        with self.lock:
            self.amcl_yaw = yaw
            self.amcl_data.append((t, yaw))
            self._cleanup_old_data(t)

    def _cleanup_old_data(self, current_time):
        """古いデータを削除（time_window秒より前のデータ）"""
        cutoff_time = current_time - self.time_window

        while self.odom_data and self.odom_data[0][0] < cutoff_time:
            self.odom_data.popleft()

        while self.amcl_data and self.amcl_data[0][0] < cutoff_time:
            self.amcl_data.popleft()

    def normalize_angle(self, angle):
        """角度を-piからpiの範囲に正規化"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def update_plot(self):
        with self.lock:
            odom_list = list(self.odom_data)
            amcl_list = list(self.amcl_data)

        if len(odom_list) < 2 or len(amcl_list) < 2:
            return

        # データ抽出
        odom_times = np.array([d[0] for d in odom_list])
        odom_yaws = np.array([d[1] for d in odom_list])
        amcl_times = np.array([d[0] for d in amcl_list])
        amcl_yaws = np.array([d[1] for d in amcl_list])

        # 基準時刻を最初の時刻に
        t0 = min(odom_times[0], amcl_times[0])
        odom_times = odom_times - t0
        amcl_times = amcl_times - t0

        # 偏差計算（amclの時刻でodomを補間）
        # yaw_offset = amcl - odom （gps_localization.pyと同じ定義）
        if len(odom_times) >= 2 and len(amcl_times) >= 2:
            odom_interp = np.interp(amcl_times, odom_times, odom_yaws)
            yaw_diff = np.array([self.normalize_angle(a - o) for a, o in zip(amcl_yaws, odom_interp)])
        else:
            yaw_diff = np.array([])

        # プロット更新（rad表記）
        self.ax1.clear()
        self.ax1.plot(odom_times, odom_yaws, 'b-', label='/Odometry', linewidth=1.5)
        self.ax1.plot(amcl_times, amcl_yaws, 'r-', label='/amcl_pose', linewidth=1.5)
        self.ax1.set_xlabel('Time [s]')
        self.ax1.set_ylabel('Yaw [rad]')
        self.ax1.set_title('Yaw Angle')
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True, alpha=0.3)

        self.ax2.clear()
        if len(yaw_diff) > 0:
            self.ax2.plot(amcl_times, yaw_diff, 'g-', linewidth=1.5)
            self.ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)

            # 統計情報（rad）
            mean_diff = np.mean(yaw_diff)
            std_diff = np.std(yaw_diff)
            self.ax2.axhline(y=mean_diff, color='r', linestyle='--', alpha=0.7,
                           label=f'Mean: {mean_diff:.4f} rad ({np.degrees(mean_diff):.2f} deg)')
            self.ax2.fill_between(amcl_times, mean_diff - std_diff, mean_diff + std_diff, alpha=0.2, color='r')
            self.ax2.legend(loc='upper right')

        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('yaw_offset (amcl - odom) [rad]')
        self.ax2.set_title('Yaw Offset for gps_localization.py')
        self.ax2.grid(True, alpha=0.3)

        self.fig.tight_layout()
        plt.pause(0.01)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

        plt.ioff()
        plt.show()


if __name__ == '__main__':
    try:
        plotter = YawComparisonPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
