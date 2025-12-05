#!/usr/bin/env python3
# coding=utf-8

import rospy
import math
import tkinter as tk
from tkinter import ttk
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
import threading

# UTM基準点（つくばチャレンジ用）
UTM_BASE_LAT = 36.0830041
UTM_BASE_LON = 140.0763757
UTM_BASE_ALT = 73.568

class GPSPoseEstimator:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('gps_pose_estimator', anonymous=False)

        # GPS情報を格納する変数
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_altitude = 0.0
        self.gps_lock = threading.Lock()
        self.gps_received = False

        # 2D Pose Estimateのパブリッシャー
        self.pose_pub = rospy.Publisher(
            '/initialpose',
            PoseWithCovarianceStamped,
            queue_size=10
        )

        # GPS情報のサブスクライバー
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)

        rospy.loginfo("GPS Pose Estimator initialized")
        rospy.loginfo(f"Base point: lat={UTM_BASE_LAT}, lon={UTM_BASE_LON}, alt={UTM_BASE_ALT}")

    def gps_callback(self, msg):
        """GPS情報を受信して保存"""
        with self.gps_lock:
            self.current_latitude = msg.latitude
            self.current_longitude = msg.longitude
            self.current_altitude = msg.altitude
            self.gps_received = True

    def latlon_to_utm(self, latitude, longitude, base_lat, base_lon):
        """
        緯度経度をUTM座標（メートル）に変換する簡易関数
        基準点からの相対位置を計算
        """
        # 地球の平均半径（メートル）
        R = 6378137.0

        # 緯度経度の差をラジアンに変換
        dlat = math.radians(latitude - base_lat)
        dlon = math.radians(longitude - base_lon)

        # 基準点の緯度をラジアンに変換
        base_lat_rad = math.radians(base_lat)

        # UTM座標に変換（メートル単位）
        utm_x = R * dlon * math.cos(base_lat_rad)
        utm_y = R * dlat

        return utm_x, utm_y

    def publish_pose_estimate(self, x, y, theta, spread_mode=False):
        """
        2D Pose Estimateをパブリッシュ
        spread_mode=Trueの場合、複数の方向にパーティクルを散布
        """
        if spread_mode:
            # 全方向にパーティクルを散布（8方向）
            num_directions = 8
            for i in range(num_directions):
                angle = theta + (2 * math.pi * i / num_directions)
                self._publish_single_pose(x, y, angle)
                rospy.sleep(0.05)  # 少し待機
        else:
            # 単一の姿勢を送信
            self._publish_single_pose(x, y, theta)

    def _publish_single_pose(self, x, y, theta):
        """単一の2D Pose Estimateをパブリッシュ"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        # 位置を設定
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        # クォータニオンに変換
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        # 共分散行列を設定（位置と方向の不確実性）
        covariance = [0.0] * 36
        covariance[0] = 0.5   # x-x
        covariance[7] = 0.5   # y-y
        covariance[35] = 0.1  # yaw-yaw
        pose_msg.pose.covariance = covariance

        # パブリッシュ
        self.pose_pub.publish(pose_msg)
        rospy.loginfo(f"Published pose: x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.2f}°")

    def get_current_utm_pose(self):
        """現在のGPS位置からUTM座標を取得"""
        with self.gps_lock:
            if not self.gps_received:
                return None, None, None, None

            lat = self.current_latitude
            lon = self.current_longitude
            alt = self.current_altitude

        # UTM座標に変換
        utm_x, utm_y = self.latlon_to_utm(lat, lon, UTM_BASE_LAT, UTM_BASE_LON)

        return utm_x, utm_y, lat, lon


class GPSPoseEstimatorGUI:
    def __init__(self, estimator):
        self.estimator = estimator

        # メインウィンドウの作成
        self.root = tk.Tk()
        self.root.title("GPS Pose Estimator")
        self.root.geometry("500x400")

        # スタイルの設定
        style = ttk.Style()
        style.theme_use('clam')

        # メインフレーム
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # タイトル
        title_label = ttk.Label(
            main_frame,
            text="GPS-Based 2D Pose Estimator",
            font=('Arial', 16, 'bold')
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=10)

        # GPS情報表示エリア
        info_frame = ttk.LabelFrame(main_frame, text="Current GPS Information", padding="10")
        info_frame.grid(row=1, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))

        # GPS座標表示
        self.lat_label = ttk.Label(info_frame, text="Latitude: --", font=('Arial', 10))
        self.lat_label.grid(row=0, column=0, sticky=tk.W, pady=2)

        self.lon_label = ttk.Label(info_frame, text="Longitude: --", font=('Arial', 10))
        self.lon_label.grid(row=1, column=0, sticky=tk.W, pady=2)

        self.utm_x_label = ttk.Label(info_frame, text="UTM X: --", font=('Arial', 10))
        self.utm_x_label.grid(row=2, column=0, sticky=tk.W, pady=2)

        self.utm_y_label = ttk.Label(info_frame, text="UTM Y: --", font=('Arial', 10))
        self.utm_y_label.grid(row=3, column=0, sticky=tk.W, pady=2)

        # 方向設定エリア
        direction_frame = ttk.LabelFrame(main_frame, text="Direction Settings", padding="10")
        direction_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))

        # 方向入力
        ttk.Label(direction_frame, text="Direction (degrees):").grid(row=0, column=0, sticky=tk.W)
        self.direction_var = tk.DoubleVar(value=0.0)
        self.direction_entry = ttk.Entry(direction_frame, textvariable=self.direction_var, width=10)
        self.direction_entry.grid(row=0, column=1, padx=5)

        # 散布モードチェックボックス
        self.spread_var = tk.BooleanVar(value=True)
        self.spread_check = ttk.Checkbutton(
            direction_frame,
            text="Spread particles in all directions (8-way)",
            variable=self.spread_var
        )
        self.spread_check.grid(row=1, column=0, columnspan=2, pady=5, sticky=tk.W)

        # ボタンエリア
        button_frame = ttk.Frame(main_frame, padding="10")
        button_frame.grid(row=3, column=0, columnspan=2, pady=10)

        # 更新ボタン
        self.update_button = ttk.Button(
            button_frame,
            text="Update GPS Info",
            command=self.update_gps_info
        )
        self.update_button.grid(row=0, column=0, padx=5)

        # Pose Estimateボタン
        self.estimate_button = ttk.Button(
            button_frame,
            text="Publish 2D Pose Estimate",
            command=self.publish_pose,
            style='Accent.TButton'
        )
        self.estimate_button.grid(row=0, column=1, padx=5)

        # スタイル設定
        style.configure('Accent.TButton', foreground='blue', font=('Arial', 10, 'bold'))

        # ステータスバー
        self.status_label = ttk.Label(
            main_frame,
            text="Ready",
            relief=tk.SUNKEN,
            anchor=tk.W
        )
        self.status_label.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        # 自動更新タイマー（1秒ごと）
        self.auto_update()

    def auto_update(self):
        """GPS情報を自動更新"""
        self.update_gps_info()
        self.root.after(1000, self.auto_update)  # 1秒後に再度実行

    def update_gps_info(self):
        """GPS情報を表示"""
        utm_x, utm_y, lat, lon = self.estimator.get_current_utm_pose()

        if utm_x is not None:
            self.lat_label.config(text=f"Latitude: {lat:.7f}°")
            self.lon_label.config(text=f"Longitude: {lon:.7f}°")
            self.utm_x_label.config(text=f"UTM X: {utm_x:.2f} m")
            self.utm_y_label.config(text=f"UTM Y: {utm_y:.2f} m")
            self.status_label.config(text="GPS signal received")
        else:
            self.lat_label.config(text="Latitude: --")
            self.lon_label.config(text="Longitude: --")
            self.utm_x_label.config(text="UTM X: --")
            self.utm_y_label.config(text="UTM Y: --")
            self.status_label.config(text="Waiting for GPS signal...")

    def publish_pose(self):
        """2D Pose Estimateをパブリッシュ"""
        utm_x, utm_y, lat, lon = self.estimator.get_current_utm_pose()

        if utm_x is None:
            self.status_label.config(text="ERROR: No GPS signal available!")
            return

        # 方向を取得（度からラジアンに変換）
        direction_deg = self.direction_var.get()
        direction_rad = math.radians(direction_deg)

        # 散布モードを取得
        spread_mode = self.spread_var.get()

        # Pose Estimateをパブリッシュ
        self.estimator.publish_pose_estimate(utm_x, utm_y, direction_rad, spread_mode)

        if spread_mode:
            self.status_label.config(
                text=f"Published pose with particles spread in all directions at ({utm_x:.2f}, {utm_y:.2f})"
            )
        else:
            self.status_label.config(
                text=f"Published pose at ({utm_x:.2f}, {utm_y:.2f}), direction={direction_deg:.1f}°"
            )

    def run(self):
        """GUIを実行"""
        self.root.mainloop()


def main():
    try:
        # ROSノードとGUIを別スレッドで実行
        estimator = GPSPoseEstimator()

        # ROS spinを別スレッドで実行
        ros_thread = threading.Thread(target=rospy.spin)
        ros_thread.daemon = True
        ros_thread.start()

        # GUIを実行（メインスレッド）
        gui = GPSPoseEstimatorGUI(estimator)
        gui.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
