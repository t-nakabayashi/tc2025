#!/usr/bin/env python3
# coding=utf-8

"""
シンプルなGPS+Odometry方位ローカライゼーション

処理内容:
1. /Odometryのヨー角に事前計算した偏差を加算してロボットの方位とする
2. /ublox/fixをUTM座標に変換してロボットの位置とする
3. 上記を/amcl_poseとして出力
"""

import rospy
import math
import tf
import yaml
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

# UTM基準点（つくばチャレンジ用）
UTM_BASE_LAT = 36.0830041
UTM_BASE_LON = 140.0763757

# inagi
#UTM_BASE_LAT = 35.649727399999996
#UTM_BASE_LON = 139.5025503


class GPSLocalization:
    def __init__(self):
        rospy.init_node('gps_localization', anonymous=False)

        # yaw_offsetファイルのパス
        self.yaw_offset_file = rospy.get_param('~yaw_offset_file',
            '/home/nkb/catkin_ws/src/tc2025/config/yaw_offset.yaml')

        # ヨー角偏差（amcl_pose - Odometry）[rad]
        # パラメータ指定があればそれを使用、なければファイルから読み込み
        param_offset = rospy.get_param('~yaw_offset', None)
        if param_offset is not None:
            self.yaw_offset = param_offset
        else:
            self.yaw_offset = self._load_yaw_offset()

        # 現在の状態
        self.current_utm_x = 0.0
        self.current_utm_y = 0.0
        self.current_yaw = 0.0
        self.gps_received = False
        self.odom_received = False

        # Publisher
        self.amcl_pose_pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10)

        # TFブロードキャスター
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Subscriber
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)

        # パブリッシュレート
        self.publish_rate = rospy.Rate(50)

        rospy.loginfo("GPS Localization Node initialized")
        rospy.loginfo(f"Base point: lat={UTM_BASE_LAT}, lon={UTM_BASE_LON}")
        rospy.loginfo(f"Yaw offset file: {self.yaw_offset_file}")
        rospy.loginfo(f"Yaw offset: {math.degrees(self.yaw_offset):.2f} deg ({self.yaw_offset:.4f} rad)")

    def _load_yaw_offset(self):
        """yaw_offset.yamlからオフセットを読み込む"""
        if os.path.exists(self.yaw_offset_file):
            try:
                with open(self.yaw_offset_file, 'r') as f:
                    data = yaml.safe_load(f)
                    offset = data.get('yaw_offset', 0.0)
                    rospy.loginfo(f"Loaded yaw_offset from file: {offset:.4f} rad")
                    return offset
            except Exception as e:
                rospy.logwarn(f"Failed to load yaw_offset file: {e}")
                return 0.0
        else:
            rospy.logwarn(f"yaw_offset file not found: {self.yaw_offset_file}")
            return 0.0

    def wrap_angle(self, a):
        """角度を-πからπの範囲に正規化"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def gps_callback(self, msg):
        """GPS情報を受信してUTM座標に変換"""
        utm_x, utm_y = self.latlon_to_utm(
            msg.latitude,
            msg.longitude,
            UTM_BASE_LAT,
            UTM_BASE_LON
        )
        self.current_utm_x = utm_x
        self.current_utm_y = utm_y
        self.gps_received = True

    def odom_callback(self, msg):
        """Odometryから方位を取得し、偏差を加算"""
        q = msg.pose.pose.orientation
        _, _, odom_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 偏差を加算
        self.current_yaw = self.wrap_angle(odom_yaw + self.yaw_offset)
        self.odom_received = True

    def latlon_to_utm(self, latitude, longitude, base_lat, base_lon):
        """緯度経度をUTM座標（メートル）に変換"""
        R = 6378137.0  # 地球の平均半径

        dlat = math.radians(latitude - base_lat)
        dlon = math.radians(longitude - base_lon)
        base_lat_rad = math.radians(base_lat)

        utm_x = R * dlon * math.cos(base_lat_rad)
        utm_y = R * dlat

        return utm_x, utm_y

    def publish_pose_and_tf(self):
        """amcl_poseとTFをパブリッシュ"""
        while not rospy.is_shutdown():
            if not (self.gps_received and self.odom_received):
                self.publish_rate.sleep()
                continue

            # クォータニオンに変換
            quat = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
            current_time = rospy.Time.now()

            # /amcl_poseメッセージを作成
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "map"

            pose_msg.pose.pose.position.x = self.current_utm_x
            pose_msg.pose.pose.position.y = self.current_utm_y
            pose_msg.pose.pose.position.z = 0.0

            pose_msg.pose.pose.orientation.x = quat[0]
            pose_msg.pose.pose.orientation.y = quat[1]
            pose_msg.pose.pose.orientation.z = quat[2]
            pose_msg.pose.pose.orientation.w = quat[3]

            # 共分散行列
            covariance = [0.0] * 36
            covariance[0] = 4.0    # x
            covariance[7] = 4.0    # y
            covariance[14] = 0.01  # z
            covariance[21] = 0.01  # roll
            covariance[28] = 0.01  # pitch
            covariance[35] = 0.25  # yaw
            pose_msg.pose.covariance = covariance

            # パブリッシュ
            self.amcl_pose_pub.publish(pose_msg)

            # TFをブロードキャスト（map -> base_link）
            self.tf_broadcaster.sendTransform(
                (self.current_utm_x, self.current_utm_y, 0.0),
                quat,
                current_time,
                "base_link",
                "map"
            )

            self.publish_rate.sleep()

    def run(self):
        """メインループ"""
        try:
            self.publish_pose_and_tf()
        except rospy.ROSInterruptException:
            rospy.loginfo("GPS Localization Node shutting down")


if __name__ == '__main__':
    try:
        node = GPSLocalization()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()
