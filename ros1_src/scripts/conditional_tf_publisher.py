#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf2_ros

class ConditionalTFPublisher:
    def __init__(self):
        # 初期化
        rospy.init_node('conditional_tf_publisher')

        # オドメトリトピック名をパラメータから取得（デフォルトは /ypspur_ros/odom）
        odom_topic = rospy.get_param('~odom_topic', '/ypspur_ros/odom')

        # 指定されたオドメトリトピックを購読
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # 10Hzでパブリッシュするためのタイマー
        self.publish_rate = rospy.Rate(10)

        # TFブロードキャスターを作成
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # オドメトリデータを保持する変数
        self.odom_data = None

    def odom_callback(self, msg):
        """オドメトリトピックからデータを取得"""
        self.odom_data = msg

    def publish_tf(self):
        """オドメトリデータを基にTFをパブリッシュ"""
        # TransformStampedメッセージを作成
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        if self.odom_data:
            # オドメトリデータがある場合はその値を使用
            odom = self.odom_data
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = odom.pose.pose.orientation
        else:
            # オドメトリデータがない場合は原点にTFをパブリッシュ
            # （odomフレームを存在させるため）
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0

        # TFをパブリッシュ
        self.tf_broadcaster.sendTransform(transform)

    def run(self):
        """メインループ"""
        while not rospy.is_shutdown():
            self.publish_tf()
            self.publish_rate.sleep()

if __name__ == '__main__':
    try:
        node = ConditionalTFPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
