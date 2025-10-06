#!/usr/bin/env python3
"""
YP-Spur制御ロボットをジョイスティックで操作するためのROS1ノード。

sensor_msgs/Joyメッセージ（joy_nodeなどが発行）を受け取り、YP-Spurが購読
する/cmd_velへTwistを送信する。軸・ボタンやスケールはROSパラメータで
調整可能で、PS2コントローラのように「上＝前進」となるよう反転設定も
備える。またボタン0〜3を押した瞬間に対応するウェイポイントフラグ
（Int32MultiArray）を発行する。
"""

import math
from typing import List

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy


class JoystickTeleop:
    def __init__(self) -> None:
        # 軸番号とスケールのパラメータ取得
        self.linear_axis = rospy.get_param("~linear_axis", 1)  # 左スティック縦方向
        self.angular_axis = rospy.get_param("~angular_axis", 0)  # 左スティック横方向
        self.linear_y_axis = rospy.get_param("~linear_y_axis", -1)  # 横移動（未使用なら-1）
        self.linear_scale = rospy.get_param("~linear_scale", 0.7)
        self.angular_scale = rospy.get_param("~angular_scale", 1.5)
        self.linear_y_scale = rospy.get_param("~linear_y_scale", 0.5)
        self.deadzone = rospy.get_param("~deadzone", 0.05)
        self.linear_axis_invert = rospy.get_param("~linear_axis_invert", True)  # 上＝前進となるように反転
        self.angular_axis_invert = rospy.get_param("~angular_axis_invert", False)  # 左右を反転させる場合はTrue

        # デッドマンボタンとターボ関連パラメータ
        self.enable_button = rospy.get_param("~enable_button", -1)  # PSコントローラではL1=4など
        self.turbo_button = rospy.get_param("~turbo_button", -1)  # ブーストに使うボタン
        self.turbo_ratio = rospy.get_param("~turbo_ratio", 1.5)

        # 入力が止まった際の自動停止タイマー設定
        self.timeout = rospy.get_param("~timeout", 0.5)
        publish_rate = rospy.get_param("~publish_rate", 20.0)  # Hz
        self.publish_period = rospy.Duration(1.0 / max(publish_rate, 1.0))

        self.cmd_vel_pub = rospy.Publisher("ypspur_ros/cmd_vel", Twist, queue_size=1)
        self.last_cmd = Twist()
        self.last_input_time = rospy.Time.now()
        self.prev_buttons = []  # ボタン状態の上昇エッジ検出用

        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        rospy.Timer(self.publish_period, self.publish_callback)

        # ウェイポイントの押下フラグを通知するトピック
        self.waypoint_flag_pub = rospy.Publisher("waypoint_flag", Int32MultiArray, queue_size=1)

        rospy.loginfo("joystick_teleop node started. Waiting for Joy messages...")

    def joy_callback(self, msg: Joy) -> None:
        now = rospy.Time.now()

        # 0〜3番ボタンの上昇エッジを先に処理（デッドマン無効でも通知したい）
        self._publish_waypoint_flag(msg.buttons)

        if self.enable_button >= 0:
            if self.enable_button >= len(msg.buttons) or msg.buttons[self.enable_button] == 0:
                self._stop()
                self.last_input_time = now
                return

        # 軸値を読み出してTwistを生成
        linear_x = self._read_axis(msg.axes, self.linear_axis, self.linear_scale)
        if self.linear_axis_invert:
            linear_x *= -1.0
        angular_z = self._read_axis(msg.axes, self.angular_axis, self.angular_scale)
        if self.angular_axis_invert:
            angular_z *= -1.0
        linear_y = 0.0
        if self.linear_y_axis >= 0:
            linear_y = self._read_axis(msg.axes, self.linear_y_axis, self.linear_y_scale)

        # ターボボタンが押されていればスケール倍率を適用
        if self.turbo_button >= 0 and self.turbo_button < len(msg.buttons) and msg.buttons[self.turbo_button]:
            linear_x *= self.turbo_ratio
            angular_z *= self.turbo_ratio
            linear_y *= self.turbo_ratio

        self.last_cmd.linear.x = linear_x
        self.last_cmd.linear.y = linear_y
        self.last_cmd.angular.z = angular_z
        self.last_input_time = now

    def publish_callback(self, _event) -> None:
        # タイムアウト超過時は安全のため停止
        if rospy.Time.now() - self.last_input_time > rospy.Duration(self.timeout):
            if not self._is_zero_twist(self.last_cmd):
                self._stop()
        self.cmd_vel_pub.publish(self.last_cmd)

    def _read_axis(self, axes: List[float], index: int, scale: float) -> float:
        if index < 0 or index >= len(axes):
            return 0.0
        value = axes[index]
        if math.fabs(value) < self.deadzone:
            return 0.0
        return value * scale

    def _stop(self) -> None:
        self.last_cmd.linear.x = 0.0
        self.last_cmd.linear.y = 0.0
        self.last_cmd.linear.z = 0.0
        self.last_cmd.angular.x = 0.0
        self.last_cmd.angular.y = 0.0
        self.last_cmd.angular.z = 0.0

    def _publish_waypoint_flag(self, buttons: List[int]) -> None:
        if not self.prev_buttons:
            self.prev_buttons = list(buttons)
            return

        limit = min(4, len(buttons))
        for idx in range(limit):
            prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
            current = buttons[idx]
            if prev == 0 and current == 1:
                msg = Int32MultiArray()
                msg.data = [1 if i == idx else 0 for i in range(limit)]
                self.waypoint_flag_pub.publish(msg)
        self.prev_buttons = list(buttons)

    @staticmethod
    def _is_zero_twist(twist: Twist) -> bool:
        return (
            twist.linear.x == 0.0
            and twist.linear.y == 0.0
            and twist.linear.z == 0.0
            and twist.angular.x == 0.0
            and twist.angular.y == 0.0
            and twist.angular.z == 0.0
        )


def main() -> None:
    rospy.init_node("joystick_teleop")
    JoystickTeleop()
    rospy.spin()


if __name__ == "__main__":
    main()
