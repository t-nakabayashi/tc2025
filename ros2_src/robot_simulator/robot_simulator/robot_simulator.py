#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot_simulator ノード (Phase2 拡張版).

/active_target を購読してロボットの等速直線運動をシミュレートし、
/amcl_pose を周期配信する。/scan_livox_front_low_move による前方障害物検知を追加。

主な仕様:
- 等速直線運動 (5km/h)・100ms周期
- 初回 active_target 受信時、目標の背後 5m から開始
- 障害物距離に応じて減速・停止
- QoS: RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)
"""

import math
import random
from dataclasses import dataclass
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = yaw * 0.5
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class RobotSimulatorNode(Node):
    """/active_target 追従 + 前方障害物停止付きロボットシミュレータ."""

    def __init__(self) -> None:
        super().__init__('robot_simulator')

        # パラメータ宣言
        self.declare_parameter('speed_kmph', 5.0)
        self.declare_parameter('timer_period_ms', 20)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('init_offset_m', 5.0)
        self.declare_parameter('stop_radius_m', 1.0)
        self.declare_parameter('noise_pos_std_m', 0.0)
        self.declare_parameter('noise_yaw_std_deg', 0.0)
        self.declare_parameter('log_debug', False)

        # Phase2: 障害物関連
        self.declare_parameter('robot_width', 0.6)
        self.declare_parameter('max_detection_distance', 5.0)
        self.declare_parameter('safety_distance', 1.0)
        self.declare_parameter('min_obstacle_distance', 0.8)
        self.declare_parameter('max_decel_mps2', 1.0)
        self.declare_parameter('enable_obstacle_stop', True)
        self.declare_parameter('rotation_speed_degps', 30.0)  # 障害物時の回頭速度[deg/s]

        # パラメータ取得
        p = self.get_parameter
        self.v_mps = p('speed_kmph').value * 1000.0 / 3600.0
        self.dt = p('timer_period_ms').value / 1000.0
        self.frame_id = p('frame_id').value
        self.child_frame_id = p('child_frame_id').value
        self.publish_tf = p('publish_tf').value
        self.init_offset_m = p('init_offset_m').value
        self.stop_radius_m = p('stop_radius_m').value
        self.noise_pos_std_m = p('noise_pos_std_m').value
        self.noise_yaw_std_deg = p('noise_yaw_std_deg').value
        self.log_debug = p('log_debug').value
        self.robot_width = p('robot_width').value
        self.max_detection_distance = p('max_detection_distance').value
        self.safety_distance = p('safety_distance').value
        self.min_obstacle_distance = p('min_obstacle_distance').value
        self.max_decel = p('max_decel_mps2').value
        self.enable_obstacle_stop = p('enable_obstacle_stop').value
        self.rotation_speed = math.radians(self.get_parameter('rotation_speed_degps').value)

        if self.v_mps <= 0:
            raise ValueError('speed_kmph must be positive.')

        # 内部状態
        self.pose: Optional[Pose2D] = None
        self.have_target = False
        self.target_pose: Optional[Pose2D] = None
        self.target_changed = False
        self.obstacle_distance: Optional[float] = None

        # QoS設定
        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,  # ← VOLATILEに変更
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # 余裕を持たせて10に設定
        )

        # 通信設定
        self.create_subscription(PoseStamped, '/active_target', self._on_target, target_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 10) if self.publish_tf else None

        self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(f'robot_simulator phase2 起動: v={self.v_mps:.3f} m/s, 障害物対応=ON')

    def _on_target(self, msg: PoseStamped) -> None:
        self.get_logger().info(f'_on_target')
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(f'frame_id mismatch: {msg.header.frame_id}')
        tx, ty = msg.pose.position.x, msg.pose.position.y
        tyaw = quat_to_yaw(msg.pose.orientation)
        self.target_pose = Pose2D(tx, ty, tyaw)
        self.target_changed = True
        if not self.have_target:
            vx, vy = math.cos(tyaw), math.sin(tyaw)
            px, py = tx - self.init_offset_m * vx, ty - self.init_offset_m * vy
            yaw0 = math.atan2(ty - py, tx - px)
            self.pose = Pose2D(px, py, yaw0)
            self.have_target = True
            self.get_logger().info(f'初期化完了: start=({px:.2f},{py:.2f})')

    def _on_scan(self, msg: LaserScan) -> None:
        half_width = self.robot_width / 2.0
        x_min = None
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.2:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(math.degrees(angle)) > 60.0:
                continue
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            if 0 < x <= self.max_detection_distance and -half_width <= y <= half_width:
                if x_min is None or x < x_min:
                    x_min = x
        self.obstacle_distance = x_min

    def _on_timer(self) -> None:
        if not self.have_target or self.pose is None or self.target_pose is None:
            return

        px, py, yaw = self.pose.x, self.pose.y, self.pose.yaw
        tx, ty = self.target_pose.x, self.target_pose.y
        dx, dy = tx - px, ty - py
        d = math.hypot(dx, dy)
        if d > 1e-9:
            ux, uy = dx / d, dy / d
        else:
            ux, uy = 0.0, 0.0
        self.get_logger().info(f'Distance: {d:.2f}m')

        step = self.v_mps * self.dt
        cand_x, cand_y = px + step * ux, py + step * uy

        # 極小解停止
        if not self.target_changed and d <= self.stop_radius_m and math.hypot(tx - cand_x, ty - cand_y) > d:
            cand_x, cand_y = px, py

        # 障害物減速・停止
        if self.enable_obstacle_stop and self.obstacle_distance is not None:
            #stop_dist = (self.v_mps ** 2) / (2 * self.max_decel) + self.min_obstacle_distance
            stop_dist = self.min_obstacle_distance
            if self.obstacle_distance < stop_dist:
                # 並進停止
                cand_x, cand_y = px, py

                # 目標方向へその場旋回
                target_yaw = math.atan2(ty - py, tx - px)
                yaw_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi  # -π～πに正規化
                max_yaw_change = self.rotation_speed * self.dt
                if abs(yaw_diff) < max_yaw_change:
                    new_yaw = target_yaw
                else:
                    new_yaw = yaw + math.copysign(max_yaw_change, yaw_diff)

                if self.log_debug:
                    self.get_logger().warn(
                        f'障害物停止: {self.obstacle_distance:.2f}m, 旋回中 (yaw→{math.degrees(new_yaw):.1f}°)'
                    )
            elif self.obstacle_distance < self.safety_distance:
                scale = (self.obstacle_distance - self.min_obstacle_distance) / (
                    self.safety_distance - self.min_obstacle_distance
                )
                step *= max(0.0, scale)
                cand_x, cand_y = px + step * ux, py + step * uy

        new_yaw = math.atan2(uy, ux) if (ux or uy) else yaw
        out_x = cand_x + random.gauss(0, self.noise_pos_std_m) if self.noise_pos_std_m > 0 else cand_x
        out_y = cand_y + random.gauss(0, self.noise_pos_std_m) if self.noise_pos_std_m > 0 else cand_y
        out_yaw = new_yaw + math.radians(random.gauss(0, self.noise_yaw_std_deg)) if self.noise_yaw_std_deg > 0 else new_yaw

        self.pose = Pose2D(cand_x, cand_y, new_yaw)
        self._publish_amcl_pose(out_x, out_y, out_yaw)
        if self.publish_tf and self.pub_tf:
            self._publish_tf(out_x, out_y, out_yaw)

        self.target_changed = False

    def _publish_amcl_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        self.pub_pose.publish(msg)

    def _publish_tf(self, x: float, y: float, yaw: float) -> None:
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.frame_id
        ts.child_frame_id = self.child_frame_id
        ts.transform.translation.x = x
        ts.transform.translation.y = y
        ts.transform.rotation = yaw_to_quat(yaw)
        msg = TFMessage(transforms=[ts])
        self.pub_tf.publish(msg)


def main() -> None:
    rclpy.init()
    node = RobotSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
