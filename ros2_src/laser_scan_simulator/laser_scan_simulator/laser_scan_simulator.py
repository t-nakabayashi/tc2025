#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LaserScan Simulator Node (v3.9 fix for ROS2 Foxy / Python3.8)

- Python 3.8互換版: 型ヒントで `|` を使わず Optional[List[str]] を使用。
- ROS(REP-103) → 画像座標で +90°回転（位置・レイ方向両方に適用）し、
  pose移動とスキャン出力がマップ上の表示と完全一致する。
"""

import math
import threading
from typing import List, Tuple, Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LaserScanSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__('laser_scan_simulator')

        # パラメータ
        self.declare_parameter('map_image_path', '/tmp/map.bmp')
        self.declare_parameter('map_resolution_m', 0.2)
        self.declare_parameter('publish_rate_hz', 40.0)
        self.declare_parameter('enable_debug_view', True)
        self.declare_parameter('debug_view_rate_hz', 10.0)
        self.declare_parameter('angle_min_deg', -135.0)
        self.declare_parameter('angle_max_deg', 135.0)
        self.declare_parameter('angle_increment_deg', 0.25)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 30.0)

        self.map_path = self.get_parameter('map_image_path').value
        self.map_res = float(self.get_parameter('map_resolution_m').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.enable_debug = bool(self.get_parameter('enable_debug_view').value)
        self.debug_rate_hz = float(self.get_parameter('debug_view_rate_hz').value)
        self.angle_min = math.radians(float(self.get_parameter('angle_min_deg').value))
        self.angle_max = math.radians(float(self.get_parameter('angle_max_deg').value))
        self.angle_inc = math.radians(float(self.get_parameter('angle_increment_deg').value))
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        # マップ読み込み
        self.map_img = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            self.get_logger().error(f"マップ画像の読み込みに失敗しました: {self.map_path}")
            raise FileNotFoundError(self.map_path)
        self.map_h, self.map_w = self.map_img.shape[:2]

        # 初期姿勢
        self.map_origin_x = (self.map_w * self.map_res) / 2.0
        self.map_origin_y = self.map_res / 2.0
        self.pose_x = self.map_origin_x
        self.pose_y = self.map_origin_y
        self.pose_yaw = 0.0
        self.init_pose_set = False
        self.init_pose_x = 0.0
        self.init_pose_y = 0.0
        self.init_pose_yaw = 0.0

        self._lock = threading.Lock()
        self.last_scan_angles = np.array([], dtype=np.float32)
        self.last_scan_ranges = np.array([], dtype=np.float32)

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)

        self.create_timer(1.0 / self.publish_rate_hz, self.on_timer_scan)
        if self.enable_debug:
            self.create_timer(1.0 / self.debug_rate_hz, self.on_timer_debug)

        self.get_logger().info('LaserScanSimulatorNode (v3.9, Python3.8対応) 初期化完了。')

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        # PoseWithCovarianceStampedをPoseStampedに変換
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        x = float(pose_stamped.pose.position.x)
        y = float(pose_stamped.pose.position.y)
        q = pose_stamped.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        if not self.init_pose_set:
            self.init_pose_x = x
            self.init_pose_y = y
            self.init_pose_yaw = yaw
            self.init_pose_set = True
            self.get_logger().info('初期姿勢を登録（画像下端中央に対応付け）')

        dx = x - self.init_pose_x
        dy = y - self.init_pose_y
        c = math.cos(-self.init_pose_yaw)
        s = math.sin(-self.init_pose_yaw)
        rel_x = dx * c - dy * s
        rel_y = dx * s + dy * c
        rel_yaw = yaw - self.init_pose_yaw

        # 位置変換: ROS→画像 (+90°回転)
        pose_x = self.map_origin_x - rel_y
        pose_y = self.map_origin_y + rel_x

        with self._lock:
            self.pose_x = pose_x
            self.pose_y = pose_y
            self.pose_yaw = rel_yaw

    def world_to_image(self, x_m: float, y_m: float) -> Tuple[int, int]:
        u = int(round(x_m / self.map_res))
        v = int(round(self.map_h - 1 - (y_m / self.map_res)))
        return u, v

    def cast_ray(self, u0: int, v0: int, dir_u: float, dir_v: float) -> float:
        step_px = 0.5
        max_px = self.range_max / self.map_res
        t = 0.0
        while t <= max_px:
            u = int(round(u0 + t * dir_u))
            v = int(round(v0 + t * dir_v))
            if not (0 <= u < self.map_w and 0 <= v < self.map_h):
                return t * self.map_res
            if self.map_img[v, u] < 128:
                return t * self.map_res
            t += step_px
        return self.range_max

    def on_timer_scan(self) -> None:
        with self._lock:
            x, y, yaw = self.pose_x, self.pose_y, self.pose_yaw

        u0, v0 = self.world_to_image(x, y)
        angles = np.arange(self.angle_min, self.angle_max + self.angle_inc / 2.0, self.angle_inc, dtype=np.float32)
        ranges = np.empty_like(angles, dtype=np.float32)

        for i, theta in enumerate(angles):
            beam_ang_world = yaw + theta
            # ROS→画像 (+90°回転)
            dir_u = -math.sin(beam_ang_world)
            dir_v = -math.cos(beam_ang_world)
            dist = self.cast_ray(u0, v0, dir_u, dir_v)
            ranges[i] = max(min(dist, self.range_max), self.range_min)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.pub_scan.publish(scan)

        with self._lock:
            self.last_scan_angles = np.copy(angles)
            self.last_scan_ranges = np.copy(ranges)

    def on_timer_debug(self) -> None:
        img_color = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        with self._lock:
            x, y, yaw = self.pose_x, self.pose_y, self.pose_yaw
            angles = np.copy(self.last_scan_angles)
            ranges = np.copy(self.last_scan_ranges)

        u0, v0 = self.world_to_image(x, y)

        overlay = img_color.copy()
        radius_px = int(self.range_max / self.map_res)
        robot_angle_deg = -math.degrees(yaw) - 90.0
        start_sweep = math.degrees(-self.angle_max)
        end_sweep = math.degrees(-self.angle_min)
        cv2.ellipse(overlay, (u0, v0), (radius_px, radius_px), robot_angle_deg, start_sweep, end_sweep, (255, 200, 100), -1)
        cv2.addWeighted(overlay, 0.25, img_color, 0.75, 0.0, img_color)

        cv2.circle(img_color, (u0, v0), 3, (0, 0, 255), -1)
        arrow_u = int(u0 - 10.0 * math.sin(yaw))
        arrow_v = int(v0 - 10.0 * math.cos(yaw))
        cv2.line(img_color, (u0, v0), (arrow_u, arrow_v), (0, 0, 255), 2)

        if angles.size > 0 and ranges.size == angles.size:
            for theta, dist in zip(angles, ranges):
                beam_ang_world = yaw + theta
                du = int((dist / self.map_res) * -math.sin(beam_ang_world))
                dv = int((dist / self.map_res) * -math.cos(beam_ang_world))
                end_pt = (u0 + du, v0 + dv)
                cv2.line(img_color, (u0, v0), end_pt, (180, 180, 180), 1)
                cv2.circle(img_color, end_pt, 2, (0, 0, 255), -1)

        cv2.imshow('LaserScan Simulator Debug', img_color)
        cv2.waitKey(1)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LaserScanSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
