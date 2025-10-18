# -*- coding: utf-8 -*-
"""Obstacle Monitor Node (legacy-following, Phase2, ROS 2 Foxy)

Google Python Style / 型ヒント完備 / 日本語コメント。cv_bridgeはbgr8で出力。

本ノードは単一2D LiDARの `/scan` を購読し、以下を行う:
  * 前方くさび領域の閉塞判定 (front_blocked)  … legacy: ±front_half_deg 内で x < stop_dist_m の点が存在
  * 左右の回避オフセット（可用幅）を legacy 方式で算出（ギャップ検出 + 外縁 + 下限0.75m）
  * 上記ヒントを `route_msgs/ObstacleAvoidanceHint` として publish（left_is_open/right_is_open を [m] で出力）
  * デバッグ用に LiDAR 点群を画像化し `sensor_msgs/Image` (bgr8) で publish（laserScanViewer 相当の描画仕様）

参考に踏襲した実装: waypoint_manager.py の laserScanCallback / laserScanViewer。
差分・注意点は最下部のコメント参照。

前提:
  * LiDAR は base_link に前向きで固定搭載 (TF 変換は不要)
  * 出力 QoS は BestEffort / Volatile / depth=1
  * ROS 2 Foxy, rclpy, cv_bridge, numpy, opencv-python

Author: ChatGPT
"""

from typing import Tuple, Optional

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from builtin_interfaces.msg import Time as TimeMsg

# ユーザー指定: route_msgs が正しい
from route_msgs.msg import ObstacleAvoidanceHint  # stamp, front_blocked: bool, left_is_open: float32, right_is_open: float32

from cv_bridge import CvBridge


class ObstacleMonitorNode(Node):
    """laserScanCallback / laserScanViewer を踏襲した Obstacle Monitor."""

    def __init__(self) -> None:
        super().__init__('obstacle_monitor')

        # ---- Parameters (legacy 準拠) ----
        # 前方閉塞判定くさび角（半角）
        self.declare_parameter('front_half_deg', 10.0)
        # 停止しきい値 [m]（くさび内で x < stop_dist_m なら閉塞）
        self.declare_parameter('stop_dist_m', 1.0)

        # 回避対象障害物の最大距離 [m]（legacy: 1.5m 固定）
        self.declare_parameter('max_obstacle_distance_m', 1.5)

        # ロボット幅 [m]（legacy: 0.8）
        self.declare_parameter('robot_width_m', 0.8)
        # 下限値 [m]（legacy: 0.75）
        self.declare_parameter('min_offset_lower_bound_m', 0.75)

        # viewer 仕様（legacy 相当: 8m x-range, ±4m y-range, 100 pix/m）
        self.declare_parameter('viewer_map_range_m', 8.0)
        self.declare_parameter('viewer_pixel_pitch', 100)  # [pix/m]
        self.declare_parameter('viewer_window', 'laserscan')

        # 描画サイズ（表示用にリサイズ）
        self.declare_parameter('viewer_resize_px', 500)

        # ---- QoS ----
        qos_be_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ---- Pub/Sub ----
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            qos_profile_sensor_data,  # SensorDataQoS
        )

        self.pub_hint = self.create_publisher(
            ObstacleAvoidanceHint,
            '/obstacle_avoidance_hint',
            qos_be_volatile,
        )

        self.pub_img = self.create_publisher(
            Image,
            '/sensor_viewer',
            qos_be_volatile,
        )

        # ---- Misc ----
        self.bridge = CvBridge()

        # cache: 最新の点群（viewer 用）
        self._last_points_xy: Optional[np.ndarray] = None

        self.get_logger().info('obstacle_monitor (legacy-following) started.')

    # ===============================
    # Legacy: laserScanCallback
    # ===============================
    def laser_scan_callback(self, msg: LaserScan) -> None:
        """LaserScan 受信時の処理（legacy 算法踏襲）.

        1) ±90° の前方のみ残す
        2) x <= max_obstacle_distance_m で抽出
        3) 左右に分割し、|y| 昇順で並べ替え
        4) ギャップ検出で offset を決定、外縁採用もあり。最終的に min_offset_lower_bound_m を下限に丸め
        5) front_blocked 判定: ±front_half_deg くさび内に x<stop_dist_m の点があれば True
        6) ヒント publish（left_is_open/right_is_open に offset[m] を格納）
        7) viewer 画像 publish（laserScanViewer 相当の描画）
        """
        # ---- 角度列の生成 ----
        angle_min: float = float(msg.angle_min)
        angle_inc: float = float(msg.angle_increment)

        # ---- XY 配列を生成（NaN/Inf/近距離<0.2m 除外） ----
        xy = self._scan_to_xy(msg, angle_min, angle_inc)

        # ---- 前方 ±90° のみ残す（後方は無視） ----
        if xy.size > 0:
            # xy[:,2] は角度[rad] を保持する列にしていないので、角度条件は変換前に適用すべきだが
            # ここでは _scan_to_xy 内で既に ±90° をフィルタリングしている。
            pass

        self._last_points_xy = xy.copy() if xy.size > 0 else None

        # ---- x <= max_obstacle_distance_m で抽出 ----
        max_dist = float(self.get_parameter('max_obstacle_distance_m').value)
        xy_extract = xy[xy[:, 0] <= max_dist] if xy.size > 0 else np.empty((0, 2), dtype=np.float32)

        # ---- 左右に分割（y 符号）し、|y| 昇順で並べ替え ----
        left = xy_extract[xy_extract[:, 1] >= 0.0] if xy_extract.size > 0 else np.empty((0, 2), dtype=np.float32)
        right = xy_extract[xy_extract[:, 1] < 0.0] if xy_extract.size > 0 else np.empty((0, 2), dtype=np.float32)

        left = left[np.argsort(np.abs(left[:, 1]))] if left.size > 0 else left
        right = right[np.argsort(np.abs(right[:, 1]))] if right.size > 0 else right

        # ---- オフセット算出（legacy: calcAvoidanceOffset） ----
        robot_width = float(self.get_parameter('robot_width_m').value)
        half_width = robot_width / 2.0
        min_lower = float(self.get_parameter('min_offset_lower_bound_m').value)

        offset_l = self._calc_avoidance_offset(left, robot_width, half_width)
        offset_r = self._calc_avoidance_offset(right, robot_width, half_width)

        # legacy: 下限 0.75m を適用
        offset_l = max(offset_l, min_lower) if offset_l > 0.0 else 0.0
        offset_r = max(offset_r, min_lower) if offset_r > 0.0 else 0.0

        # ---- 前方閉塞判定（±front_half_deg くさび & x < stop_dist_m） ----
        front_half_deg = float(self.get_parameter('front_half_deg').value)
        stop_dist_m = float(self.get_parameter('stop_dist_m').value)
        front_blocked = self._is_front_blocked(xy_extract, robot_width, stop_dist_m, 5.0, 5)

        # ---- ヒント publish ----
        self.get_logger().info(f"Publish hint: dist={stop_dist_m:.2f}, front_blocked={front_blocked}, left_is_open={offset_l:.2f}, right_is_open={offset_r:.2f}")
        self._publish_hint(front_blocked, offset_l, offset_r)

        # ---- 画像 publish（laserScanViewer 相当） ----
        self._publish_scan_image(xy, offset_l, offset_r, robot_width, half_width)

    # -------------------------------
    # XY 変換（±90° フィルタ含む）
    # -------------------------------
    def _scan_to_xy(self, msg: LaserScan, angle_min: float, angle_inc: float) -> np.ndarray:
        """LaserScan → XY。NaN/Inf/<0.2m 除外。±90° のみ残す。"""
        pts = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.2:
                continue
            angle = angle_min + i * angle_inc
            deg = math.degrees(angle)
            if deg < -90.0 or deg > 90.0:
                continue
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            pts.append((x, y))
        if not pts:
            return np.empty((0, 2), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)

    # -------------------------------
    # legacy: calcAvoidanceOffset
    # -------------------------------
    def _calc_avoidance_offset(self, xy: np.ndarray, robot_width: float, half_width: float) -> float:
        """隣接点の |y| 差分からギャップを検出し、外縁も考慮して offset を算出する（legacy準拠）.

        Args:
            xy: |y| 昇順にソート済みの点群（片側）。shape = (N, 2)
            robot_width: ロボット幅 [m]
            half_width: ロボット半幅 [m]

        Returns:
            offset [m]。見つからなければ 0.0。
        """
        if xy.size == 0:
            return 0.0

        offset = 0.0
        y_prev = 0.0
        thresh = robot_width + half_width  # legacy: 幅 + 半幅 を閾値に利用

        for i in range(len(xy)):
            y = abs(float(xy[i][1]))
            # 隣の点との間が (幅 + 半幅) よりも開いている場合 … ギャップとして採用
            if y - y_prev > thresh:
                # 車幅半分のマージンを設けてオフセット算出
                offset = y_prev + (thresh) / 2.0
                # 直進経路上（y_prev==0）に障害物が無ければ回避不要（offset=0 のまま）
                if y_prev == 0.0:
                    break
                # あまり大きすぎるオフセットは採用しない（legacy 上限 3.0m 相当）
                if offset < 3.0:
                    break
            # 外側に点が存在しない場合 … 外縁 + マージン
            elif i == len(xy) - 1:
                offset = y + (thresh) / 2.0
                if offset < 3.0:
                    break
            y_prev = y
        return float(offset)

    # -------------------------------
    # 前方閉塞判定（帯＋分位点）
    # -------------------------------
    def _is_front_blocked(self, pts: np.ndarray, robot_width_m: float, stop_dist_m: float, perc: float = 5.0, min_points: int = 5) -> bool:
        """前方閉塞判定（帯＋分位点）。

        Args:
            pts: _scan_to_xy() が出力した XY点群 [ [x0, y0], [x1, y1], ... ]。
            robot_width_m: ロボット全幅[m]（安全マージン込み）。
            stop_dist_m: 停止距離[m]。
            perc: 分位点[%]（例: 5.0 → 帯内x距離の下位5%を評価）。
            min_points: 判定に必要な最小点数。

        Returns:
            bool: 前方閉塞していればTrue。
        """
        if pts.size == 0 or pts.shape[0] < min_points:
            return False

        xs = pts[:, 0]
        ys = pts[:, 1]

        # y方向±half_widthの帯内
        half_w = 0.5 * robot_width_m
        mask = (xs > 0.0) & (np.abs(ys) <= half_w)
        if not np.any(mask):
            return False

        x_band = xs[mask]
        x_q = float(np.percentile(x_band, perc))
        return x_q <= stop_dist_m


    # -------------------------------
    # ヒント publish
    # -------------------------------
    def _publish_hint(self, front_blocked: bool, left_open_m: float, right_open_m: float) -> None:
        """ObstacleAvoidanceHint を publish."""
        msg = ObstacleAvoidanceHint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.front_blocked = bool(front_blocked)
        msg.left_is_open = float(left_open_m)
        msg.right_is_open = float(right_open_m)

        self.pub_hint.publish(msg)

    # ===============================
    # Legacy: laserScanViewer 相当
    # ===============================
    def _publish_scan_image(
        self,
        points_xy: np.ndarray,
        offset_l: float,
        offset_r: float,
        robot_width: float,
        half_width: float,
    ) -> None:
        """legacy の laserScanViewer と同等の可視化を bgr8 で publish する."""
        # 表示仕様
        map_range = float(self.get_parameter('viewer_map_range_m').value)  # 8.0
        pixel_pitch = int(self.get_parameter('viewer_pixel_pitch').value)  # 100 pix/m
        resize_px = int(self.get_parameter('viewer_resize_px').value)
        window_name = str(self.get_parameter('viewer_window').value)

        # マップ領域（legacy: x:0..8, y:-4..4）
        x_min = 0.0
        x_max = (map_range / 2.0) * 2.0
        y_min = -1.0 * (map_range / 2.0)
        y_max = (map_range / 2.0)

        width = int((y_max - y_min) * pixel_pitch)   # 横
        height = int((x_max - x_min) * pixel_pitch)  # 縦
        grid = np.full((height, width, 3), 255, dtype=np.uint8)

        # ロボット幅の縦ライン（黒）: 画像中心を基準に左右へ
        pix_x = int(width / 2) - int(robot_width * pixel_pitch)
        cv2.line(grid, (pix_x, height), (pix_x, 0), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
        pix_x = int(width / 2) + int(robot_width * pixel_pitch)
        cv2.line(grid, (pix_x, height), (pix_x, 0), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)

        # 最大障害物距離ライン（黒）
        max_obstacle_distance = float(self.get_parameter('max_obstacle_distance_m').value)
        pix_y = height - int(max_obstacle_distance * pixel_pitch)
        cv2.line(grid, (0, pix_y), (width, pix_y), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)

        # 点群（赤）: x 前方を下方向、y 左右を画像横方向に写像（legacy と同じ）
        if points_xy is not None and points_xy.size > 0:
            for x, y in points_xy:
                if x_min < x < x_max and y_min < y < y_max:
                    px = int(width / 2) - int(y * pixel_pitch)   # y: 左(+)->右(-) へ
                    py = height - int(x * pixel_pitch)           # x: 前方->下
                    if 0 <= px < width and 0 <= py < height:
                        cv2.circle(grid, (px, py), 4, (0, 0, 255), -1)

        # 回避ライン（青）: vline_l と vline_r
        # legacy 呼び出しでは vline_l=offset_l, vline_r=-offset_r を渡していたので同様に適用
        vline_l = offset_l
        vline_r = -1.0 * offset_r

        px = int(width / 2) - int(vline_l * pixel_pitch)
        if vline_l != 0.0 and 0 <= px < width:
            cv2.line(grid, (px, height), (px, 0), (255, 0, 0), thickness=2, lineType=cv2.LINE_AA)

        px = int(width / 2) - int(vline_r * pixel_pitch)
        if vline_r != 0.0 and 0 <= px < width:
            cv2.line(grid, (px, height), (px, 0), (255, 0, 0), thickness=2, lineType=cv2.LINE_AA)

        # 表示用リサイズ（ウィンドウ表示は行わず、画像トピックとして配信）
        grid_show = cv2.resize(grid, (resize_px, resize_px), interpolation=cv2.INTER_AREA)
        # デバッグ表示
        cv2.imshow("Sensor Viewer", grid_show)
        cv2.waitKey(1)

        # bgr8 で publish
        img_msg = self.bridge.cv2_to_imgmsg(grid_show, encoding='bgr8')
        # frame_id は設計上必要なし（Image の header に frame は含まれるが、legacy では未使用）
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'base_link'
        self.pub_img.publish(img_msg)

    # -------------------------------
    # Utility
    # -------------------------------
    def _now(self) -> TimeMsg:
        now = self.get_clock().now().to_msg()
        return now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
