#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/active_route を MarkerArray へ変換して配信するノード."""
from __future__ import annotations

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from route_msgs.msg import Route, Waypoint  # type: ignore


def _q_to_yaw(q: Quaternion) -> float:
    """クォータニオンをyaw角[rad]へ変換する."""

    x = float(getattr(q, "x", 0.0))
    y = float(getattr(q, "y", 0.0))
    z = float(getattr(q, "z", 0.0))
    w = float(getattr(q, "w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ActiveRouteMarkerNode(Node):
    """/active_route を購読して可視化用MarkerArrayを生成するノード."""

    def __init__(self) -> None:
        super().__init__("active_route_marker")

        qos_route = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_marker = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_route = self.create_subscription(
            Route, "active_route", self._on_route, qos_route
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, "active_route/markers", qos_marker
        )

        self.get_logger().info("active_route_marker node started.")

    def _on_route(self, msg: Route) -> None:
        """Route受信時にMarkerArrayへ変換してpublishする."""

        header = self._build_header(msg.header)
        marker_array = MarkerArray()
        marker_array.markers.append(self._build_delete_all_marker(header))

        if not msg.waypoints:
            self.get_logger().warn("/active_route が空のためMarkerを削除して終了します。")
            self.pub_markers.publish(marker_array)
            return

        segments_marker = self._build_segments_marker(header, msg.waypoints)
        left_open_marker, right_open_marker = self._build_open_markers(header, msg.waypoints)
        waypoint_markers = self._build_waypoint_markers(header, msg.waypoints)
        label_markers = self._build_label_markers(header, msg.waypoints)

        marker_array.markers.append(segments_marker)
        marker_array.markers.append(left_open_marker)
        marker_array.markers.append(right_open_marker)
        marker_array.markers.extend(waypoint_markers)
        marker_array.markers.extend(label_markers)

        self.pub_markers.publish(marker_array)

    def _build_header(self, src_header: Header) -> Header:
        """受信ヘッダーを元に可視化用ヘッダーを生成する."""

        header = Header()
        header.frame_id = getattr(src_header, "frame_id", "map") or "map"
        header.stamp = self.get_clock().now().to_msg()
        return header

    @staticmethod
    def _build_delete_all_marker(header: Header) -> Marker:
        """既存Markerを全消去するMarkerを生成する."""

        marker = Marker()
        marker.header = header
        marker.action = Marker.DELETEALL
        return marker

    def _build_segments_marker(self, header: Header, waypoints: List[Waypoint]) -> Marker:
        """Waypoint間の線分をLINE_LISTで描画する."""

        marker = Marker()
        marker.header = header
        marker.ns = "active_route_segments"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.07
        marker.color.r = 0.4
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 1.0

        for i in range(len(waypoints) - 1):
            a = waypoints[i].pose.position
            b = waypoints[i + 1].pose.position
            marker.points.append(self._point_from_xyz(a.x, a.y, a.z))
            marker.points.append(self._point_from_xyz(b.x, b.y, b.z))
        return marker

    def _build_open_markers(self, header: Header, waypoints: List[Waypoint]) -> tuple[Marker, Marker]:
        """左右open距離をLINE_LISTで描画するMarkerを生成する."""

        left_marker = Marker()
        left_marker.header = header
        left_marker.ns = "active_route_left_open"
        left_marker.id = 0
        left_marker.type = Marker.LINE_LIST
        left_marker.action = Marker.ADD
        left_marker.scale.x = 0.05
        left_marker.color.r = 0.5
        left_marker.color.g = 0.3
        left_marker.color.b = 1.0
        left_marker.color.a = 1.0

        right_marker = Marker()
        right_marker.header = header
        right_marker.ns = "active_route_right_open"
        right_marker.id = 0
        right_marker.type = Marker.LINE_LIST
        right_marker.action = Marker.ADD
        right_marker.scale.x = 0.05
        right_marker.color.r = 0.3
        right_marker.color.g = 1.0
        right_marker.color.b = 0.3
        right_marker.color.a = 1.0

        for wp in waypoints:
            yaw = _q_to_yaw(wp.pose.orientation)
            origin = self._point_from_xyz(
                wp.pose.position.x,
                wp.pose.position.y,
                wp.pose.position.z,
            )
            if wp.left_open > 0.0:
                end_left = self._offset_point(origin, yaw + math.pi / 2.0, wp.left_open)
                left_marker.points.append(origin)
                left_marker.points.append(end_left)
            if wp.right_open > 0.0:
                end_right = self._offset_point(origin, yaw - math.pi / 2.0, wp.right_open)
                right_marker.points.append(origin)
                right_marker.points.append(end_right)

        return left_marker, right_marker

    def _build_waypoint_markers(self, header: Header, waypoints: List[Waypoint]) -> List[Marker]:
        """各Waypointの位置を示すMarker群を生成する."""

        markers: List[Marker] = []
        for idx, wp in enumerate(waypoints):
            marker = Marker()
            marker.header = header
            marker.ns = "active_route_waypoints"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = wp.pose
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = self._waypoint_color(wp)
            markers.append(marker)
        return markers

    def _build_label_markers(self, header: Header, waypoints: List[Waypoint]) -> List[Marker]:
        """Waypointラベル表示用のTEXT_MARKER群を生成する."""

        markers: List[Marker] = []
        for idx, wp in enumerate(waypoints):
            marker = Marker()
            marker.header = header
            marker.ns = "active_route_labels"
            marker.id = idx
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = self._point_from_xyz(
                wp.pose.position.x, wp.pose.position.y, wp.pose.position.z + 0.6
            )
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = str(wp.label)
            markers.append(marker)
        return markers

    @staticmethod
    def _waypoint_color(wp: Waypoint) -> tuple[float, float, float, float]:
        """STOP/SIGフラグに応じた色を返す."""

        if getattr(wp, "line_stop", False):
            return 1.0, 0.1, 0.1, 1.0
        if getattr(wp, "signal_stop", False):
            return 1.0, 0.9, 0.1, 1.0
        return 0.9, 0.9, 0.9, 1.0

    @staticmethod
    def _point_from_xyz(x: float, y: float, z: float) -> Point:
        """座標値からPointを生成する."""

        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _offset_point(self, origin: Point, yaw: float, distance: float) -> Point:
        """yawと距離からオフセットしたPointを計算する."""

        dx = math.cos(yaw) * distance
        dy = math.sin(yaw) * distance
        return self._point_from_xyz(origin.x + dx, origin.y + dy, origin.z)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActiveRouteMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
