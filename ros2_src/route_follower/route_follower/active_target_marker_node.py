#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/active_target をMarkerへ変換して強調表示するノード."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class ActiveTargetMarkerNode(Node):
    """/active_target を購読し、ARROW Marker を配信するノード."""

    def __init__(self) -> None:
        super().__init__("active_target_marker")

        qos_target = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_marker = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_target = self.create_subscription(
            PoseStamped,
            "active_target",
            self._on_target,
            qos_target,
        )
        self.pub_marker = self.create_publisher(Marker, "active_target/marker", qos_marker)
        self.get_logger().info("active_target_marker node started.")

    def _on_target(self, msg: PoseStamped) -> None:
        """PoseStamped を ARROW Marker に変換してpublishする."""

        marker = Marker()
        marker.header = msg.header
        marker.ns = "active_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.pub_marker.publish(marker)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActiveTargetMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
