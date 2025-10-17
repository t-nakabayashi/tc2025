#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""route_follower_node（ROS2ラッパー層）

follower_core.FollowerCore を内部に保持し、
ROS2のPublisher / Subscriber / Timer / ServiceClient を仲介する。

役割：
    - ROSメッセージを Core の内部構造体（Pose, Waypoint, Route, HintSample）へ変換。
    - Core.tick() で得た出力を ROSメッセージ（PoseStamped, FollowerState）としてpublish。
    - /report_stuck サービスの呼び出しを管理（CoreからWAITING_REROUTE遷移時に発火）。
"""

from __future__ import annotations

import time
import sys
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header, Bool, Int32
from route_msgs.msg import Route, Waypoint, FollowerState, ObstacleAvoidanceHint  # type: ignore
from route_msgs.srv import ReportStuck  # type: ignore

# 可変ルート探索（外部モジュール）
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

from follower_core import (
    FollowerCore,
    Pose as CorePose,
    Waypoint as CoreWaypoint,
    Route as CoreRoute,
    HintSample,
)


# ============================================================
# QoS設定ヘルパ
# ============================================================

def qos_transient_local(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_volatile(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_best_effort(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ============================================================
# メインノード
# ============================================================

class RouteFollowerNode(Node):
    """route_follower（ROS2ラッパー層）."""

    def __init__(self) -> None:
        super().__init__("route_follower")
        self.core = FollowerCore(self.get_logger())
        self.get_logger().info("route_follower_node started.")

        # QoS設定
        self.qos_tl = qos_transient_local()
        self.qos_vol = qos_volatile()
        self.qos_be = qos_best_effort()

        # Pub/Sub設定
        self.sub_route = self.create_subscription(Route, "/active_route", self._on_route, self.qos_tl)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, self.qos_vol)
        self.sub_hint = self.create_subscription(
            ObstacleAvoidanceHint, "/obstacle_avoidance_hint", self._on_hint, self.qos_be
        )
        self.sub_manual = self.create_subscription(Bool, "/manual_start", self._on_manual_start, self.qos_vol)
        self.sub_sig = self.create_subscription(Int32, "/sig_recog", self._on_sig_recog, self.qos_vol)

        self.pub_target = self.create_publisher(PoseStamped, "/active_target", self.qos_vol)
        self.pub_state = self.create_publisher(FollowerState, "/follower_state", self.qos_vol)

        self.cli_report_stuck = self.create_client(ReportStuck, "/report_stuck")

        # Timer
        self._last_pub_target_pose = None
        self._last_pub_time = 0.0
        self._republish_interval = 1.0 / self.core.republish_target_hz
        self.timer = self.create_timer(1.0 / 20.0, self._on_timer)

    # ========================================================
    # Callback群
    # ========================================================

    def _on_route(self, msg: Route) -> None:
        """/active_route 受信."""
        if not msg.waypoints:
            self.get_logger().warn("/active_route: 空のrouteを無視します。")
            return

        wp_list = []
        for w in msg.waypoints:
            wp_list.append(
                CoreWaypoint(
                    label=w.label,
                    pose=CorePose(w.pose.position.x, w.pose.position.y, self._yaw_from_quat(w.pose.orientation)),
                    line_stop=bool(getattr(w, "line_stop", False)),
                    signal_stop=bool(getattr(w, "signal_stop", False)),
                    left_open=float(getattr(w, "left_open", 0.0)),
                    right_open=float(getattr(w, "right_open", 0.0)),
                )
            )

        route = CoreRoute(version=int(getattr(msg, "version", -1)), waypoints=wp_list,
                            start_index=int(getattr(msg, "start_index", 0)), start_label=str(getattr(msg, "start_label", "")))
        self.core.update_route(route)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """/amcl_pose 受信."""
        # PoseWithCovarianceStampedをPoseStampedに変換
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        pose = CorePose(
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            self._yaw_from_quat(pose_stamped.pose.orientation),
        )
        self.core.update_pose(pose)

    def _on_hint(self, msg: ObstacleAvoidanceHint) -> None:
        """/obstacle_avoidance_hint 受信."""
        sample = HintSample(
            t=time.time(),
            front_blocked=bool(msg.front_blocked),
            left_open=float(msg.left_is_open),
            right_open=float(msg.right_is_open),
        )
        self.core.update_hint(sample)

    def _on_manual_start(self, msg: Bool) -> None:
        """manual_start(Bool) の立ち上がりで Core に通知"""
        if msg.data:
            self.core.update_control_inputs(manual_start=True)

    def _on_sig_recog(self, msg: Int32) -> None:
        """sig_recog(Int32) の最新値を Core に渡す"""
        self.core.update_control_inputs(sig_recog=int(msg.data))

    # ========================================================
    # 周期処理
    # ========================================================

    def _on_timer(self) -> None:
        """20Hz周期でCore.tick()実行."""
        output = self.core.tick()

        # active_target 出力
        self._handle_target_publish(output)

        # follower_state 出力
        self._handle_state_publish(output)

        # stuck報告判定
        if self.core.status == self.core.status.WAITING_REROUTE:
            self._handle_stuck_report()

    def _handle_target_publish(self, output):
        """active_targetのデバウンス付きpublish"""
        now_ros = self.get_clock().now()
        now_sec = now_ros.seconds_nanoseconds()[0] + now_ros.seconds_nanoseconds()[1] * 1e-9
        pose = output.target_pose
        need_pub = False

        if pose is not None:
            if (self._last_pub_target_pose is None
                or self._euclid_diff(pose, self._last_pub_target_pose) > 1e-6
                or (now_sec - self._last_pub_time) >= self._republish_interval):
                need_pub = True

        if not need_pub:
            return

        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = now_ros.to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose = self._pose_to_msg(pose)
        self.pub_target.publish(pose_msg)
        self._last_pub_target_pose = pose
        self._last_pub_time = now_sec

    def _handle_state_publish(self, output):
        """FollowerState メッセージを生成・publish"""
        if output.state is None:
            return

        state = output.state
        msg = FollowerState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.route_version = int(state["route_version"])
        msg.state = str(state["status"])
        msg.current_index = int(state["index"])
        msg.avoidance_attempt_count = int(state["avoid_count"])
        msg.last_stagnation_reason = str(state["reason"])
        self.pub_state.publish(msg)

    # ========================================================
    # stuck報告
    # ========================================================

    def _handle_stuck_report(self) -> None:
        """WAITING_REROUTE遷移時に /report_stuck 呼び出し."""
        if not self.cli_report_stuck.service_is_ready():
            self.get_logger().warn("/report_stuck not ready.")
            return

        req = ReportStuck.Request()
        req.route_version = int(self.core.route_version)
        req.current_index = int(self.core.index)
        req.reason = str(self.core.last_stagnation_reason)
        req.avoid_trial_count = int(self.core.avoid_attempt_count)
        req.last_applied_offset_m = float(self.core.last_applied_offset_m)

        future = self.cli_report_stuck.call_async(req)

        def _cb_done(fut):
            if fut.cancelled() or not fut.done():
                self.get_logger().warn("report_stuck timeout or cancelled.")
                return
            res: ReportStuck.Response = fut.result()
            self.get_logger().info(
                f"report_stuck result: decision={res.decision}, note='{res.note}', offset_hint={res.offset_hint:.2f}"
            )

        future.add_done_callback(_cb_done)

    # ========================================================
    # ユーティリティ
    # ========================================================

    def _yaw_from_quat(self, q: Quaternion) -> float:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _euclid_diff(self, a: CorePose, b: CorePose) -> float:
        """Pose間の位置差を返す（m単位）"""
        return math.hypot(a.x - b.x, a.y - b.y)

    def _pose_to_msg(self, pose: CorePose) -> Pose:
        p = Pose()
        p.position.x = pose.x
        p.position.y = pose.y
        p.position.z = 0.0
        q = Quaternion()
        q.x, q.y = 0.0, 0.0
        q.z = math.sin(pose.yaw / 2.0)
        q.w = math.cos(pose.yaw / 2.0)
        p.orientation = q
        return p


# ============================================================
# main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
