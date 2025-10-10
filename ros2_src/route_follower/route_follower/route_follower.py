#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
route_follower ノード（詳細設計書v2 + 現フェーズ修正版）

【現時点のフェーズ対応】
- 状態機械導入（Enum管理）
- line_stop属性の実装（到達でWAITING_STOPへ遷移、/manual_start=Trueで再開）
- /follower_state の周期発行（デバウンス付き）
- /active_target の1Hz再送
- /active_route のframe_id厳格チェック
- エラー処理強化・軽微修正
- 状態遷移は全てタイマーコールバック内で完結

将来フェーズ拡張想定:
- REROUTING 状態は定義済みだが遷移は未実装
- 障害物検知時のサブゴールやリルート拡張に対応可能な構造
"""

import math
import time
from enum import Enum, auto
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseStamped
from route_msgs.msg import Route, Waypoint, FollowerState  # type: ignore


def qos_transient_local() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def qos_volatile() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class FollowerStatus(Enum):
    IDLE = auto()
    RUNNING = auto()
    WAITING_STOP = auto()
    REROUTING = auto()  # 未使用（将来用）
    FINISHED = auto()
    ERROR = auto()


class RouteFollower(Node):
    def __init__(self) -> None:
        super().__init__("route_follower")

        # ===== QoS =====
        self.qos_tl = qos_transient_local()
        self.qos_vol = qos_volatile()

        # ===== パラメータ =====
        self.declare_parameter("arrival_threshold", 0.6)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("republish_target_hz", 1.0)  # [Hz] active_target再送
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("start_immediately", True)
        self.declare_parameter("state_debounce_ms", 100)

        # ===== 内部状態 =====
        self._route: Optional[Route] = None
        self._wp_list: List[Waypoint] = []
        self._index: int = 0
        self._last_pub_target: Optional[PoseStamped] = None
        self._last_pub_time: float = 0.0
        self._current_pose: Optional[PoseStamped] = None
        self._route_version: int = -1

        # 状態機械
        self._status: FollowerStatus = FollowerStatus.IDLE

        # イベントフラグ
        self._pending_route: Optional[Route] = None
        self._route_rejected: bool = False
        self._manual_start_true: bool = False

        # ===== Publisher / Subscriber =====
        self.sub_route = self.create_subscription(Route, "/active_route", self._on_route, self.qos_tl)
        self.sub_pose = self.create_subscription(PoseStamped, "/amcl_pose", self._on_pose, self.qos_vol)
        self.sub_manual = self.create_subscription(Bool, "/manual_start", self._on_manual_start, self.qos_vol)

        self.pub_target = self.create_publisher(PoseStamped, "/active_target", self.qos_tl)
        self.pub_state = self.create_publisher(FollowerState, "/follower_state", self.qos_vol)

        # ===== Timer =====
        period = 1.0 / max(self.get_parameter("control_rate_hz").value, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

        # follower_state デバウンス
        self._last_state_pub_time = 0.0
        self._state_debounce_s = self.get_parameter("state_debounce_ms").value / 1000.0

        self.get_logger().info("route_follower started.")

    # ====================== Subscribers ======================

    def _on_route(self, msg: Route) -> None:
        """ /active_route 受信 """
        if not msg.waypoints:
            self.get_logger().warn("/active_route: 空の route を受信。無視します。")
            return
        if not msg.header.frame_id:
            msg.header.frame_id = self.get_parameter("target_frame").value
        elif msg.header.frame_id != self.get_parameter("target_frame").value:
            self.get_logger().error(
                f"/active_route: frame mismatch '{msg.header.frame_id}' != '{self.get_parameter('target_frame').value}'"
            )
            self._route_rejected = True
            return
        self._pending_route = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        self._current_pose = msg

    def _on_manual_start(self, msg: Bool) -> None:
        if msg.data:
            self._manual_start_true = True

    # ====================== Timer ======================

    def _on_timer(self) -> None:
        """周期制御処理"""
        # 1) pending route / error
        if self._route_rejected:
            self._change_state(FollowerStatus.ERROR)
            self._route_rejected = False

        if self._pending_route is not None:
            new = self._pending_route
            self._pending_route = None
            if self._route is not None:
                # 次フェーズでREROUTINGへ遷移する想定
                self.get_logger().info("new route received (phase1): resetting route.")
            self._route = new
            self._wp_list = list(new.waypoints)
            self._index = 0
            try:
                self._route_version = int(getattr(new, "version", -1))
            except Exception:
                self._route_version = -1
            if self.get_parameter("start_immediately").value:
                self._change_state(FollowerStatus.RUNNING)
                self._publish_target(self._wp_list[self._index])
            else:
                self._change_state(FollowerStatus.IDLE)

        # 2) manual_start解除
        if self._manual_start_true:
            self._manual_start_true = False
            if self._status == FollowerStatus.WAITING_STOP:
                if self._index < len(self._wp_list) - 1:
                    self._index += 1
                    self._publish_target(self._wp_list[self._index])
                    self._change_state(FollowerStatus.RUNNING)
                else:
                    self._change_state(FollowerStatus.FINISHED)

        # 3) RUNNING動作
        if self._status == FollowerStatus.RUNNING and self._route and self._wp_list:
            if self._current_pose is None:
                self._resend_if_needed()
                self._publish_state()
                return
            target_wp = self._wp_list[self._index]
            if self._reached(self._current_pose, target_wp, self.get_parameter("arrival_threshold").value):
                # --- line_stop: 到達したwaypoint情報を出す ---
                if getattr(target_wp, "line_stop", False):
                    self.get_logger().info(
                        f"Reached line-stop waypoint. index={self._index}, label='{target_wp.label}'"
                    )
                    self._change_state(FollowerStatus.WAITING_STOP)

                # --- 通常: 次WPに進む ---
                elif self._index < len(self._wp_list) - 1:
                    self._index += 1
                    next_wp = self._wp_list[self._index]
                    self.get_logger().info(
                        f"Published next waypoint. index={self._index}, label='{next_wp.label}')"
                    )
                    self._publish_target(next_wp)

                else:
                    self.get_logger().info("Final waypoint reached. Follower finished.")
                    self._change_state(FollowerStatus.FINISHED)
            else:
                self._resend_if_needed()

        self._publish_state()

    # ====================== Helpers ======================

    def _publish_target(self, wp: Waypoint) -> None:
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("target_frame").value
        msg.pose = wp.pose
        self.pub_target.publish(msg)
        self._last_pub_target = msg
        self._last_pub_time = time.time()

    def _resend_if_needed(self) -> None:
        """active_target の保険的再送 (1Hz既定)"""
        interval = 1.0 / max(self.get_parameter("republish_target_hz").value, 1e-3)
        if self._last_pub_target is None:
            return
        if (time.time() - self._last_pub_time) >= interval:
            self.pub_target.publish(self._last_pub_target)
            self._last_pub_time = time.time()

    def _reached(self, cur: PoseStamped, wp: Waypoint, threshold: float) -> bool:
        dx = float(cur.pose.position.x) - float(wp.pose.position.x)
        dy = float(cur.pose.position.y) - float(wp.pose.position.y)
        dist = math.hypot(dx, dy)
        return dist < threshold

    def _change_state(self, new_state: FollowerStatus) -> None:
        if new_state != self._status:
            self.get_logger().info(f"state change: {self._status.name} -> {new_state.name}")
            self._status = new_state

    def _publish_state(self) -> None:
        now = time.time()
        if now - self._last_state_pub_time < self._state_debounce_s:
            return
        self._last_state_pub_time = now
        msg = FollowerState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.route_version = self._route_version
        msg.current_index = self._index
        msg.status = self._status.name
        cur_label = ""
        next_label = ""
        if self._wp_list and 0 <= self._index < len(self._wp_list):
            cur_label = self._wp_list[self._index].label
            if self._index + 1 < len(self._wp_list):
                next_label = self._wp_list[self._index + 1].label
        msg.current_waypoint_label = cur_label
        msg.next_waypoint_label = next_label
        if self._current_pose:
            msg.current_pose = self._current_pose.pose
            wp = self._wp_list[self._index] if self._wp_list else None
            if wp:
                dx = float(self._current_pose.pose.position.x) - float(wp.pose.position.x)
                dy = float(self._current_pose.pose.position.y) - float(wp.pose.position.y)
                msg.distance_to_target = math.hypot(dx, dy)
        else:
            from geometry_msgs.msg import Pose
            msg.current_pose = Pose()
            msg.distance_to_target = 0.0
        self.pub_state.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
