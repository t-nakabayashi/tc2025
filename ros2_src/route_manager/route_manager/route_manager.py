#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
route_manager ノード（phase1 実装・I/F確定反映版／非ブロッキング化）

変更点（2025-10-10）:
- Services 名称を ROS2 標準（小文字スネークケース＋先頭スラッシュ）に変更: /get_route, /update_route
- GetRoute / UpdateRoute 応答の success / message をハンドリング
- /mission_info（TRANSIENT_LOCAL）, /route_state（VOLATILE） を追加実装
- follower_state を購読して進捗を /route_state に反映（FINISHED → COMPLETED）
- ライフサイクル状態管理: UNSET/REQUESTING/ACTIVE/UPDATING/COMPLETED/ERROR
- **重要**: spin_until_future_complete をコールバック内で使わない非ブロッキング実装に変更
  （Future の done コールバック＋ワンショットタイマーで timeout を実現）
- 受信Routeの厳密検証 (frame_id=='map', version>0, 経路非空, 画像同封, 画像encodingの任意検証)

依存メッセージ/サービス（route_msgs パッケージ想定）:
  - msg: Route, RouteState, MissionInfo, FollowerState
  - srv: GetRoute, UpdateRoute
"""

from __future__ import annotations

from typing import List, Optional

import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from route_msgs.msg import Route, RouteState, MissionInfo, FollowerState  # type: ignore
from route_msgs.srv import GetRoute, UpdateRoute  # type: ignore


def _qos_tl() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _qos_stream() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class RouteManager(Node):
    def __init__(self) -> None:
        super().__init__("route_manager")

        # ========= パラメータ宣言 =========
        self.declare_parameter("start_label", "")
        self.declare_parameter("goal_label", "")
        self.declare_parameter("checkpoint_labels", [])
        self.declare_parameter("auto_request_on_startup", True)

        # 追加（サービス名/timeout/retry/配信周期/画像検証）
        self.declare_parameter("planner_get_service_name", "/get_route")
        self.declare_parameter("planner_update_service_name", "/update_route")
        self.declare_parameter("planner_timeout_sec", 5.0)
        self.declare_parameter("planner_retry_count", 2)
        self.declare_parameter("state_publish_rate_hz", 1.0)
        self.declare_parameter("image_encoding_check", False)

        # ========= パラメータ取得 =========
        self.start_label: str = self.get_parameter("start_label").get_parameter_value().string_value
        self.goal_label: str = self.get_parameter("goal_label").get_parameter_value().string_value
        self.checkpoint_labels: List[str] = list(self.get_parameter("checkpoint_labels").get_parameter_value().string_array_value)
        self.auto_request: bool = self.get_parameter("auto_request_on_startup").get_parameter_value().bool_value

        self.planner_get_service_name: str = self.get_parameter("planner_get_service_name").get_parameter_value().string_value
        self.planner_update_service_name: str = self.get_parameter("planner_update_service_name").get_parameter_value().string_value
        self.planner_timeout_sec: float = float(self.get_parameter("planner_timeout_sec").get_parameter_value().double_value)
        self.planner_retry_count: int = int(self.get_parameter("planner_retry_count").get_parameter_value().integer_value)
        self.state_publish_rate_hz: float = float(self.get_parameter("state_publish_rate_hz").get_parameter_value().double_value)
        self.image_encoding_check: bool = self.get_parameter("image_encoding_check").get_parameter_value().bool_value

        # ========= QoS定義 =========
        self.qos_tl = _qos_tl()
        self.qos_stream = _qos_stream()

        # ========= Publisher =========
        self.pub_active_route = self.create_publisher(Route, "/active_route", self.qos_tl)
        self.pub_route_state = self.create_publisher(RouteState, "/route_state", self.qos_stream)
        self.pub_mission_info = self.create_publisher(MissionInfo, "/mission_info", self.qos_tl)

        # ========= Subscriber =========
        self.sub_follower_state = self.create_subscription(
            FollowerState, "/follower_state", self._on_follower_state, self.qos_stream
        )

        # ========= Service Clients =========
        self.cb_cli = MutuallyExclusiveCallbackGroup()
        self.cb_timer = ReentrantCallbackGroup()
        self.cli_get = self.create_client(GetRoute, self.planner_get_service_name, callback_group=self.cb_cli)
        self.cli_update = self.create_client(UpdateRoute, self.planner_update_service_name, callback_group=self.cb_cli)

        # ========= 内部状態 =========
        self.current_status: str = "UNSET"
        self.current_route: Optional[Route] = None
        self.current_version: int = 0
        self.current_index: int = -1
        self.current_label: str = ""

        # リクエスト管理（非ブロッキング用）
        self._get_seq: int = 0
        self._pending_get_seq: Optional[int] = None
        self._pending_get_timer = None  # rclpy.timer.Timer | None

        # ========= タイマー =========
        # 起動時一度だけ自動要求
        self._once_done = False
        self.timer_once = self.create_timer(0.1, self._on_ready_once, callback_group=self.cb_timer)
        # 状態の周期配信
        sp = max(0.1, 1.0 / max(0.1, self.state_publish_rate_hz))
        self.timer_state = self.create_timer(sp, self._publish_route_state, callback_group=self.cb_timer)

        self.get_logger().info("route_manager started")
        # 起動時に mission_info を配信（ラッチ）
        self._publish_mission_info()

    # ==============================================================
    # 起動直後の一度きり処理
    # ==============================================================
    def _on_ready_once(self) -> None:
        if self._once_done:
            return
        self._once_done = True

        if not self.auto_request:
            self.get_logger().info("auto_request_on_startup = False → 起動時のGetRoute呼び出しをスキップします。")
            return

        self.get_logger().info(
            f"GetRouteを実行: start={self.start_label}, goal={self.goal_label}, checkpoints={len(self.checkpoint_labels)}"
        )
        self.request_initial_route()

    # ==============================================================
    # follower_state 取り込み
    # ==============================================================
    def _on_follower_state(self, msg: FollowerState) -> None:
        try:
            self.current_index = int(getattr(msg, "current_index", self.current_index))
        except Exception:
            pass
        try:
            self.current_label = str(getattr(msg, "current_label", self.current_label))
        except Exception:
            pass
        status = str(getattr(msg, "status", "") or "")
        if status == "FINISHED" and self.current_status != "COMPLETED":
            self.current_status = "COMPLETED"
        elif status == "ERROR":
            self.current_status = "ERROR"
        self._publish_route_state()

    # ==============================================================
    # GetRoute（非ブロッキング）
    # ==============================================================
    def request_initial_route(self) -> None:
        if not self._wait_for_service(self.cli_get, self.planner_timeout_sec):
            self._set_error(f"GetRoute service not available: {self.planner_get_service_name}")
            return

        req = GetRoute.Request()
        req.start_label = self.start_label
        req.goal_label = self.goal_label
        req.checkpoint_labels = list(self.checkpoint_labels)

        self.current_status = "REQUESTING"
        self._publish_route_state()

        self._get_seq += 1
        seq = self._get_seq
        future = self.cli_get.call_async(req)

        # timeout タイマー（ワンショット）
        def on_timeout():
            if self._pending_get_seq == seq:
                self._pending_get_seq = None
                if self._pending_get_timer:
                    self._pending_get_timer.cancel()
                    self._pending_get_timer = None
                self._set_error("GetRoute timeout")  # 状態配信含む

        self._pending_get_seq = seq
        self._pending_get_timer = self.create_timer(self.planner_timeout_sec, on_timeout, callback_group=self.cb_timer)

        # 完了コールバック
        def on_done(fut):
            try:
                resp: GetRoute.Response = fut.result()
            except Exception as e:
                if self._pending_get_seq == seq:
                    if self._pending_get_timer:
                        self._pending_get_timer.cancel()
                        self._pending_get_timer = None
                    self._pending_get_seq = None
                    self._set_error(f"GetRoute call failed: {e}")
                return

            # 期限内に別シーケンスでタイムアウト済なら無視
            if self._pending_get_seq != seq:
                return

            # 正常経路
            if self._pending_get_timer:
                self._pending_get_timer.cancel()
                self._pending_get_timer = None
            self._pending_get_seq = None

            success = bool(getattr(resp, "success", True))
            message = str(getattr(resp, "message", "") or "")
            route: Optional[Route] = getattr(resp, "route", None)

            if not success:
                self._set_error(f"GetRoute failed (planner): {message or 'planner returned success=false'}")
                return
            if route is None:
                self._set_error("GetRoute response has no 'route'")
                return
            if not self._validate_route(route):
                self._set_error("GetRoute route validation failed")
                return

            self.current_route = route
            self.current_version = int(getattr(route, "version", 0))
            self.current_status = "ACTIVE"
            self._publish_active_route(route)
            self._publish_route_state()
            self.get_logger().info(
                f"Initial route accepted: version={self.current_version}, waypoints={self._count_waypoints(route)}"
            )

        future.add_done_callback(on_done)

    # ==============================================================
    # UpdateRoute（phase2 用の骨格。非ブロッキング + success/message）
    # ==============================================================
    def request_update_route(self, *, prev_index: Optional[int] = None, next_index: Optional[int] = None,
                             prev_wp_label: Optional[str] = None, next_wp_label: Optional[str] = None) -> None:
        if self.current_route is None or self.current_version <= 0:
            self.get_logger().error("No active route/version to update.")
            return

        pairs_ok = (prev_index is not None and prev_wp_label is not None) or (next_index is not None and next_wp_label is not None)
        if not pairs_ok:
            self.get_logger().error("UpdateRoute requires (prev_index & prev_wp_label) and/or (next_index & next_wp_label)." )
            return

        if not self._wait_for_service(self.cli_update, self.planner_timeout_sec):
            self.get_logger().error(f"UpdateRoute service not available: {self.planner_update_service_name}")
            return

        req = UpdateRoute.Request()
        req.route_version = int(self.current_version)
        if prev_index is not None:
            req.prev_index = int(prev_index)
        if next_index is not None:
            req.next_index = int(next_index)
        if prev_wp_label is not None:
            req.prev_wp_label = str(prev_wp_label)
        if next_wp_label is not None:
            req.next_wp_label = str(next_wp_label)

        self.current_status = "UPDATING"
        self._publish_route_state()

        future = self.cli_update.call_async(req)

        def on_done(fut):
            try:
                resp: UpdateRoute.Response = fut.result()
            except Exception as e:
                self._set_error(f"UpdateRoute call failed: {e}")
                return

            success = bool(getattr(resp, "success", True))
            message = str(getattr(resp, "message", "") or "")
            route: Optional[Route] = getattr(resp, "route", None)

            if not success:
                self._set_error(f"UpdateRoute failed (planner): {message or 'planner returned success=false'}")
                return
            if route is None:
                self._set_error("UpdateRoute response has no 'route'")
                return
            if not self._validate_route(route):
                self._set_error("UpdateRoute route validation failed")
                return

            self.current_route = route
            self.current_version = int(getattr(route, "version", 0))
            self.current_status = "ACTIVE"
            self._publish_active_route(route)
            self._publish_route_state()
            self.get_logger().info(
                f"Updated route accepted: version={self.current_version}, waypoints={self._count_waypoints(route)}"
            )

        future.add_done_callback(on_done)

    # ==============================================================
    # ユーティリティ
    # ==============================================================
    def _validate_route(self, route: Route) -> bool:
        frame_id = getattr(getattr(route, "header", None), "frame_id", "")
        if frame_id != "map":
            self.get_logger().error(f"Invalid frame_id: expected 'map', got '{frame_id}'")
            return False

        version = int(getattr(route, "version", 0))
        if version <= 0:
            self.get_logger().error(f"Invalid route.version: {version} (must be > 0)")
            return False

        n_wp = self._count_waypoints(route)
        if n_wp <= 0:
            self.get_logger().error("Route has no waypoints.")
            return False

        img: Optional[Image] = getattr(route, "route_image", None)
        if img is None:
            self.get_logger().error("Route has no route_image.")
            return False
        if self.image_encoding_check:
            if not getattr(img, "encoding", ""):
                self.get_logger().error("route_image.encoding is empty.")
                return False

        return True

    def _count_waypoints(self, route: Route) -> int:
        try:
            return len(getattr(route, "waypoints"))
        except Exception:
            return 0

    def _publish_active_route(self, route: Route) -> None:
        self.pub_active_route.publish(route)

    def _publish_mission_info(self) -> None:
        msg = MissionInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.start_label = self.start_label
        msg.goal_label = self.goal_label
        msg.checkpoint_labels = list(self.checkpoint_labels)
        self.pub_mission_info.publish(msg)

    def _publish_route_state(self) -> None:
        msg = RouteState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.status = self.current_status
        msg.current_index = int(self.current_index)
        msg.current_label = str(self.current_label)
        msg.route_version = int(self.current_version)
        msg.total_waypoints = int(self._count_waypoints(self.current_route) if self.current_route else 0)
        self.pub_route_state.publish(msg)

    def _set_error(self, reason: str) -> None:
        self.current_status = "ERROR"
        self.get_logger().error(f"RouteManager ERROR: {reason}")
        self._publish_route_state()

    def _wait_for_service(self, client, timeout_sec: float) -> bool:
        start = time.time()
        while not client.wait_for_service(timeout_sec=0.2):
            if time.time() - start > timeout_sec:
                return False
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
