#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""route_manager_node.py
Phase2 準拠・正式版（5段階 replan/shift/skip/fallback/failed を統合）。

本ファイルは**ROS2依存のラッパー**に責務を限定し、実処理は `manager_core.py` と
`manager_fsm.py` に委譲する形へリファクタリングした。

- /report_stuck サーバで、以下の順に判断する：
  1) UpdateRoute をまず試す（replan_first）
  2) shift（左右オフセットで次WPのみ横シフト）
  3) skip（次WPをスキップしてローカル再配信）
  4) フォールバック UpdateRoute（fallback_replan）
  5) failed（HOLDING）
- バージョン：Route.version = major*1000 + minor。planner へは major のみ送信。
- GetRoute は初期ルート取得にのみ使用（ReportStuck では使用しない）。
- Google Python Style + 型ヒント + 日本語コメント を付与。

※ 上記の仕様/コメントは、元の実装から**省略せず**に移植・維持している。
"""
from __future__ import annotations

import sys
from pathlib import Path
import asyncio
import time
from typing import Any, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

# route_msgs はユーザ環境のメッセージ/サービスに準拠
from route_msgs.msg import FollowerState, ManagerStatus, MissionInfo, Route, RouteState  # type: ignore
from route_msgs.msg import Waypoint  # type: ignore
from route_msgs.srv import GetRoute, ReportStuck, UpdateRoute  # type: ignore

# 可変ルート探索（外部モジュール）
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

# 非ROS依存Core
from manager_core import (
    RouteManagerCore,
    RouteModel,
    WaypointLite,
    Pose2D,
    VersionMM,
)

# -----------------------------------------------------------------------------
# QoSユーティリティ（元実装をそのまま保持）
# -----------------------------------------------------------------------------
def qos_tl() -> QoSProfile:
    """Transient Local（ラッチ配信）向けQoS。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def qos_vol(depth: int = 10) -> QoSProfile:
    """通常ストリーム向けQoS（VOLATILE）。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# -----------------------------------------------------------------------------
# 変換ヘルパ：ROS <-> Core（非ROS）
# -----------------------------------------------------------------------------
def ros_route_to_core(route: Route) -> RouteModel:
    """ROS Route -> Core RouteLite へ最小限の情報を移す。"""
    rm_wps = []
    version_int = int(getattr(route, "version", 0))
    frame_id = getattr(getattr(route, "header", None), "frame_id", "map") or "map"
    has_image = getattr(route, "route_image", None) is not None
    for wp in getattr(route, "waypoints", []):
        w = WaypointLite(
            label=str(getattr(wp, "label", "")),
            pose=Pose2D(
                x=float(getattr(getattr(wp, "pose", None).position, "x", 0.0)),
                y=float(getattr(getattr(wp, "pose", None).position, "y", 0.0)),
            ),
            line_stop=bool(getattr(wp, "line_stop", False)),
            signal_stop=bool(getattr(wp, "signal_stop", False)),
            not_skip=bool(getattr(wp, "not_skip", False)),
            right_open=float(getattr(wp, "right_open", 0.0) or 0.0),
            left_open=float(getattr(wp, "left_open", 0.0) or 0.0),
        )
        rm_wps.append(w)
    return RouteModel(
        waypoints=rm_wps,
        version=VersionMM(major=version_int, minor=0),
        frame_id=frame_id,
        has_image=has_image,
        current_index=0 if rm_wps else -1,
        current_label=(rm_wps[0].label if rm_wps else ""),
    )


def core_route_to_ros(route: RouteModel) -> Route:
    """Core RouteLite -> ROS Route へ変換する。"""
    msg = Route()
    msg.header = Header()
    msg.header.frame_id = route.frame_id or "map"
    msg.version = int(route.version.to_int())
    msg.start_index = route.current_index
    msg.start_wp_label = route.current_label
    # route_image の実体はplanner応答から受領する前提。ここでは有無のみを尊重。
    if route.has_image:
        msg.route_image = Image()  # encodingなどはplanner実装に依存
    msg.waypoints = []
    for w in route.waypoints:
        # Waypoint の詳細フィールドはユーザ環境の定義に依存、代表的なもののみ移送
        wp = Waypoint()
        # ただし上記は型情報が不透明になり得るため、実際の環境定義に合わせて必要に応じて置き換えること
        # 最低限の位置・ラベル・フラグを反映
        try:
            # 位置
            wp.pose.position.x = float(w.pose.x)
            wp.pose.position.y = float(w.pose.y)
            # ラベル/フラグ
            wp.label = str(w.label)
            wp.line_stop = bool(w.line_stop)
            wp.signal_stop = bool(w.signal_stop)
            wp.not_skip = bool(w.not_skip)
            # 開放長
            wp.right_open = float(w.right_open)
            wp.left_open = float(w.left_open)
        except Exception:
            pass
        msg.waypoints.append(wp)
    return msg


# -----------------------------------------------------------------------------
# Node本体
# -----------------------------------------------------------------------------
class RouteManagerNode(Node):
    """RouteManager のROS2 I/F実装（Phase2・正式5段階版）。

    本ノードは「通信とI/F」に徹し、実処理は `RouteManagerCore` へ委譲する。
    """

    def __init__(self) -> None:
        super().__init__("route_manager")

        # ---------------- パラメータ宣言 ----------------
        self.declare_parameter("start_label", "")
        self.declare_parameter("goal_label", "")
        self.declare_parameter("checkpoint_labels", [])
        self.declare_parameter("auto_request_on_startup", True)
        self.declare_parameter("planner_get_service_name", "/get_route")
        self.declare_parameter("planner_update_service_name", "/update_route")
        self.declare_parameter("planner_timeout_sec", 5.0)
        self.declare_parameter("planner_retry_count", 2)
        self.declare_parameter("state_publish_rate_hz", 1.0)
        self.declare_parameter("image_encoding_check", False)
        self.declare_parameter("report_stuck_timeout_sec", 5.0)
        self.declare_parameter("offset_step_m_max", 0.3)  # shift 最大横ずれ[m]

        # ---------------- パラメータ取得 ----------------
        self.start_label: str = self.get_parameter("start_label").get_parameter_value().string_value
        self.goal_label: str = self.get_parameter("goal_label").get_parameter_value().string_value
        self.checkpoint_labels: List[str] = list(
            self.get_parameter("checkpoint_labels").get_parameter_value().string_array_value
        )
        self.auto_request: bool = self.get_parameter("auto_request_on_startup").get_parameter_value().bool_value
        self.srv_get_name: str = self.get_parameter("planner_get_service_name").get_parameter_value().string_value
        self.srv_update_name: str = self.get_parameter("planner_update_service_name").get_parameter_value().string_value
        self.timeout_sec: float = float(self.get_parameter("planner_timeout_sec").get_parameter_value().double_value)
        self.retry_count: int = int(self.get_parameter("planner_retry_count").get_parameter_value().integer_value)
        self.state_rate_hz: float = float(self.get_parameter("state_publish_rate_hz").get_parameter_value().double_value)
        self.image_encoding_check: bool = self.get_parameter("image_encoding_check").get_parameter_value().bool_value
        self.report_stuck_timeout_sec: float = float(
            self.get_parameter("report_stuck_timeout_sec").get_parameter_value().double_value
        )
        self.offset_step_m_max: float = float(
            self.get_parameter("offset_step_m_max").get_parameter_value().double_value
        )

        # ---------------- QoS ----------------
        self.qos_tl = qos_tl()
        self.qos_stream = qos_vol()

        # ---------------- Publisher ----------------
        self.pub_active_route = self.create_publisher(Route, "/active_route", self.qos_tl)
        self.pub_route_state = self.create_publisher(RouteState, "/route_state", self.qos_stream)
        self.pub_mission_info = self.create_publisher(MissionInfo, "/mission_info", self.qos_tl)
        self.pub_manager_status = self.create_publisher(ManagerStatus, "/manager_status", self.qos_stream)

        # ---------------- Subscriber ----------------
        self.sub_follower_state = self.create_subscription(
            FollowerState, "/follower_state", self._on_follower_state, self.qos_stream
        )

        # ---------------- Service Clients ----------------
        self.cb_cli = MutuallyExclusiveCallbackGroup()
        self.cli_get = self.create_client(GetRoute, self.srv_get_name, callback_group=self.cb_cli)
        self.cli_update = self.create_client(UpdateRoute, self.srv_update_name, callback_group=self.cb_cli)

        # ---------------- Service Server ----------------
        self.cb_srv = MutuallyExclusiveCallbackGroup()
        self.srv_report_stuck = self.create_service(
            ReportStuck, "/report_stuck", self._on_report_stuck, callback_group=self.cb_srv
        )

        # ---------------- Core 構築 ----------------
        self.core = RouteManagerCore(
            logger=lambda m: self.get_logger().info(m),
            publish_active_route=self._publish_active_route_from_core,
            publish_status=self._publish_status_from_core,
            publish_route_state=self._publish_route_state_from_core,
            offset_step_m_max=self.offset_step_m_max,
        )

        # Planner呼び出しの非同期コールバックをCoreへ注入
        self.core.set_planner_callbacks(
            get_cb=self._planner_get_async,
            update_cb=self._planner_update_async,
        )

        # ---------------- タイマー ----------------
        self._once_done = False
        sp = max(0.1, 1.0 / max(0.1, self.state_rate_hz))
        self.timer_once = self.create_timer(0.1, self._on_ready_once)
        self.timer_state = self.create_timer(sp, self._publish_route_state_tick)

        self.get_logger().info("route_manager (Phase2: 5-step handling, Node/Core/FSM split) started.")
        self._publish_mission_info()

    # ------------------------------------------------------------------
    # Core -> Node: Publish 実装（ROSメッセージへ変換して配信）
    # ------------------------------------------------------------------
    def _publish_active_route_from_core(self, route: RouteModel) -> None:
        self.get_logger().info(f"[Node] publish /active_route: version={int(route.version.to_int())}, waypoints={len(route.waypoints)}")
        ros_route = core_route_to_ros(route)
        self.pub_active_route.publish(ros_route)

    def _publish_status_from_core(self, state: str, decision: str, cause: str, route_version: int) -> None:
        msg = ManagerStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.decision = decision
        msg.last_cause = cause
        msg.route_version = int(route_version)
        self.get_logger().info(f"[Node] publish /manager_status: state={state}, decision={decision}, cause={cause}, ver={int(route_version)}")
        self.pub_manager_status.publish(msg)

    def _publish_route_state_from_core(self, idx: int, label: str, ver: int, total: int, status: str) -> None:
        msg = RouteState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.status = status
        msg.current_index = int(idx)
        msg.current_label = str(label)
        msg.route_version = int(ver)
        msg.total_waypoints = int(total)
        #self.get_logger().info(f"[Node] publish /route_state: idx={idx}, label='{label}', ver={ver}, total={total}, status={status}")
        self.pub_route_state.publish(msg)

    # ------------------------------------------------------------------
    # Subscriber: follower_state
    # ------------------------------------------------------------------
    def _on_follower_state(self, msg: FollowerState) -> None:
        try:
            current_index = int(getattr(msg, "current_index", -1))
        except Exception:
            current_index = -1
        try:
            current_label = str(getattr(msg, "current_label", "") or "")
        except Exception:
            current_label = ""
        #self.get_logger().info(f"[Node] recv /follower_state: idx={current_index}, label='{current_label}'")
        self.core.update_follower_state(current_index, current_label)

    # ------------------------------------------------------------------
    # Service Server: /report_stuck（Core+FSMで5段階ロジックを維持）
    # ------------------------------------------------------------------
    def _on_report_stuck(self, req: ReportStuck.Request, res: ReportStuck.Response) -> ReportStuck.Response:
        self.get_logger().info("[Node] /report_stuck: received -> delegate to Core/FSM")
        # Coreのイベントループ上でFSM処理を非同期実行し、結果を同期的に取得
        result = self.core.run_async(self.core.on_report_stuck()).result()
        # Coreの決定内容をReportStuck.Responseに整形
        note = getattr(result, "message", "")
        if getattr(result, "success", False):
            # replan/shift/skip の区別は note で示す（元実装のコメントを維持）
            if note == "skipped":
                res.decision = 2  # DECISION_SKIP
            else:
                res.decision = 1  # DECISION_REPLAN（shift含む）
            res.note = note
            res.waiting_deadline = Duration(sec=0, nanosec=200 * 10**6)
            self.get_logger().info(f"[Node] /report_stuck: success decision={res.decision} note='{note}'")
        else:
            res.decision = 3  # DECISION_FAILED
            res.note = note or "avoidance_failed"
            res.waiting_deadline = Duration(sec=0, nanosec=0)
            self.get_logger().info(f"[Node] /report_stuck: failed note='{res.note}'")
        return res

    # ------------------------------------------------------------------
    # 起動直後
    # ------------------------------------------------------------------
    def _on_ready_once(self):
        """起動後一度だけFSMに初期ルート要求を投げる。"""
        if getattr(self, '_once_done', False):
            return
        self._once_done = True

        if not getattr(self, 'auto_request', True):
            self.get_logger().info("[Node] auto_request=False -> skip initial route request")
            return

        # ROSサービス接続待ち
        start = time.time()
        while not self.cli_get.wait_for_service(timeout_sec=0.2):
            if time.time() - start > getattr(self, 'connect_timeout_sec', 10.0):
                self.get_logger().error("[Node] get_route unavailable")
                return

        # FSM経由で初期ルート要求（Coreが保持するloopにタスク投入）
        self.get_logger().info("[Node] request initial route via FSM")
        fut = self.core.run_async(
            self.core.request_initial_route(
                self.start_label, self.goal_label, list(self.checkpoint_labels)
            )
        )
        fut.result()  # 同期的に完了を待つ


    # ------------------------------------------------------------------
    # Planner呼び出し実装（Coreへ注入する非同期関数）
    # ------------------------------------------------------------------
    async def _planner_get_async(self, start_label: str, goal_label: str, checkpoint_labels: List[str]):
        self.get_logger().info(f"[Node] call planner GetRoute: start='{start_label}', goal='{goal_label}', checkpoints={checkpoint_labels}")
        req = GetRoute.Request()
        req.start_label = start_label
        req.goal_label = goal_label
        req.checkpoint_labels = list(checkpoint_labels)
        future = self.cli_get.call_async(req)
        # 非同期完了待ち（タイムアウトはFSM側で適用）
        while not future.done():
            await asyncio.sleep(0.01)
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().info(f"[Node] planner GetRoute exception: {exc}")
            return type("Resp", (), {"success": False, "message": f"get_route exception: {exc}"})
        ok = bool(getattr(resp, "success", False))
        self.get_logger().info(f"[Node] planner GetRoute returned ok={ok}")
        route = getattr(resp, "route", None)
        if ok and route is not None:
            route_lite = ros_route_to_core(route)
            return type("Resp", (), {"success": True, "message": "ok", "route": route_lite})
        return type("Resp", (), {"success": False, "message": "invalid response"})

    async def _planner_update_async(
        self, major_version: int, prev_index: int, prev_label: str, next_index: Optional[int], next_label: str
    ):
        if not self.cli_update.wait_for_service(timeout_sec=self.get_parameter("planner_timeout_sec").value):
            self.get_logger().info("[Node] planner UpdateRoute unavailable")
            return type("Resp", (), {"success": False, "message": "update_route unavailable"})
        self.get_logger().info(f"[Node] call planner UpdateRoute: ver(major)={major_version}, prev=({prev_index},{prev_label}), next=({next_index},{next_label})")
        req = UpdateRoute.Request()
        req.route_version = int(major_version)
        req.prev_index = int(prev_index)
        req.prev_wp_label = str(prev_label)
        if next_index is not None:
            req.next_index = int(next_index)
            req.next_wp_label = str(next_label)
        future = self.cli_update.call_async(req)
        while not future.done():
            await asyncio.sleep(0.01)
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().info(f"[Node] planner UpdateRoute exception: {exc}")
            return type("Resp", (), {"success": False, "message": f"update_route exception: {exc}"})
        ok = bool(getattr(resp, "success", False))
        self.get_logger().info(f"[Node] planner UpdateRoute returned ok={ok}")
        route = getattr(resp, "route", None)
        if ok and route is not None:
            route_lite = ros_route_to_core(route)
            return type("Resp", (), {"success": True, "message": "ok", "route": route_lite})
        return type("Resp", (), {"success": False, "message": "invalid response"})

    # ------------------------------------------------------------------
    # 定期: RouteState のTick送信（Core由来の最新値を反映）
    # ------------------------------------------------------------------
    def _publish_route_state_tick(self) -> None:
        # Coreからのpublishで十分だが、低頻度での再送（冪等）を担保したい場合に維持
        pass

    def _publish_mission_info(self) -> None:
        msg = MissionInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.start_label = self.start_label
        msg.goal_label = self.goal_label
        msg.checkpoint_labels = list(self.checkpoint_labels)
        self.get_logger().info(f"[Node] publish /mission_info: start='{msg.start_label}', goal='{msg.goal_label}', checkpoints={list(self.checkpoint_labels)}")
        self.pub_mission_info.publish(msg)

    def destroy_node(self) -> None:
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RouteManagerNode()
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