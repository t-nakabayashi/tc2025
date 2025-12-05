#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""route_manager_node.py
Phase2 準拠・正式版（4段階 replan/shift/skip/failed を統合）。

本ファイルは**ROS2依存のラッパー**に責務を限定し、実処理は `manager_core.py` と
`manager_fsm.py` に委譲する形へリファクタリングした。

- /report_stuck サーバで、以下の順に判断する：
  1) UpdateRoute をまず試す（replan_first）
  2) shift（左右オフセットで次WPのみ横シフト）
  3) skip（次WPをスキップしてローカル再配信）
  4) failed（HOLDING）
- バージョン：Route.version = major*1000 + minor。planner へは major のみ送信。
- GetRoute は初期ルート取得にのみ使用（ReportStuck では使用しない）。
- Google Python Style + 型ヒント + 日本語コメント を付与。

※ 上記の仕様/コメントは、元の実装から**省略せず**に移植・維持している。
"""
from __future__ import annotations

import sys
from array import array
from pathlib import Path
import asyncio
import time
from concurrent.futures import Future
from typing import Any, Iterable, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
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
    RouteImageData,
    WaypointLite,
    Pose2D,
    VersionMM,
    StuckReport,
    FollowerStateUpdate,
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
def _image_msg_to_route_image_data(image: Optional[Image]) -> Optional[RouteImageData]:
    """sensor_msgs/Image を RouteImageData へ変換する。"""

    if image is None:
        return None
    try:
        height = int(getattr(image, "height", 0))
        width = int(getattr(image, "width", 0))
    except Exception:
        return None
    raw_data = getattr(image, "data", b"")
    data_bytes: bytes
    try:
        data_bytes = bytes(raw_data)
    except Exception:
        try:
            data_bytes = raw_data.tobytes()  # type: ignore[call-arg]
        except Exception:
            data_bytes = b""
    if height <= 0 or width <= 0 or not data_bytes:
        return None
    encoding = str(getattr(image, "encoding", ""))
    step = int(getattr(image, "step", 0) or 0)
    is_bigendian = int(getattr(image, "is_bigendian", 0) or 0)
    return RouteImageData(
        height=height,
        width=width,
        encoding=encoding,
        step=step,
        is_bigendian=is_bigendian,
        data=data_bytes,
    )


def ros_route_to_core(route: Route) -> RouteModel:
    """ROS Route -> Core RouteLite へ最小限の情報を移す。"""
    rm_wps = []
    version_int = int(getattr(route, "version", 0))
    frame_id = getattr(getattr(route, "header", None), "frame_id", "map") or "map"
    route_image = _image_msg_to_route_image_data(getattr(route, "route_image", None))
    has_image = route_image is not None
    for wp in getattr(route, "waypoints", []):
        orientation = getattr(getattr(wp, "pose", None), "orientation", None)
        w = WaypointLite(
            label=str(getattr(wp, "label", "")),
            pose=Pose2D(
                x=float(getattr(getattr(wp, "pose", None).position, "x", 0.0)),
                y=float(getattr(getattr(wp, "pose", None).position, "y", 0.0)),
                orientation_x=float(getattr(orientation, "x", 0.0)),
                orientation_y=float(getattr(orientation, "y", 0.0)),
                orientation_z=float(getattr(orientation, "z", 0.0)),
                orientation_w=float(getattr(orientation, "w", 1.0)),
            ),
            line_stop=bool(getattr(wp, "line_stop", False)),
            signal_stop=bool(getattr(wp, "signal_stop", False)),
            not_skip=bool(getattr(wp, "not_skip", False)),
            right_open=float(getattr(wp, "right_open", 0.0) or 0.0),
            left_open=float(getattr(wp, "left_open", 0.0) or 0.0),
            segment_is_fixed=bool(getattr(wp, "segment_is_fixed", False)),
        )
        rm_wps.append(w)
    start_index = int(getattr(route, "start_index", 0))
    if rm_wps and 0 <= start_index < len(rm_wps):
        current_index = start_index
        current_label = rm_wps[start_index].label
    else:
        current_index = 0 if rm_wps else -1
        current_label = rm_wps[0].label if rm_wps else ""
    start_label = str(
        getattr(route, "start_waypoint_label", getattr(route, "start_label", current_label))
    )
    if start_label:
        current_label = start_label
    return RouteModel(
        waypoints=rm_wps,
        version=VersionMM(major=version_int, minor=0),
        frame_id=frame_id,
        has_image=has_image,
        current_index=current_index,
        current_label=current_label,
        route_image=route_image,
    )


def core_route_to_ros(route: RouteModel) -> Route:
    """Core RouteLite -> ROS Route へ変換する。"""
    msg = Route()
    msg.header = Header()
    msg.header.frame_id = route.frame_id or "map"
    msg.version = int(route.version.to_int())
    msg.start_index = route.current_index
    msg.start_waypoint_label = route.current_label
    if route.route_image is not None and route.route_image.is_valid():
        img = Image()
        img.header = Header()
        img.header.frame_id = route.frame_id or "map"
        img.height = int(route.route_image.height)
        img.width = int(route.route_image.width)
        img.encoding = route.route_image.encoding
        img.step = int(route.route_image.step)
        img.is_bigendian = int(route.route_image.is_bigendian)
        img.data = array("B", route.route_image.data)
        msg.route_image = img
    elif route.has_image:
        # has_image が真だが実体が無い場合は後段で空画像とならないように空メッセージを置く。
        msg.route_image = Image()
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
            wp.segment_is_fixed = bool(w.segment_is_fixed)
            # 方位（planner由来のクォータニオンを維持する）
            wp.pose.orientation.x = float(getattr(w.pose, "orientation_x", 0.0))
            wp.pose.orientation.y = float(getattr(w.pose, "orientation_y", 0.0))
            wp.pose.orientation.z = float(getattr(w.pose, "orientation_z", 0.0))
            wp.pose.orientation.w = float(getattr(w.pose, "orientation_w", 1.0))
        except Exception:
            pass
        msg.waypoints.append(wp)
    return msg


# -----------------------------------------------------------------------------
# Node本体
# -----------------------------------------------------------------------------
class RouteManagerNode(Node):
    """RouteManager のROS2 I/F実装（Phase2・正式4段階版）。

    本ノードは「通信とI/F」に徹し、実処理は `RouteManagerCore` へ委譲する。
    """

    def __init__(self) -> None:
        super().__init__("route_manager")

        # ---------------- パラメータ宣言 ----------------
        self.declare_parameter("start_label", "")
        self.declare_parameter("goal_label", "")
        self.declare_parameter("checkpoint_labels", [])
        self.declare_parameter("planner_timeout_sec", 5.0)
        self.declare_parameter("planner_retry_count", 2)
        self.declare_parameter("planner_connect_timeout_sec", 10.0)
        self.declare_parameter("state_publish_rate_hz", 1.0)
        self.declare_parameter("image_encoding_check", False)
        self.declare_parameter("report_stuck_timeout_sec", 5.0)
        self.declare_parameter("offset_step_max_m", 1.0)  # shift 最大横ずれ[m]

        # ---------------- パラメータ取得 ----------------
        self.start_label: str = (
            self.get_parameter("start_label").get_parameter_value().string_value.strip()
        )
        self.goal_label: str = (
            self.get_parameter("goal_label").get_parameter_value().string_value.strip()
        )
        self.checkpoint_labels: List[str] = self._resolve_checkpoint_labels()
        self.timeout_sec: float = float(self.get_parameter("planner_timeout_sec").get_parameter_value().double_value)
        self.retry_count: int = int(self.get_parameter("planner_retry_count").get_parameter_value().integer_value)
        self.connect_timeout_sec: float = float(
            self.get_parameter("planner_connect_timeout_sec").get_parameter_value().double_value
        )
        self.state_rate_hz: float = float(self.get_parameter("state_publish_rate_hz").get_parameter_value().double_value)
        self.image_encoding_check: bool = self.get_parameter("image_encoding_check").get_parameter_value().bool_value
        self.report_stuck_timeout_sec: float = float(
            self.get_parameter("report_stuck_timeout_sec").get_parameter_value().double_value
        )
        self.offset_step_max_m: float = float(
            self.get_parameter("offset_step_max_m").get_parameter_value().double_value
        )
        planner_get_service = 'get_route'
        planner_update_service = 'update_route'
        active_route_topic = 'active_route'
        route_state_topic = 'route_state'
        mission_info_topic = 'mission_info'
        manager_status_topic = 'manager_status'
        follower_state_topic = 'follower_state'
        report_stuck_service = 'report_stuck'

        # ---------------- QoS ----------------
        self.qos_tl = qos_tl()
        self.qos_stream = qos_vol()

        # ---------------- Publisher ----------------
        self.pub_active_route = self.create_publisher(Route, active_route_topic, self.qos_tl)
        self.pub_route_state = self.create_publisher(RouteState, route_state_topic, self.qos_stream)
        self.pub_mission_info = self.create_publisher(MissionInfo, mission_info_topic, self.qos_tl)
        self.pub_manager_status = self.create_publisher(ManagerStatus, manager_status_topic, self.qos_stream)

        # ---------------- Subscriber ----------------
        self.sub_follower_state = self.create_subscription(
            FollowerState,
            follower_state_topic,
            self._on_follower_state,
            self.qos_stream,
        )

        # ---------------- Service Clients ----------------
        self.cb_cli = MutuallyExclusiveCallbackGroup()
        self.cli_get = self.create_client(GetRoute, planner_get_service, callback_group=self.cb_cli)
        self.cli_update = self.create_client(UpdateRoute, planner_update_service, callback_group=self.cb_cli)

        # ---------------- Service Server ----------------
        self.cb_srv = MutuallyExclusiveCallbackGroup()
        self.srv_report_stuck = self.create_service(
            ReportStuck, report_stuck_service, self._on_report_stuck, callback_group=self.cb_srv
        )

        # リマップを考慮した名称をログ用に保持
        self.srv_get_name = self._resolve_service_name(planner_get_service)
        self.srv_update_name = self._resolve_service_name(planner_update_service)
        self.active_route_topic = self._resolve_topic_name(active_route_topic)
        self.route_state_topic = self._resolve_topic_name(route_state_topic)
        self.mission_info_topic = self._resolve_topic_name(mission_info_topic)
        self.manager_status_topic = self._resolve_topic_name(manager_status_topic)
        self.follower_state_topic = self._resolve_topic_name(follower_state_topic)
        self.report_stuck_service_name = self._resolve_service_name(report_stuck_service)

        # ---------------- Core 構築 ----------------
        self.core = RouteManagerCore(
            logger=lambda m: self.get_logger().info(m),
            publish_active_route=self._publish_active_route_from_core,
            publish_status=self._publish_status_from_core,
            publish_route_state=self._publish_route_state_from_core,
            offset_step_max_m=self.offset_step_max_m,
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

        # Publishログの間引き用タイムスタンプおよび最新状態を保持する。
        self._last_status_log_time: float = 0.0
        self._last_route_state_log_time: float = 0.0
        self._last_status_publish_time: float = 0.0
        self._last_route_state_publish_time: float = 0.0
        self._latest_status_payload: Optional[
            tuple[str, str, str, int]
        ] = None
        self._latest_route_state_payload: Optional[
            tuple[int, str, int, int, str, str]
        ] = None
        # 1Hzで最新状態を再送するためのタイマーを追加する。
        self._state_snapshot_timer = self.create_timer(0.2, self._publish_state_snapshots)
        self._last_report_pose: Optional[PoseStamped] = None

    def _resolve_topic_name(self, name: str) -> str:
        """リマップ適用後のトピック名を返す補助関数."""
        try:
            return self.resolve_topic_name(name)
        except AttributeError:
            return name

    def _resolve_service_name(self, name: str) -> str:
        """リマップ適用後のサービス名を返す補助関数."""
        try:
            return self.resolve_service_name(name)
        except AttributeError:
            return name

    def _resolve_checkpoint_labels(self) -> List[str]:
        """チェックポイントパラメータを配列へ正規化する。"""

        param = self.get_parameter("checkpoint_labels")
        if param.type_ == Parameter.Type.STRING_ARRAY:
            raw_values = list(param.get_parameter_value().string_array_value)
            return [label.strip() for label in raw_values if label.strip()]
        if param.type_ == Parameter.Type.STRING:
            text = param.get_parameter_value().string_value
            return self._parse_checkpoint_text(text)
        # YAMLで不明な型が渡された場合はフォールバックとして iterable を走査する。
        raw_value = param.get_parameter_value()
        candidate: Iterable[str]
        try:
            candidate = list(raw_value.string_array_value)
        except AttributeError:
            candidate = []
        return [label.strip() for label in candidate if isinstance(label, str) and label.strip()]

    @staticmethod
    def _parse_checkpoint_text(raw_value: str) -> List[str]:
        """文字列で与えられたチェックポイント一覧を分割する。"""

        text = (raw_value or "").replace("\n", ",").strip()
        if not text:
            return []
        if text.startswith("[") and text.endswith("]"):
            text = text[1:-1]
        labels: List[str] = []
        for chunk in text.split(","):
            label = chunk.strip().strip("\"'")
            if label:
                labels.append(label)
        return labels

    def _add_future_logging(self, future: Future, context: str) -> None:
        """Future完了時に例外を検知してログ出力する補助関数."""

        def _done_callback(done_future: Future) -> None:
            try:
                done_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"[Node] {context} で例外が発生しました: {exc}")

        future.add_done_callback(_done_callback)

    def _on_follower_state(self, msg: FollowerState) -> None:
        """RouteFollower からの状態通知を受け取りCoreへ伝搬する."""

        update = FollowerStateUpdate(
            route_version=int(getattr(msg, "route_version", 0)),
            state=str(getattr(msg, "state", "")),
            active_waypoint_index=int(getattr(msg, "active_waypoint_index", -1)),
            active_waypoint_label=str(getattr(msg, "active_waypoint_label", "")),
        )

        if (update.state or "").strip().upper() == "FINISHED":
            self.get_logger().info(
                f"[Node] {self.follower_state_topic}: FINISHED を検知しました。FSMへ完了通知を送ります。"
            )

        future = self.core.run_async(self.core.on_follower_state_update(update))
        self._add_future_logging(future, "core.on_follower_state_update")

    # ------------------------------------------------------------------
    # Core -> Node: Publish 実装（ROSメッセージへ変換して配信）
    # ------------------------------------------------------------------
    def _publish_active_route_from_core(self, route: RouteModel) -> None:
        self.get_logger().info(
            f"[Node] publish {self.active_route_topic}: version={int(route.version.to_int())}, "
            f"waypoints={len(route.waypoints)}"
        )
        ros_route = core_route_to_ros(route)
        self.pub_active_route.publish(ros_route)

    def _publish_status_from_core(self, state: str, decision: str, cause: str, route_version: int) -> None:
        payload = (state, decision, cause, int(route_version))
        self._latest_status_payload = payload
        self._emit_manager_status(payload, force_log=True)

    @staticmethod
    def _normalize_route_state_status(status: str) -> int:
        """RouteState.status に格納する列挙値へ変換する。"""
        normalized = (status or "").strip().lower()
        mapping = {
            "": RouteState.STATUS_UNKNOWN,
            "unknown": RouteState.STATUS_UNKNOWN,
            "idle": RouteState.STATUS_IDLE,
            "running": RouteState.STATUS_RUNNING,
            "active": RouteState.STATUS_RUNNING,
            "requesting": RouteState.STATUS_RUNNING,
            "updating": RouteState.STATUS_UPDATING_ROUTE,
            "updating_route": RouteState.STATUS_UPDATING_ROUTE,
            "waiting_reroute": RouteState.STATUS_HOLDING,
            "holding": RouteState.STATUS_HOLDING,
            "completed": RouteState.STATUS_COMPLETED,
            "finished": RouteState.STATUS_COMPLETED,
            "error": RouteState.STATUS_ERROR,
            "failed": RouteState.STATUS_ERROR,
        }
        return mapping.get(normalized, RouteState.STATUS_UNKNOWN)

    def _publish_route_state_from_core(
        self,
        idx: int,
        label: str,
        ver: int,
        total: int,
        status: str,
        message: str,
    ) -> None:
        payload = (
            int(idx),
            str(label),
            int(ver),
            int(total),
            str(status),
            str(message or ""),
        )
        self._latest_route_state_payload = payload
        self._emit_route_state(payload, force_log=True)

    @staticmethod
    def _should_log(now_sec: float, last_log_time: float) -> bool:
        """1秒間隔でログ出力するべきかを判定する。"""
        if last_log_time == 0.0:
            return True
        return now_sec - last_log_time >= 1.0

    def _log_manager_status(self, payload: tuple[str, str, str, int]) -> None:
        """ManagerStatusの内容をinfoログとして出力する。"""
        state, decision, cause, route_version = payload
        self.get_logger().info(
            f"[Node] publish {self.manager_status_topic}: state={state}, decision={decision}, "
            f"cause={cause}, ver={route_version}"
        )

    def _log_route_state(self, payload: tuple[int, str, int, int, str, str]) -> None:
        """RouteStateの内容をinfoログとして出力する。"""
        idx, label, ver, total, status, message = payload
        self.get_logger().info(
            f"[Node] publish {self.route_state_topic}: idx={idx}, label='{label}', "
            f"ver={ver}, total={total}, status={status}, message='{message}'"
        )

    def _emit_manager_status(
        self, payload: tuple[str, str, str, int], force_log: bool = False
    ) -> None:
        """ManagerStatusをpublishし、ログを1Hzに間引く。"""
        msg = ManagerStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state, msg.decision, msg.last_cause, msg.route_version = payload
        now_sec = time.monotonic()
        self.pub_manager_status.publish(msg)
        if force_log or self._should_log(now_sec, self._last_status_log_time):
            self._log_manager_status(payload)
            self._last_status_log_time = now_sec
        self._last_status_publish_time = now_sec

    def _emit_route_state(
        self, payload: tuple[int, str, int, int, str, str], force_log: bool = False
    ) -> None:
        """RouteStateをpublishし、ログを1Hzに間引く。"""
        msg = RouteState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        idx, label, ver, total, status, message = payload
        msg.status = self._normalize_route_state_status(status)
        msg.current_index = idx
        msg.current_label = label
        msg.route_version = ver
        msg.total_waypoints = total
        msg.message = message
        now_sec = time.monotonic()
        self.pub_route_state.publish(msg)
        if force_log or self._should_log(now_sec, self._last_route_state_log_time):
            self._log_route_state(payload)
            self._last_route_state_log_time = now_sec
        self._last_route_state_publish_time = now_sec

    def _publish_state_snapshots(self) -> None:
        """最新状態を1Hz間隔で再送してログとpublish回数を一致させる。"""
        now_sec = time.monotonic()
        if (
            self._latest_status_payload is not None
            and self._should_log(now_sec, self._last_status_publish_time)
        ):
            self._emit_manager_status(self._latest_status_payload)
        if (
            self._latest_route_state_payload is not None
            and self._should_log(now_sec, self._last_route_state_publish_time)
        ):
            self._emit_route_state(self._latest_route_state_payload)

    @staticmethod
    def _copy_pose_stamped(src: PoseStamped) -> PoseStamped:
        """PoseStampedメッセージを新規インスタンスへ複製する。"""

        copied = PoseStamped()
        copied.header = Header()
        if hasattr(src, "header") and hasattr(src.header, "stamp"):
            copied.header.stamp.sec = int(getattr(src.header.stamp, "sec", 0))
            copied.header.stamp.nanosec = int(getattr(src.header.stamp, "nanosec", 0))
        copied.header.frame_id = str(getattr(src.header, "frame_id", ""))
        pose_field = getattr(src, "pose", None)
        position = getattr(pose_field, "position", None)
        orientation = getattr(pose_field, "orientation", None)
        if position is not None:
            copied.pose.position.x = float(getattr(position, "x", 0.0))
            copied.pose.position.y = float(getattr(position, "y", 0.0))
            copied.pose.position.z = float(getattr(position, "z", 0.0))
        if orientation is not None:
            copied.pose.orientation.x = float(getattr(orientation, "x", 0.0))
            copied.pose.orientation.y = float(getattr(orientation, "y", 0.0))
            copied.pose.orientation.z = float(getattr(orientation, "z", 0.0))
            copied.pose.orientation.w = float(getattr(orientation, "w", 1.0))
        else:
            copied.pose.orientation.w = 1.0
        return copied

    # ------------------------------------------------------------------
    # Service Server: report_stuck（Core+FSMで4段階ロジックを維持）
    # ------------------------------------------------------------------
    def _on_report_stuck(self, req: ReportStuck.Request, res: ReportStuck.Response) -> ReportStuck.Response:
        pose_msg = getattr(req, "current_pose", None)
        if isinstance(pose_msg, PoseStamped):
            self._last_report_pose = self._copy_pose_stamped(pose_msg)
        else:
            self._last_report_pose = None
        pose_field = getattr(pose_msg, "pose", None)
        position = getattr(pose_field, "position", None)
        orientation = getattr(pose_field, "orientation", None)
        pose2d = Pose2D(
            x=float(getattr(position, "x", 0.0)),
            y=float(getattr(position, "y", 0.0)),
            orientation_x=float(getattr(orientation, "x", 0.0)),
            orientation_y=float(getattr(orientation, "y", 0.0)),
            orientation_z=float(getattr(orientation, "z", 0.0)),
            orientation_w=float(getattr(orientation, "w", 1.0)),
        )
        report = StuckReport(
            route_version=int(getattr(req, "route_version", 0)),
            current_index=int(getattr(req, "current_index", -1)),
            current_label=str(getattr(req, "current_wp_label", "") or ""),
            current_pose=pose2d,
            reason_code=int(getattr(req, "reason_code", ReportStuck.Request.REASON_UNKNOWN)),
            reason_detail=str(getattr(req, "reason_detail", "")),
            avoid_trial_count=int(getattr(req, "avoid_trial_count", 0)),
            last_hint_blocked=bool(getattr(req, "last_hint_blocked", False)),
            last_applied_offset_m=float(getattr(req, "last_applied_offset_m", 0.0)),
        )

        self.get_logger().info(
            f"[Node] {self.report_stuck_service_name}: received -> delegate to Core/FSM "
            f"idx={report.current_index} label='{report.current_label}' reason='{report.reason_label()}' "
            f"(code={report.reason_code})"
        )
        # Coreのイベントループ上でFSM処理を非同期実行し、結果を同期的に取得
        result = self.core.run_async(self.core.on_report_stuck(report)).result()
        offset_hint = float(self.core.get_last_offset_hint())

        # Coreの決定内容をReportStuck.Responseに整形
        note = getattr(result, "message", "")
        if getattr(result, "success", False):
            # replan/shift/skip の区別は note で示す（元実装のコメントを維持）
            if note == "skipped":
                res.decision_code = ReportStuck.Response.DECISION_SKIP
            else:
                res.decision_code = ReportStuck.Response.DECISION_REPLAN
            res.note = note
            res.waiting_deadline = Duration(sec=0, nanosec=200 * 10**6)
            res.offset_hint = offset_hint
            self.get_logger().info(
                f"[Node] {self.report_stuck_service_name}: success decision_code={res.decision_code} note='{note}'"
            )
        else:
            res.decision_code = ReportStuck.Response.DECISION_FAILED
            res.note = note or "avoidance_failed"
            res.waiting_deadline = Duration(sec=0, nanosec=0)
            res.offset_hint = 0.0
            self.get_logger().info(
                f"[Node] {self.report_stuck_service_name}: failed note='{res.note}'"
            )
        return res

    # ------------------------------------------------------------------
    # 起動直後
    # ------------------------------------------------------------------
    def _on_ready_once(self):
        """起動後一度だけFSMに初期ルート要求を投げる。"""
        if getattr(self, '_once_done', False):
            return
        self._once_done = True

        # ROSサービス接続待ち
        start = time.time()
        while not self.cli_get.wait_for_service(timeout_sec=0.2):
            if time.time() - start > self.connect_timeout_sec:
                self.get_logger().error(f"[Node] {self.srv_get_name} unavailable")
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
            return type("Resp", (), {"success": False, "message": f"{self.srv_get_name} exception: {exc}"})
        ok = bool(getattr(resp, "success", False))
        self.get_logger().info(f"[Node] planner GetRoute returned ok={ok}")
        route = getattr(resp, "route", None)
        if ok and route is not None:
            route_lite = ros_route_to_core(route)
            return type("Resp", (), {"success": True, "message": "ok", "route": route_lite})
        return type("Resp", (), {"success": False, "message": "invalid response"})

    async def _planner_update_async(
        self,
        major_version: int,
        prev_index: int,
        prev_label: str,
        current_index: int,
        current_label: str,
    ):
        if not self.cli_update.wait_for_service(timeout_sec=self.timeout_sec):
            self.get_logger().info(f"[Node] planner UpdateRoute unavailable ({self.srv_update_name})")
            return type("Resp", (), {"success": False, "message": f"{self.srv_update_name} unavailable"})
        self.get_logger().info(
            "[Node] call planner UpdateRoute: "
            f"ver(major)={major_version}, prev=({prev_index},{prev_label}), "
            f"current=({current_index},{current_label})"
        )
        req = UpdateRoute.Request()
        req.route_version = int(major_version)
        req.prev_index = int(prev_index)
        req.prev_wp_label = str(prev_label)
        req.current_index = int(current_index)
        req.current_wp_label = str(current_label)
        if self._last_report_pose is not None:
            req.current_pose = self._copy_pose_stamped(self._last_report_pose)
        future = self.cli_update.call_async(req)
        while not future.done():
            await asyncio.sleep(0.01)
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().info(f"[Node] planner UpdateRoute exception: {exc}")
            return type("Resp", (), {"success": False, "message": f"{self.srv_update_name} exception: {exc}"})
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
        self.get_logger().info(
            f"[Node] publish {self.mission_info_topic}: start='{msg.start_label}', "
            f"goal='{msg.goal_label}', checkpoints={list(self.checkpoint_labels)}"
        )
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