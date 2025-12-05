#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manager_core.py
route_manager の**非ROS依存**コア実装。

- Node層（ROS2依存）は通信・入出力に限定し、すべての業務ロジックを本モジュールへ集約する。
- 既存の `route_manager_node.py` に含まれていたロジック（Update→SHIFT→SKIP→failed の再計画、
  ルート受理・バージョン管理、幾何計算など）を、**コメントも含め省略や要約を行わず移植**する。
- FSM（manager_fsm.py）と連携し、初期GetRouteやReportStuckを非同期に安全実行する。

注意:
- 本モジュールはROSメッセージ型に依存しない。Node層からは、
  - 「プランナに対するGet/Updateの非同期呼び出し」
  - 「ActiveRoute/Status/RouteStateのPublish」
  の**コールバック**が注入される。
- Google Python Style / 型ヒントを遵守し、日本語コメントを丁寧に記載する。
"""

from __future__ import annotations

import copy
import asyncio
import math
import threading
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Awaitable, Callable, Dict, List, Optional, Protocol, Set, Tuple

from manager_fsm import RouteManagerFSM, SimpleServiceResult, ServiceResult

# =============================================================================
# 非ROS依存データモデル
# =============================================================================

class StuckReason(IntEnum):
    """ReportStuckで通知される滞留理由コードの内部表現."""

    UNKNOWN = 0
    FRONT_BLOCKED = 1
    ROAD_BLOCKED = 2
    NO_HINT = 3
    NO_SPACE = 4
    AVOIDANCE_FAILED = 5


_STUCK_REASON_LABELS: Dict[StuckReason, str] = {
    StuckReason.UNKNOWN: 'unknown',
    StuckReason.FRONT_BLOCKED: 'front_blocked',
    StuckReason.ROAD_BLOCKED: 'road_blocked',
    StuckReason.NO_HINT: 'no_hint',
    StuckReason.NO_SPACE: 'no_space',
    StuckReason.AVOIDANCE_FAILED: 'avoidance_failed',
}


@dataclass
class VersionMM:
    """バージョンを major/minor で保持する軽量表現。

    元実装（route_manager_node.py）のコメント/ロジックを**省略せずに**移植:
    - バージョン：Route.version = major*100 + minor。planner へは major のみ送信。
    """
    major: int = 0
    minor: int = 0

    def to_int(self) -> int:
        """上位2桁がmajor、下位2桁がminorになるよう整数化する。"""
        return ((int(self.major) % 100) * 100) + (int(self.minor) % 100)

    @classmethod
    def from_int(cls, value: int) -> "VersionMM":
        """整数値を major/minor に分解する。"""
        v = int(value)
        major = int(v / 100)
        minor = v % 100
        return cls(major=major, minor=minor)


@dataclass
class Pose2D:
    """2D位置（方位を含む最小限の姿勢表現）。"""
    x: float = 0.0
    y: float = 0.0
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 1.0


@dataclass
class RouteImageData:
    """Route に埋め込む画像データをROS非依存で保持する。"""

    height: int = 0
    width: int = 0
    encoding: str = ""
    step: int = 0
    is_bigendian: int = 0
    data: bytes = field(default_factory=bytes)

    def is_valid(self) -> bool:
        """画像データが有効かどうかを判定する。"""

        return self.height > 0 and self.width > 0 and bool(self.data)


@dataclass
class StuckReport:
    """/report_stuck で受け取る情報をCore内部で扱いやすい形にしたもの。"""

    route_version: int
    current_index: int
    current_label: str
    current_pose: Pose2D
    reason_code: int
    reason_detail: str
    avoid_trial_count: int
    last_hint_blocked: bool
    last_applied_offset_m: float

    def reason_label(self) -> str:
        """理由コードと詳細文字列から内部で使用するラベルを返す。"""
        if self.reason_detail:
            return str(self.reason_detail)
        try:
            reason = StuckReason(int(self.reason_code))
        except ValueError:
            return 'unknown'
        return _STUCK_REASON_LABELS.get(reason, 'unknown')


@dataclass
class WaypointLite:
    """必要最小限のWP表現。元実装の属性を反映（省略無し）。"""
    label: str = ""
    pose: Pose2D = field(default_factory=Pose2D)
    line_stop: bool = False
    signal_stop: bool = False
    not_skip: bool = False
    right_open: float = 0.0
    left_open: float = 0.0
    segment_is_fixed: bool = False


@dataclass
class FollowerStateUpdate:
    """RouteFollower からの進捗通知を非ROS構造体として保持する。"""

    route_version: int = 0
    state: str = ""
    active_waypoint_index: int = -1
    active_waypoint_label: str = ""


@dataclass
class FollowerEvent:
    """FSMへ渡すフォロワ状態更新イベント."""

    finished: bool = False



@dataclass
class RouteModel:
    """GetRoute/UpdateRouteで取得した経路を構造的に保持する軽量モデル。

    元の `route_manager_node.py` の実装/コメントを移植（省略なし）。
    """
    waypoints: List[WaypointLite]
    version: VersionMM
    frame_id: str = "map"
    has_image: bool = True
    current_index: int = 0
    current_label: str = ""
    route_image: Optional[RouteImageData] = None

    def __post_init__(self) -> None:
        """画像有無フラグを保持している画像と同期する。"""

        if self.route_image is not None and not self.route_image.is_valid():
            # 無効な画像は破棄し、フラグを整合させる。
            self.route_image = None
        self.has_image = self.route_image is not None

    @classmethod
    def from_route(cls, route_like) -> "RouteModel":
        """Route互換オブジェクトからRouteModelを生成する。"""
        # 既にRouteModelならコピーして返す
        if isinstance(route_like, cls):
            rm = route_like
            return cls(
                waypoints=list(getattr(rm, "waypoints", [])),
                version=VersionMM(major=int(getattr(rm.version, "major", 0)), minor=int(getattr(rm.version, "minor", 0))),
                frame_id=str(getattr(rm, "frame_id", "map")),
                has_image=bool(getattr(rm, "route_image", None) is not None or getattr(rm, "has_image", False)),
                current_index=int(getattr(rm, "current_index", 0)),
                current_label=str(getattr(rm, "current_label", "")),
                route_image=copy.deepcopy(getattr(rm, "route_image", None)),
            )
        # route_likeがRouteLite相当（version:int, waypoints:List）ならそこから生成
        wps = list(getattr(route_like, "waypoints", []))
        raw_version = getattr(route_like, "version", 0)
        try:
            ver_int = int(raw_version)
        except Exception:
            ver_int = 0
        ver = VersionMM.from_int(ver_int)
        cur_idx = 0
        cur_lbl = wps[0].label if wps else ""
        route_image = cls._extract_route_image(getattr(route_like, "route_image", None))
        return cls(
            waypoints=wps,
            version=ver,
            frame_id=str(
                getattr(
                    getattr(route_like, "header", None), "frame_id", getattr(route_like, "frame_id", "map")
                )
            ),
            has_image=route_image is not None,
            current_index=cur_idx,
            current_label=cur_lbl,
            route_image=route_image,
        )

    @staticmethod
    def _extract_route_image(image_obj: Any) -> Optional["RouteImageData"]:
        """Route互換オブジェクトに含まれる画像をRouteImageDataへ変換する。"""

        if image_obj is None:
            return None
        try:
            height = int(getattr(image_obj, "height", 0))
            width = int(getattr(image_obj, "width", 0))
        except Exception:
            return None
        raw_data = getattr(image_obj, "data", b"")
        try:
            if hasattr(raw_data, "tobytes"):
                data_bytes = raw_data.tobytes()  # type: ignore[call-arg]
            else:
                data_bytes = bytes(raw_data)
        except Exception:
            data_bytes = b""
        if height <= 0 or width <= 0 or not data_bytes:
            return None
        encoding = str(getattr(image_obj, "encoding", ""))
        step = int(getattr(image_obj, "step", 0) or 0)
        is_bigendian = int(getattr(image_obj, "is_bigendian", 0) or 0)
        return RouteImageData(
            height=height,
            width=width,
            encoding=encoding,
            step=step,
            is_bigendian=is_bigendian,
            data=data_bytes,
        )

    def total(self) -> int:
        return len(self.waypoints)

    def label_at(self, index: int) -> Optional[str]:
        if 0 <= index < len(self.waypoints):
            return str(getattr(self.waypoints[index], "label", "")) or None
        return None

    def prev_index(self) -> Optional[int]:
        prv = self.current_index - 1
        return prv if 0 <= prv < len(self.waypoints) else None

    def next_index(self) -> Optional[int]:
        nxt = self.current_index + 1
        return nxt if 0 <= nxt < len(self.waypoints) else None

    def advance_to(self, index: Optional[int] = None, label: Optional[str] = None) -> None:
        """FollowerState に合わせて進捗を更新する。"""
        if index is not None and 0 <= index < len(self.waypoints):
            self.current_index = index
            self.current_label = self.label_at(index) or self.current_label
            return
        if label:
            for i, wp in enumerate(self.waypoints):
                if str(getattr(wp, "label", "")) == label:
                    self.current_index = i
                    self.current_label = label
                    return

    def slice_from(self, start_index: int) -> "RouteModel":
        """start_index から末尾までの部分ルートを返す（RouteModel）。"""
        new_wps = list(self.waypoints[start_index:]) if 0 <= start_index < len(self.waypoints) else []
        return RouteModel(
            waypoints=new_wps,
            version=VersionMM(major=self.version.major, minor=self.version.minor),
            frame_id=self.frame_id,
            has_image=self.has_image,
            current_index=0 if new_wps else -1,
            current_label=(new_wps[0].label if new_wps else ""),
            route_image=copy.deepcopy(self.route_image),
        )

    def is_completed(self) -> bool:
        """経路の完走を判定する。"""
        # 総waypoint数が0の場合は完走扱いしない
        if not self.waypoints:
            return False
        # current_index が最終インデックスに到達したかどうか
        return self.current_index >= len(self.waypoints) - 1

# 幾何ユーティリティ（shift用）：元実装のロジックを移植（省略無し）。
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def seg_vec(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return (b[0] - a[0], b[1] - a[1])


def seg_len(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    vx, vy = seg_vec(a, b)
    return math.hypot(vx, vy)


def unit_normal(a: Tuple[float, float], b: Tuple[float, float], right_side: bool) -> Tuple[float, float]:
    """AB に直交する単位法線ベクトルを返す。right_side=True で右向き。"""
    vx, vy = seg_vec(a, b)
    n = math.hypot(vx, vy) or 1.0
    if right_side:
        return (vy / n, -vx / n)
    return (-vy / n, vx / n)


# =============================================================================
# コールバックI/F（Node層から注入）
# =============================================================================

class PlannerGetCb(Protocol):
    async def __call__(self, start_label: str, goal_label: str, checkpoint_labels: List[str]) -> ServiceResult: ...


class PlannerUpdateCb(Protocol):
    async def __call__(
        self,
        major_version: int,
        prev_index: int,
        prev_label: str,
        current_index: int,
        current_label: str,
    ) -> ServiceResult:
        ...


PublishActiveRoute = Callable[[RouteModel], None]
PublishStatus = Callable[[str, str, str, int], None]
PublishRouteState = Callable[[int, str, int, int, str, str], None]  # idx, label, ver, total, status_name, message


# =============================================================================
# Core本体
# =============================================================================

class RouteManagerCore:
    """RouteManager の非ROS依存コア。

    - FSMの生成・駆動
    - 4段階 replan/shift/skip/failed の完全実装
    - ルート受理・バージョン管理・状態更新
    - Node層からの各種コールバック注入
    """
    def __init__(
        self,
        logger: Callable[[str], None],
        publish_active_route: PublishActiveRoute,
        publish_status: PublishStatus,
        publish_route_state: PublishRouteState,
        offset_step_max_m: float = 1.5,
    ) -> None:
        # 依存注入
        self._logger_func = logger
        self._publish_active_route = publish_active_route
        self._publish_status = publish_status
        self._publish_route_state = publish_route_state

        # パラメータ
        self.offset_step_max_m = float(offset_step_max_m)

        # 内部状態
        self.route_model: Optional[RouteModel] = None
        self.current_index: int = -1
        self.current_label: str = ""
        self.current_status: str = "IDLE"
        self.current_segment_is_fixed: bool = False
        self._skip_history: Dict[str, int] = {}
        self._skipped_indices: Set[int] = set()
        # 1回のローカル再計画シーケンスで許可するSKIP回数は1回のみ。
        # planner由来の新ルートを受理した時点でリセットする。
        self._skip_allowed: bool = True
        self._shift_preference: Dict[str, bool] = {}
        self._last_stuck_report: Optional[StuckReport] = None
        self._last_known_pose: Optional[Pose2D] = None
        self._last_replan_offset_hint: float = 0.0
        self._last_route_message: str = ""

        # FSM 構築（timeoutはNode層から調整可能）
        self.fsm = RouteManagerFSM(
            logger=self._log,
            get_timeout_sec=5.0,
            update_timeout_sec=5.0,
            replan_timeout_sec=5.0,
        )
        self.fsm.add_transition_hook(self._on_transition)

        # Planner呼び出し関数（Node層注入）
        self._planner_get: Optional[PlannerGetCb] = None
        self._planner_update: Optional[PlannerUpdateCb] = None

        # FSMへコールバック登録（Core内のメソッドを渡す）
        self.fsm.set_get_callback(self._cb_get_route)
        self.fsm.set_update_callback(self._cb_update_route)
        self.fsm.set_replan_callback(self._cb_replan)

        # --- Core専用asyncioイベントループ（FSMと共有）を起動（非ROS） ---
        # Node層からはこのループへ run_coroutine_threadsafe で投げ込むだけにして、
        # タイマー/スレッドとの競合やイベントループの二重生成を防ぐ。
        self.loop = asyncio.new_event_loop()
        def _core_loop_runner() -> None:
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()
        self._loop_thread = threading.Thread(target=_core_loop_runner, daemon=True)
        self._loop_thread.start()

    # ------------------------------------------------------------------
    # Node層からの依存注入
    # ------------------------------------------------------------------
    def _log(self, message: object, *parts: object) -> None:
        """注入されたロガーを用いて複数引数のログ出力を一つにまとめる。"""

        text = str(message)
        if parts:
            text += "".join(str(part) for part in parts)
        self._logger_func(text)

    def set_planner_callbacks(self, get_cb: PlannerGetCb, update_cb: PlannerUpdateCb) -> None:
        """プランナサービス呼び出し用のコールバックを設定する。"""
        self._planner_get = get_cb
        self._planner_update = update_cb
        self._log("[Core] planner callbacks set")

    # ------------------------------------------------------------------
    # 外部I/F（Node層から呼ばれるAPI）
    # ------------------------------------------------------------------

    def run_async(self, coro: asyncio.coroutines) -> 'concurrent.futures.Future':
        """他スレッド（Node側）からCoreのイベントループへ安全にコルーチンを投入する。
        返り値は concurrent.futures.Future で、.result() により同期待機が可能。
        """
        import concurrent.futures
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def _sync_from_stuck_report(self, report: StuckReport) -> None:
        """/report_stuck の内容を内部状態に反映する。"""
        self._last_stuck_report = report
        self.current_index = int(report.current_index)
        self.current_label = str(report.current_label or "")
        self._last_known_pose = Pose2D(
            x=float(report.current_pose.x),
            y=float(report.current_pose.y),
            orientation_x=float(getattr(report.current_pose, "orientation_x", 0.0)),
            orientation_y=float(getattr(report.current_pose, "orientation_y", 0.0)),
            orientation_z=float(getattr(report.current_pose, "orientation_z", 0.0)),
            orientation_w=float(getattr(report.current_pose, "orientation_w", 1.0)),
        )

        if self.route_model is not None:
            if int(self.route_model.version.to_int()) != int(report.route_version):
                self._log(
                    f"[Core] _sync_from_stuck_report: version mismatch local={int(self.route_model.version.to_int())} "
                    f"report={int(report.route_version)}"
                )
            self.route_model.advance_to(index=self.current_index, label=self.current_label)
        self._update_current_segment_flag()

        self._log(
            f"[Core] _sync_from_stuck_report: idx={self.current_index}, label='{self.current_label}', "
            f"pose=({self._last_known_pose.x:.3f},{self._last_known_pose.y:.3f})"
        )

        self._emit_route_state(
            index=int(self.route_model.current_index if self.route_model else self.current_index),
            label=str(self.route_model.current_label if self.route_model else self.current_label),
            version=int(self.route_model.version.to_int() if self.route_model else report.route_version),
            total=int(self.route_model.total() if self.route_model else 0),
            status=str(self.current_status),
        )

    def _update_current_segment_flag(self) -> None:
        """現在ターゲットのWaypointが固定区間かどうかのフラグを更新する。"""
        fixed = False
        if self.route_model is not None:
            idx = int(getattr(self.route_model, "current_index", -1))
            if 0 <= idx < len(self.route_model.waypoints):
                cur_wp = self.route_model.waypoints[idx]
                fixed = bool(getattr(cur_wp, "segment_is_fixed", False))
        self.current_segment_is_fixed = fixed

    def get_last_offset_hint(self) -> float:
        """直近の再計画で決定した横シフト量（ヒント）を返す。"""
        return float(self._last_replan_offset_hint)

    def is_current_segment_fixed(self) -> bool:
        """現在ターゲットの区間が固定ルートかどうかを返す。"""
        return bool(self.current_segment_is_fixed)

    async def request_initial_route(self, start_label: str, goal_label: str, checkpoint_labels: List[str]) -> ServiceResult:
        """初期ルートをFSM経由で要求する。"""
        self._log(f"[Core] request_initial_route: start='{start_label}', goal='{goal_label}', checkpoints={checkpoint_labels}")
        # FSMへ渡すデータはコールバック側で参照するため、Coreに保持
        self._initial_start = start_label
        self._initial_goal = goal_label
        self._initial_checkpoints = list(checkpoint_labels)
        return await self.fsm.handle_event(RouteManagerFSM.E_REQUEST_INITIAL_ROUTE, None)

    async def on_report_stuck(self, report: StuckReport) -> ServiceResult:
        """ReportStuck受付時の処理をFSMに委譲し、情報を同期する。"""
        self._log("[Core] on_report_stuck: received")
        self._sync_from_stuck_report(report)
        return await self.fsm.handle_event(RouteManagerFSM.E_REPORT_STUCK, report)

    async def on_follower_state_update(self, update: FollowerStateUpdate) -> ServiceResult:
        """RouteFollower からの進捗更新を反映し、完了時はFSMへ通知する。"""

        self._log(
            "[Core] on_follower_state_update: state=%s, index=%d, label='%s', ver=%d"
            % (
                str(update.state),
                int(update.active_waypoint_index),
                str(update.active_waypoint_label),
                int(update.route_version),
            )
        )

        route_version = int(update.route_version)
        self.current_index = int(update.active_waypoint_index)
        self.current_label = str(update.active_waypoint_label or "")

        if self.route_model is not None:
            local_version = int(self.route_model.version.to_int())
            if route_version > 0 and local_version != route_version:
                self._log(
                    f"[Core] on_follower_state_update: version mismatch local={local_version} follower={route_version}"
                )
            self.route_model.advance_to(index=self.current_index, label=self.current_label or "")

        emit_version: Optional[int] = route_version if route_version > 0 else None
        self._emit_route_state(index=self.current_index, label=self.current_label, version=emit_version)

        normalized_state = (update.state or "").strip().upper()
        if normalized_state == "FINISHED":
            self._log("[Core] follower reported FINISHED -> notify FSM")
            self._set_status("completed", decision="none", cause="", route_version=emit_version)
            event = FollowerEvent(finished=True)
            return await self.fsm.handle_event(RouteManagerFSM.E_FOLLOWER_UPDATE, event)

        return SimpleServiceResult(True, "Follower update consumed")

    # ------------------------------------------------------------------
    # FSM用コールバック（Get/Update/Replan）
    # ------------------------------------------------------------------
    async def _cb_get_route(self, _unused: Optional[Any]) -> ServiceResult:
        """初期GetRouteの実処理。Node層のget_cbを呼び出し、成功なら受理してpublish。"""
        self._log("[Core] _cb_get_route: call planner_get")
        if self._planner_get is None:
            self._log("[Core] _cb_get_route: planner_get not set")
            return SimpleServiceResult(False, "planner_get not set")

        res = await self._planner_get(self._initial_start, self._initial_goal, self._initial_checkpoints)
        self._log(f"[Core] _cb_get_route: planner_get returned success={getattr(res,'success',False)} msg='{getattr(res,'message','')}'")
        if getattr(res, "success", False) and hasattr(res, "route"):
            route = getattr(res, "route")
            rm = RouteModel.from_route(route)
            if not self._validate_route_model(rm):
                self._log("[Core] _cb_get_route: route validation failed")
                return SimpleServiceResult(False, "invalid route")
            # planner 由来： version は major のみ（minorをリセット）
            rm = RouteModel.from_route(route)
            rm.version = VersionMM(major=int(getattr(rm.version, "major", 0)), minor=0)  # no-op: rm.version already set in Node
            print("version:", int(rm.version.to_int()))
            self._accept_route(
                rm,
                log_prefix="GetRoute OK",
                source="planner",
                event_message="route_ready",
            )
            self._set_status("running", decision="none", cause="")
            return SimpleServiceResult(True, "route_ready")
        return SimpleServiceResult(False, getattr(res, "message", "get_route failed"))

    async def _cb_update_route(self, _unused: Optional[Any]) -> ServiceResult:
        """明示的UpdateRoute（FSMのUPDATEイベント用）。内部ではplanner_updateを呼び出す。"""
        self._log("[Core] _cb_update_route: begin")
        if self._planner_update is None or self.route_model is None:
            self._log("[Core] _cb_update_route: missing planner_update or route_model")
            return SimpleServiceResult(False, "planner_update not set or no route")
        ok = await self._try_update_route(reason="explicit_update")
        self._log(f"[Core] _cb_update_route: result ok={ok}")
        message = "update_route" if ok else "update_route_failed"
        return SimpleServiceResult(ok, message)

    async def _cb_replan(self, data: Optional[Any]) -> ServiceResult:
        """ReportStuck時の再計画（Update→SHIFT→SKIP→failed）。成功でTrue。"""
        self._log("[Core] _cb_replan: begin replan sequence")
        self._last_replan_offset_hint = 0.0
        report: Optional[StuckReport] = data if isinstance(data, StuckReport) else self._last_stuck_report
        reason_enum: Optional[StuckReason] = None
        failure_reason = "avoidance_failed"
        if report is not None:
            reason_label = report.reason_label()
            if reason_label:
                failure_reason = reason_label
            reason_desc = f"{reason_label}(code={report.reason_code})"
            self._log(
                f"[Core] _cb_replan: report idx={report.current_index}, label='{report.current_label}', "
                f"reason='{reason_desc}', avoid_trials={report.avoid_trial_count}, "
                f"last_offset={report.last_applied_offset_m:.2f}, hint_blocked={report.last_hint_blocked}"
            )
            try:
                reason_enum = StuckReason(int(report.reason_code))
            except ValueError:
                reason_enum = None

        normalized_reason = (reason_label or "").strip().lower()
        is_road_blocked = reason_enum == StuckReason.ROAD_BLOCKED or normalized_reason == "road_blocked"
        skip_local_adjustments = False
        skip_planner_update = False
        fixed_segment = self.is_current_segment_fixed()
        if is_road_blocked:
            skip_local_adjustments = True
            if self.route_model is None:
                self._log("[Core] _cb_replan: road_blocked reported but no active route -> fail")
                failure_reason = "road_blocked"
                self._emit_route_state(message=failure_reason)
                self._set_status("holding", decision="failed", cause=failure_reason)
                return SimpleServiceResult(False, failure_reason)
            if fixed_segment:
                self._log("[Core] _cb_replan: road_blocked on fixed segment -> reissue current route")
                return self._reissue_current_route(
                    log_prefix="RoadBlock FixedSegment",
                    event_message="road_blocked_fixed",
                    decision_label="road_blocked_fixed",
                    status_cause="road_blocked",
                )
            self._log("[Core] _cb_replan: road_blocked on variable segment -> planner replan only")
        elif fixed_segment:
            skip_planner_update = True
            self._log("[Core] _cb_replan: fixed segment -> skip planner UpdateRoute trial")

        # 1) UpdateRoute（最初の試行）
        self._log("[Core] _cb_replan: step1 try UpdateRoute (replan_first)")
        if not skip_planner_update:
            if await self._try_update_route(reason="replan_first"):
                self._log("[Core] _cb_replan: step1 success")
                self._last_replan_offset_hint = 0.0
                return SimpleServiceResult(True, "replan_first")
        else:
            self._log("[Core] _cb_replan: step1 skipped (fixed segment)")

        if skip_local_adjustments:
            if is_road_blocked:
                failure_reason = "road_blocked"
            else:
                failure_reason = failure_reason or "road_blocked"
            self._log("[Core] _cb_replan: road_blocked -> skip SHIFT/SKIP and report failure")
            self._emit_route_state(message=failure_reason)
            self._set_status("holding", decision="failed", cause=failure_reason)
            return SimpleServiceResult(False, failure_reason)

        # 2) SHIFT（左右オフセット）
        if self.route_model is not None and not skip_local_adjustments:
            if reason_enum not in (None, StuckReason.FRONT_BLOCKED, StuckReason.AVOIDANCE_FAILED):
                reason_text = report.reason_label() if report is not None else 'unknown'
                reason_code = getattr(report, 'reason_code', 'N/A') if report is not None else 'N/A'
                self._log(
                    f"[Core] _cb_replan: skip SHIFT (reason={reason_text} code={reason_code})"
                )
            else:
                cur = self.route_model.current_index
                prv = self._find_prev_active_index(cur)
                self._log(f"[Core] _cb_replan: step2 try SHIFT cur={cur} prv_active={prv}")
                if prv is not None and cur < self.route_model.total():
                    cur_wp = self.route_model.waypoints[cur]
                    prev_wp = self.route_model.waypoints[prv]
                    if self._try_shift(prev_wp, cur_wp, cur, report):
                        self._log("[Core] _cb_replan: step2 success (SHIFT)")
                        return SimpleServiceResult(True, "shift")

        # 3) SKIP（次WPスキップ）
        if self.route_model is not None and not skip_local_adjustments:
            cur = self.route_model.current_index
            self._log(f"[Core] _cb_replan: step3 try SKIP cur={cur}")
            if cur is not None and self._try_skip(cur):
                self._last_replan_offset_hint = 0.0
                self._log("[Core] _cb_replan: step3 success (SKIP)")
                return SimpleServiceResult(True, "skipped")

        # 4) 失敗
        self._log("[Core] _cb_replan: step4 failed -> holding")
        failure_reason = failure_reason or "avoidance_failed"
        self._emit_route_state(message=failure_reason)
        self._set_status("holding", decision="failed", cause=failure_reason)
        return SimpleServiceResult(False, failure_reason)

    def _reissue_current_route(
        self,
        *,
        log_prefix: str,
        event_message: str,
        decision_label: str,
        status_cause: str,
    ) -> SimpleServiceResult:
        """固定経路向けに現在のルートを複製し、マイナーバージョンのみを更新して再配信する。"""

        if self.route_model is None:
            self._log("[Core] _reissue_current_route: no active route -> abort")
            return SimpleServiceResult(False, status_cause or "fixed_segment")

        current = self.route_model
        new_rm = RouteModel(
            waypoints=copy.deepcopy(current.waypoints),
            version=VersionMM(
                major=current.version.major,
                minor=current.version.minor + 1,
            ),
            frame_id=current.frame_id,
            has_image=current.has_image,
            current_index=current.current_index,
            current_label=current.current_label,
            route_image=copy.deepcopy(current.route_image),
        )
        event_text = event_message or "fixed_segment_retry"
        self._accept_route(
            new_rm,
            log_prefix=log_prefix,
            source="local",
            event_message=event_text,
        )
        self._set_status("running", decision=decision_label, cause=status_cause)
        self._last_replan_offset_hint = 0.0
        return SimpleServiceResult(True, event_text)

    # ------------------------------------------------------------------
    # 具体ロジック（Update/Shift/Skip/Accept/Validate/Status）
    # ------------------------------------------------------------------
    async def _try_update_route(self, *, reason: str) -> bool:
        """プランナの UpdateRoute を呼び、成功時は受理してpublish。"""
        if self._planner_update is None or self.route_model is None or self.route_model.version.to_int() <= 0:
            self._log("[Core] _try_update_route: guard failed (no planner_update or no route or invalid version)")
            return False

        current_idx = int(self.route_model.current_index)
        current_lbl = str(self.route_model.label_at(current_idx) or "")
        prev_idx_opt = self.route_model.prev_index()
        if prev_idx_opt is None:
            self._log(
                "[Core] _try_update_route: prev_index not available (current is first waypoint)"
            )
            return False
        prev_idx = int(prev_idx_opt)
        prev_lbl = str(self.route_model.label_at(prev_idx) or "")
        self._log(
            f"[Core] _try_update_route: call planner_update reason={reason} "
            f"major={self.route_model.version.major} prev=({prev_idx},{prev_lbl}) "
            f"current=({current_idx},{current_lbl})"
        )

        resp = await self._planner_update(
            int(self.route_model.version.major),
            prev_idx,
            prev_lbl,
            current_idx,
            current_lbl,
        )

        ok = bool(getattr(resp, "success", False))
        route = getattr(resp, "route", None)
        self._log(f"[Core] _try_update_route: planner_update returned ok={ok} has_route={route is not None}")
        if not ok or route is None:
            self._log("[Core] _try_update_route: validation failed")
            return False

        # planner 由来： version は major のみ（minorをリセット）
        rm = RouteModel.from_route(route)
        rm.version = VersionMM(major=int(getattr(rm.version, "major", 0)), minor=0)  # no-op: rm.version already set in Node
        if reason == "replan_first":
            event_message = "replan_first"
            decision_label = "replan_first"
        elif reason == "explicit_update":
            event_message = "update_route"
            decision_label = "update"
        else:
            event_message = str(reason)
            decision_label = str(reason)
        self._accept_route(
            rm,
            log_prefix=f"UpdateRoute OK ({reason})",
            source="planner",
            event_message=event_message,
        )
        self._set_status("running", decision=decision_label, cause="")
        self._last_replan_offset_hint = 0.0
        return True

    def _try_shift(
        self,
        prev_wp: WaypointLite,
        cur_wp: WaypointLite,
        cur_idx: int,
        report: Optional[StuckReport] = None,
    ) -> bool:
        """shift：次WPを横方向にずらしたローカル経路を生成し、受理・配信する。"""
        right_open = float(getattr(cur_wp, "right_open", 0.0) or 0.0)
        left_open = float(getattr(cur_wp, "left_open", 0.0) or 0.0)
        label = str(getattr(cur_wp, "label", "") or "")
        pref_key = f"{cur_idx}:{label}" if label else str(cur_idx)
        self._log(
            f"[Core] _try_shift: right_open={right_open}, left_open={left_open}, cur_idx={cur_idx} key='{pref_key}'"
        )
        if right_open <= 0.0 and left_open <= 0.0:
            self._log("[Core] _try_shift: no open space -> abort")
            return False

        ax, ay = float(prev_wp.pose.x), float(prev_wp.pose.y)
        bx, by = float(cur_wp.pose.x), float(cur_wp.pose.y)
        stored_pref = self._shift_preference.get(pref_key)
        right_side = stored_pref if stored_pref is not None else (right_open >= left_open)

        if stored_pref is None:
            oriented: Optional[bool] = None
            if report is not None:
                px, py = float(report.current_pose.x), float(report.current_pose.y)
                seg_x, seg_y = bx - ax, by - ay
                vec_x, vec_y = px - bx, py - by
                if not math.isclose(seg_x, 0.0, abs_tol=1e-9) or not math.isclose(seg_y, 0.0, abs_tol=1e-9):
                    cross = seg_x * vec_y - seg_y * vec_x
                    if not math.isclose(cross, 0.0, abs_tol=1e-6):
                        oriented = cross < 0.0
                        self._log(
                            f"[Core] _try_shift: orientation from report -> cross={cross:.6f}, right_side={oriented}"
                        )
            if oriented is None:
                oriented = right_side
            right_side = oriented
            self._shift_preference[pref_key] = right_side
        else:
            self._log(
                f"[Core] _try_shift: reuse stored orientation right_side={right_side} for key='{pref_key}'"
            )

        open_len = right_open if right_side else left_open
        shift_d = clamp(open_len, 0.0, float(self.offset_step_max_m))
        self._log(f"[Core] _try_shift: right_side={right_side}, open_len={open_len}, shift_d={shift_d}")
        if shift_d <= 0.0:
            self._log("[Core] _try_shift: shift_d <= 0 -> abort")
            return False

        # 位置算出
        nvec = unit_normal((ax, ay), (bx, by), right_side=right_side)
        sx, sy = bx + nvec[0] * shift_d, by + nvec[1] * shift_d
        self._log(f"[Core] _try_shift: base pose=({bx:.3f},{by:.3f}), new pose=({sx:.3f},{sy:.3f}), right_side={right_side}")

        # pose のみ変更、その他フラグは維持
        new_wp = copy.deepcopy(cur_wp)
        new_wp.pose.x = sx
        new_wp.pose.y = sy

        # shift した側の余白を削り、反対側へ同量を加算して帳尻を合わせる
        if right_side:
            new_wp.right_open = max(0.0, right_open - shift_d)
            new_wp.left_open = left_open + shift_d
        else:
            new_wp.left_open = max(0.0, left_open - shift_d)
            new_wp.right_open = right_open + shift_d

        self._log(
            "[Core] _try_shift: updated opens -> "
            f"right_open={new_wp.right_open:.3f}, left_open={new_wp.left_open:.3f}"
        )

        # 新ルート（ローカル更新）
        new_wps = copy.deepcopy(self.route_model.waypoints)
        new_wps[cur_idx] = new_wp
        cur = self.route_model.version
        rm = RouteModel(
            waypoints=new_wps,
            version=VersionMM(major=cur.major, minor=cur.minor + 1),
            frame_id=self.route_model.frame_id,
            has_image=self.route_model.has_image,
            current_index=self.route_model.current_index,
            current_label=self.route_model.current_label,
            route_image=copy.deepcopy(self.route_model.route_image),
        )
        event_message = f"shift_{'right' if right_side else 'left'}({shift_d:.1f}m)"
        self._accept_route(
            rm,
            log_prefix="Local SHIFT",
            source="local",
            event_message=event_message,
        )
        self._set_status("running", decision="shift", cause="")
        self._last_replan_offset_hint = shift_d if right_side else -shift_d
        return True

    def _find_prev_active_index(self, start_idx: int) -> Optional[int]:
        """スキップ済みインデックスを除外して直前の有効WPを探す。"""
        if self.route_model is None:
            return None
        idx = start_idx - 1
        while idx >= 0:
            if idx not in self._skipped_indices:
                return idx
            idx -= 1
        return None

    def _try_skip(self, cur_idx: int) -> bool:
        """skip：次WPをスキップしてローカル再配信する。"""
        if self.route_model is None:
            self._log("[Core] _try_skip: no route_model")
            return False
        if not self._skip_allowed:
            self._log("[Core] _try_skip: skip quota exhausted -> abort")
            return False
        total = self.route_model.total()
        if cur_idx < 0 or cur_idx >= total:
            self._log(f"[Core] _try_skip: cur_idx out of range -> abort (cur_idx={cur_idx}, total={total})")
            return False

        cur_wp = self.route_model.waypoints[cur_idx]
        not_skip = bool(getattr(cur_wp, "not_skip", False))
        label = str(getattr(cur_wp, "label", "") or "")
        history_key = f"{cur_idx}:{label}" if label else str(cur_idx)
        skipped_count = self._skip_history.get(history_key, 0)

        self._log(
            f"[Core] _try_skip: cur_idx={cur_idx}, label='{label}', not_skip={not_skip}, "
            f"history_count={skipped_count}"
        )

        if not_skip:
            self._log("[Core] _try_skip: cur_idx.not_skip -> abort")
            return False
        if skipped_count >= 1:
            self._log("[Core] _try_skip: already skipped once -> abort")
            return False

        nxt_idx = cur_idx + 1
        if nxt_idx >= total:
            self._log("[Core] _try_skip: next is last -> abort")
            return False

        new_wps = copy.deepcopy(self.route_model.waypoints)
        new_cur_label = new_wps[nxt_idx].label
        self._log(
            "[Core] _try_skip: advance current index -> "
            f"skipped_idx={cur_idx}, skipped_label='{label}', new_index={nxt_idx}, new_label='{new_cur_label}'"
        )

        cur = self.route_model.version
        rm = RouteModel(
            waypoints=new_wps,
            version=VersionMM(major=cur.major, minor=cur.minor + 1),
            frame_id=self.route_model.frame_id,
            has_image=self.route_model.has_image,
            current_index=nxt_idx,
            current_label=new_cur_label,
            route_image=copy.deepcopy(self.route_model.route_image),
        )
        if new_cur_label:
            event_message = f"skip->{new_cur_label}"
        else:
            event_message = f"skip_index:{nxt_idx}"
        self._accept_route(
            rm,
            log_prefix="Local SKIP",
            source="local",
            event_message=event_message,
        )
        self._set_status("running", decision="skip", cause="")
        self._skip_history[history_key] = skipped_count + 1
        self._skipped_indices.add(cur_idx)
        self._shift_preference.pop(history_key, None)
        self._last_replan_offset_hint = 0.0
        self._skip_allowed = False
        return True

    def _accept_route(
        self,
        rm: RouteModel,
        *,
        log_prefix: str,
        source: str,
        event_message: str = "",
    ) -> None:
        """Route を受理し、内部モデル・公開情報・バージョンを統一して更新する。"""
        self._log(f"[Core] _accept_route: source={source}, {log_prefix}")
        if source != "local":
            self._skip_history.clear()
            self._skipped_indices.clear()
            self._shift_preference.clear()
            self._skip_allowed = True

        self.route_model = rm
        self.current_index = int(rm.current_index)
        self.current_label = str(rm.current_label or "")
        # 現在インデックス同期
        if self.current_index >= 0:
            self.route_model.advance_to(index=self.current_index, label=self.current_label or "")
        self._update_current_segment_flag()
        # publish（Node層へ委譲）。非ROSだが、Node側のpublish関数は変換を担保する。
        self._publish_active_route(self.route_model)
        self._emit_route_state(message=event_message or None)
        self._log(f"{log_prefix}: version={int(self.route_model.version.to_int())} "
                  f"(major={self.route_model.version.major}, minor={self.route_model.version.minor}), "
                  f"waypoints={len(self.route_model.waypoints)} source={source}")

    def _validate_route_model(self, rm: RouteModel) -> bool:
        """Routeの妥当性検証（元実装に準拠）。"""
        if getattr(rm, "frame_id", "") != "map":
            self._log(f"Invalid frame_id: expected 'map', got '{getattr(rm, 'frame_id', '')}'")
            return False
        version = int(rm.version.to_int())
        if version <= 0:
            self._log(f"Invalid route.version: {version} (must be > 0)")
            return False
        n_wp = len(getattr(rm, "waypoints", []))
        if n_wp <= 0:
            self._log("Route has no waypoints.")
            return False
        if getattr(rm, "route_image", None) is None or not rm.route_image.is_valid():
            self._log("Route has no route_image.")
            return False
        return True

    def _set_status(
        self,
        state: str,
        decision: str = "none",
        cause: str = "",
        route_version: Optional[int] = None,
    ) -> None:
        """Statusを生成してpublish（非ROS）。"""
        ver = int(route_version if route_version is not None else (
            self.route_model.version.to_int() if self.route_model else 0
        ))
        normalized_cause = cause if state == "holding" else ""
        self.current_status = state
        self._log(
            f"[Core] _set_status: state={state}, decision={decision}, cause={normalized_cause}, ver={ver}"
        )
        self._publish_status(state, decision, normalized_cause, ver)
        self._emit_route_state(status=state)

    def _emit_route_state(
        self,
        *,
        status: Optional[str] = None,
        message: Optional[str] = None,
        index: Optional[int] = None,
        label: Optional[str] = None,
        version: Optional[int] = None,
        total: Optional[int] = None,
    ) -> None:
        """最新のRouteState情報をpublishコールバックへ引き渡す。"""
        if status is not None:
            self.current_status = status
        idx = index if index is not None else int(
            self.route_model.current_index if self.route_model else self.current_index
        )
        lbl = label if label is not None else str(
            self.route_model.current_label if self.route_model else self.current_label
        )
        ver = version if version is not None else int(
            self.route_model.version.to_int() if self.route_model else 0
        )
        tot = total if total is not None else int(
            self.route_model.total() if self.route_model else 0
        )
        if message is not None:
            self._last_route_message = message
        self._publish_route_state(idx, lbl, ver, tot, str(self.current_status), self._last_route_message)

    # ------------------------------------------------------------------
    # FSM Transition Hook
    # ------------------------------------------------------------------
    def _on_transition(self, old: str, new: str) -> None:
        """FSM状態遷移をログ化。必要に応じてメトリクス等もここで扱う。"""
        self._log(f"[Core] FSM transition: {old} -> {new}")