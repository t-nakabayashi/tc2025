#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manager_core.py
route_manager の**非ROS依存**コア実装。

目的:
- Node層（ROS2依存）は通信・入出力に限定し、すべての業務ロジックを本モジュールへ集約する。
- 既存の `route_manager_node.py` に含まれていたロジック（5段階 replan/shift/skip/fallback/failed、
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
from typing import Any, Awaitable, Callable, List, Optional, Protocol, Tuple

from manager_fsm import RouteManagerFSM, SimpleServiceResult, ServiceResult

# =============================================================================
# 非ROS依存データモデル
# =============================================================================

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
    """2D位置（最小限）。"""
    x: float = 0.0
    y: float = 0.0


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
                has_image=bool(getattr(rm, "has_image", True)),
                current_index=int(getattr(rm, "current_index", 0)),
                current_label=str(getattr(rm, "current_label", "")),
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
        return cls(waypoints=wps, version=ver, frame_id=str(getattr(getattr(route_like, "header", None), "frame_id", getattr(route_like, "frame_id", "map"))), has_image=bool(getattr(route_like, "has_image", True)), current_index=cur_idx, current_label=cur_lbl)

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
    async def __call__(self, major_version: int, prev_index: int, prev_label: str,
                       next_index: Optional[int], next_label: str) -> ServiceResult: ...


PublishActiveRoute = Callable[[RouteModel], None]
PublishStatus = Callable[[str, str, str, int], None]
PublishRouteState = Callable[[int, str, int, int, str], None]  # idx, label, ver, total, status


# =============================================================================
# Core本体
# =============================================================================

class RouteManagerCore:
    """RouteManager の非ROS依存コア。

    - FSMの生成・駆動
    - 5段階 replan/shift/skip/fallback/failed の完全実装
    - ルート受理・バージョン管理・状態更新
    - Node層からの各種コールバック注入
    """
    def __init__(
        self,
        logger: Callable[[str], None],
        publish_active_route: PublishActiveRoute,
        publish_status: PublishStatus,
        publish_route_state: PublishRouteState,
        offset_step_m_max: float = 1.5,
    ) -> None:
        # 依存注入
        self._log = logger
        self._publish_active_route = publish_active_route
        self._publish_status = publish_status
        self._publish_route_state = publish_route_state

        # パラメータ
        self.offset_step_m_max = float(offset_step_m_max)

        # 内部状態
        self.route_model: Optional[RouteModel] = None
        self.current_index: int = -1
        self.current_label: str = ""
        self.current_status: str = "IDLE"

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

    def update_follower_state(self, current_index: int, current_label: str) -> None:
        """FollowerStateの進捗をCoreへ反映する（非ROS依存）。"""
        #self._log(f"[Core] update_follower_state: idx={current_index}, label='{current_label}'")
        self.current_index = int(current_index)
        self.current_label = str(current_label or "")

        # 進捗をroute_modelへ反映
        finished = False
        if self.route_model is not None:
            self.route_model.advance_to(index=self.current_index, label=self.current_label)
            # route_modelの完走判定メソッドで結果を取得
            if hasattr(self.route_model, "is_completed"):
                finished = bool(self.route_model.is_completed())

        # route_stateをpublish
        self._publish_route_state(
            int(self.route_model.current_index if self.route_model else -1),
            str(self.route_model.current_label if self.route_model else ""),
            int(self.route_model.version.to_int() if self.route_model else 0),
            int(self.route_model.total() if self.route_model else 0),
            str(self.current_status),
        )

        # --- FSMへFOLLOWER_UPDATEイベントを通知（完走検知を含む） ---
        # FSMは data.finished を用いて完走判定を行う設計であるため、Core側で完走判定結果を付与して送信する。
        if hasattr(self, "fsm") and self.fsm is not None:
            class FollowerUpdateData:
                """FSMへ渡す進捗情報（完走検知用の簡易データ構造）"""
                def __init__(self, finished: bool):
                    self.finished = finished

            data = FollowerUpdateData(finished=finished)

            async def _notify_fsm() -> None:
                """FSMにFOLLOWER_UPDATEイベントを非同期通知する。"""
                try:
                    await self.fsm.handle_event(self.fsm.E_FOLLOWER_UPDATE, data=data)
                except Exception as e:
                    self._log(f"[Core] failed to notify FSM (FOLLOWER_UPDATE): {e}")

            # Core専用イベントループ上で安全に非同期実行
            import asyncio
            asyncio.run_coroutine_threadsafe(_notify_fsm(), self.loop)

    async def request_initial_route(self, start_label: str, goal_label: str, checkpoint_labels: List[str]) -> ServiceResult:
        """初期ルートをFSM経由で要求する。"""
        self._log(f"[Core] request_initial_route: start='{start_label}', goal='{goal_label}', checkpoints={checkpoint_labels}")
        # FSMへ渡すデータはコールバック側で参照するため、Coreに保持
        self._initial_start = start_label
        self._initial_goal = goal_label
        self._initial_checkpoints = list(checkpoint_labels)
        return await self.fsm.handle_event(RouteManagerFSM.E_REQUEST_INITIAL_ROUTE, None)

    async def on_report_stuck(self) -> ServiceResult:
        """ReportStuck受付時の処理をFSMに委譲（内部で5段階ロジックを実行）。"""
        self._log("[Core] on_report_stuck: received")
        return await self.fsm.handle_event(RouteManagerFSM.E_REPORT_STUCK, None)

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
            self._accept_route(rm, log_prefix="GetRoute OK", source="planner")
            self._set_status("running", decision="none", cause="route_ready")
            return SimpleServiceResult(True, "get_route ok")
        return SimpleServiceResult(False, getattr(res, "message", "get_route failed"))

    async def _cb_update_route(self, _unused: Optional[Any]) -> ServiceResult:
        """明示的UpdateRoute（FSMのUPDATEイベント用）。内部ではplanner_updateを呼び出す。"""
        self._log("[Core] _cb_update_route: begin")
        if self._planner_update is None or self.route_model is None:
            self._log("[Core] _cb_update_route: missing planner_update or route_model")
            return SimpleServiceResult(False, "planner_update not set or no route")
        ok = await self._try_update_route(reason="explicit_update")
        self._log(f"[Core] _cb_update_route: result ok={ok}")
        return SimpleServiceResult(ok, "update_route " + ("ok" if ok else "failed"))

    async def _cb_replan(self, _unused: Optional[Any]) -> ServiceResult:
        """ReportStuck時の再計画（5段階ロジック）。成功でTrue。"""
        self._log("[Core] _cb_replan: begin 5-step sequence")
        # 1) UpdateRoute（最初の試行）
        self._log("[Core] _cb_replan: step1 try UpdateRoute (replan_first)")
        if await self._try_update_route(reason="replan_first"):
            self._log("[Core] _cb_replan: step1 success")
            return SimpleServiceResult(True, "replan_first")

        # 2) SHIFT（左右オフセット）
        if self.route_model is not None:
            cur = self.route_model.current_index
            prv = self.route_model.prev_index()
            self._log(f"[Core] _cb_replan: step2 try SHIFT cur={cur} prv={prv}")
            if prv is not None and prv < self.route_model.total():
                cur_wp = self.route_model.waypoints[cur]
                prev_wp = self.route_model.waypoints[prv]
                if self._try_shift(prev_wp, cur_wp, cur):
                    self._log("[Core] _cb_replan: step2 success (SHIFT)")
                    return SimpleServiceResult(True, "shift")

        # 3) SKIP（次WPスキップ）
        if self.route_model is not None:
            cur = self.route_model.current_index
            self._log(f"[Core] _cb_replan: step3 try SKIP cur={cur}")
            if cur is not None and self._try_skip(cur):
                self._log("[Core] _cb_replan: step3 success (SKIP)")
                return SimpleServiceResult(True, "skipped")

        # 4) UpdateRoute（フォールバック）
        self._log("[Core] _cb_replan: step4 try UpdateRoute (fallback_replan)")
        if await self._try_update_route(reason="fallback_replan"):
            self._log("[Core] _cb_replan: step4 success")
            return SimpleServiceResult(True, "fallback_replan")

        # 5) 失敗
        self._log("[Core] _cb_replan: step5 failed -> holding")
        self._set_status("holding", decision="failed", cause="avoidance_failed")
        return SimpleServiceResult(False, "avoidance_failed")

    # ------------------------------------------------------------------
    # 具体ロジック（Update/Shift/Skip/Accept/Validate/Status）
    # ------------------------------------------------------------------
    async def _try_update_route(self, *, reason: str) -> bool:
        """プランナの UpdateRoute を呼び、成功時は受理してpublish。"""
        if self._planner_update is None or self.route_model is None or self.route_model.version.to_int() <= 0:
            self._log("[Core] _try_update_route: guard failed (no planner_update or no route or invalid version)")
            return False

        prev_idx = int(self.route_model.current_index)
        prev_lbl = str(self.route_model.label_at(prev_idx) or "")
        nxt_idx = self.route_model.next_index()
        nxt_lbl = str(self.route_model.label_at(nxt_idx) or "") if nxt_idx is not None else ""
        self._log(f"[Core] _try_update_route: call planner_update reason={reason} "
                  f"major={self.route_model.version.major} prev=({prev_idx},{prev_lbl}) "
                  f"next=({nxt_idx},{nxt_lbl})")

        resp = await self._planner_update(
            int(self.route_model.version.major),
            prev_idx,
            prev_lbl,
            int(nxt_idx) if nxt_idx is not None else None,
            nxt_lbl,
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
        self._accept_route(rm, log_prefix=f"UpdateRoute OK ({reason})", source="planner")
        return True

    def _try_shift(self, prev_wp: WaypointLite, cur_wp: WaypointLite, cur_idx: int) -> bool:
        """shift：次WPを横方向にずらしたローカル経路を生成し、受理・配信する。"""
        right_open = float(getattr(cur_wp, "right_open", 0.0) or 0.0)
        left_open = float(getattr(cur_wp, "left_open", 0.0) or 0.0)
        self._log(f"[Core] _try_shift: right_open={right_open}, left_open={left_open}, cur_idx={cur_idx}")
        if right_open <= 0.0 and left_open <= 0.0:
            self._log("[Core] _try_shift: no open space -> abort")
            return False

        # 空きが大きい側に寄せる
        right_side = right_open >= left_open
        open_len = right_open if right_side else left_open
        shift_d = clamp(open_len, 0.0, float(self.offset_step_m_max))
        self._log(f"[Core] _try_shift: right_side={right_side}, open_len={open_len}, shift_d={shift_d}")
        if shift_d <= 0.0:
            self._log("[Core] _try_shift: shift_d <= 0 -> abort")
            return False

        # 位置算出
        ax, ay = float(prev_wp.pose.x), float(prev_wp.pose.y)
        bx, by = float(cur_wp.pose.x), float(cur_wp.pose.y)
        nvec = unit_normal((ax, ay), (bx, by), right_side=right_side)
        sx, sy = bx + nvec[0] * shift_d, by + nvec[1] * shift_d
        self._log(f"[Core] _try_shift: base pose=({bx:.3f},{by:.3f}), new pose=({sx:.3f},{sy:.3f}), right_side={right_side}")

        # pose のみ変更、その他フラグは維持
        new_wp = copy.deepcopy(cur_wp)
        new_wp.pose.x = sx
        new_wp.pose.y = sy

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
        )
        self._accept_route(rm, log_prefix="Local SHIFT", source="local")
        return True

    def _try_skip(self, cur_idx: int) -> bool:
        """skip：次WPをスキップしてローカル再配信する。"""
        if self.route_model is None:
            self._log("[Core] _try_skip: no route_model")
            return False
        cur_idx = self.route_model.waypoints[cur_idx]
        not_skip = bool(getattr(cur_idx, "not_skip", False))
        self._log(f"[Core] _try_skip: cur_idx={cur_idx}, not_skip={not_skip}")
        if not_skip:
            self._log("[Core] _try_skip: cur_idx.not_skip -> abort")
            return False
        if cur_idx + 1 >= self.route_model.total():
            self._log("[Core] _try_skip: next is last -> abort")
            return False

        # 次WPをスキップ
        nxt_idx = self.route_model.next_index()
        if nxt_idx is None or nxt_idx >= self.route_model.total():
            self._log("[Core] _try_skip: nxt_idx is wrong -> abort")
            return False

        # 新ルート（ローカル更新）
        new_wps = copy.deepcopy(self.route_model.waypoints)
        cur = self.route_model.version
        rm = RouteModel(
            waypoints=new_wps,
            version=VersionMM(major=cur.major, minor=cur.minor + 1),
            frame_id=self.route_model.frame_id,
            has_image=self.route_model.has_image,
            current_index=nxt_idx,
            current_label=new_wps[nxt_idx].label,
        )
        self._accept_route(rm, log_prefix="Local SKIP", source="local")
        return True

    def _accept_route(self, rm: RouteModel, *, log_prefix: str, source: str) -> None:
        """Route を受理し、内部モデル・公開情報・バージョンを統一して更新する。"""
        self._log(f"[Core] _accept_route: source={source}, {log_prefix}")
        self.route_model = rm
        # 現在インデックス同期
        if self.current_index >= 0:
            self.route_model.advance_to(index=self.current_index, label=self.current_label or "")
        # publish（Node層へ委譲）。非ROSだが、Node側のpublish関数は変換を担保する。
        self._publish_active_route(self.route_model)
        self._publish_route_state(
            int(self.route_model.current_index if self.route_model else -1),
            str(self.route_model.current_label if self.route_model else ""),
            int(self.route_model.version.to_int() if self.route_model else 0),
            int(self.route_model.total() if self.route_model else 0),
            str(self.current_status),
        )
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
        if not getattr(rm, "has_image", False):
            self._log("Route has no route_image.")
            return False
        return True

    def _set_status(self, state: str, decision: str = "none", cause: str = "", route_version: Optional[int] = None) -> None:
        """Statusを生成してpublish（非ROS）。"""
        ver = int(route_version if route_version is not None else (
            self.route_model.version.to_int() if self.route_model else 0
        ))
        self.current_status = state
        self._log(f"[Core] _set_status: state={state}, decision={decision}, cause={cause}, ver={ver}")
        self._publish_status(state, decision, cause, ver)
        # RouteStateも更新
        self._publish_route_state(
            int(self.route_model.current_index if self.route_model else -1),
            str(self.route_model.current_label if self.route_model else ""),
            ver,
            int(self.route_model.total() if self.route_model else 0),
            str(self.current_status),
        )

    # ------------------------------------------------------------------
    # FSM Transition Hook
    # ------------------------------------------------------------------
    def _on_transition(self, old: str, new: str) -> None:
        """FSM状態遷移をログ化。必要に応じてメトリクス等もここで扱う。"""
        self._log(f"[Core] FSM transition: {old} -> {new}")