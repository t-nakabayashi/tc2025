#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FollowerCore（route_follower ロジック本体 / 最終版）

設計方針（本ファイル）:
  - ROS2 依存なし（純Python）
  - 周期処理 tick() は「最新の確定情報のみ」を参照して状態遷移・出力を決定
  - 非タイマー（サブスク相当）の update_* は mailbox に書き込むだけ
      - Route / Pose は最新1件の mailbox + 受け渡し用ロック（_inbox_lock）
      - Hint は Node 側で「多数決+中央値」まで統計済みとし、Core には最新値1件のみ渡す
        （Core 側は _hint_lock で最新統計値を保持・tick はスナップショット取得して使用）
  - 状態変更や履歴更新は tick() のみが行う（一貫性確保）

Phase2 必須要件（本ロジックで満たす）:
  - 滞留検知（過去位置差＋平均速度, 連続時間しきい値あり）
  - Hintに基づく L字回避（横シフト→前進の2段階サブ目標 / deque）
  - 回避失敗時は WAITING_REROUTE へ（/report_stuck の呼出しは Node 側責務）
  - ルート差し替え時はポーズ履歴クリア＋滞留グレース期間（即誤検知防止）

I/O（Core 観点のデータ構造）
  - 入力: update_route(Route), update_pose(Pose), update_hint_stat(fb_major, med_l, med_r, enough)
  - 出力: FollowerOutput(target_pose: Pose|None, state: dict|None)

作者メモ:
  - ここでの Waypoint.pose.yaw は制御上の向きとして使う（回避サブゴール生成など）
"""

from __future__ import annotations

import math
import time
import threading
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import Deque, List, Optional, Tuple
from statistics import median


# ============================================================
# データ構造
# ============================================================

@dataclass
class Pose:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class Waypoint:
    label: str
    pose: Pose
    line_stop: bool = False
    signal_stop: bool = False
    left_open: float = 0.0
    right_open: float = 0.0


@dataclass
class Route:
    version: int
    waypoints: List[Waypoint]
    start_index: int = 0
    start_waypoint_label: str = ""


@dataclass
class HintSample:
    t: float
    front_blocked: bool
    left_offset: float
    right_offset: float
    front_clearance: float = math.inf


@dataclass
class FollowerOutput:
    """tick() の出力。target/state は必要時のみ埋める。"""
    target_pose: Optional[Pose] = None
    state: Optional[dict] = None


class FollowerStatus(Enum):
    IDLE = auto()
    RUNNING = auto()
    WAITING_STOP = auto()
    STAGNATION_DETECTED = auto()
    AVOIDING = auto()
    WAITING_REROUTE = auto()
    FINISHED = auto()
    ERROR = auto()


# ============================================================
# FollowerCore
# ============================================================

class FollowerCore:
    """route_follower のロジック中核（ROS非依存 / スレッド安全 / tickは最新情報のみ参照）."""

    # ========================= 初期化 =========================
    def __init__(self, logger=None) -> None:
        # ロガ（Node側のlogger.info互換を渡すことを想定。未指定ならprint）
        self.log = logger.info if logger else print

        # ---- パラメータ ----
        # 追従
        self.arrival_threshold = 0.6
        self.control_rate_hz = 20.0
        self.republish_target_hz = 1e-9

        # 滞留検知
        self.window_sec = 2.0
        self.progress_epsilon_m = 0.10
        self.min_speed_mps = 0.05
        self.stagnation_duration_sec = 15.0
        # ルート適用直後の滞留グレース期間（誤検知防止）
        self.stagnation_grace_sec = 2.0

        # 回避行動（L字）
        self.avoid_min_offset_m = 0.35
        self.avoid_max_offset_m = 5.0
        self.avoid_forward_clearance_m = 2.0
        self.max_avoidance_attempts_per_wp = 2
        self.avoid_back_offset_m = 0.5

        # 再経路待機
        self.reroute_timeout_sec = 30.0

        # road_blocked 起因のリルート抑制用ホールド時間
        self.road_blocked_confirmation_sec = 5.0

        # ---- 内部状態（tickのみが変更）----
        self.status = FollowerStatus.IDLE
        self.route: Optional[Route] = None
        self.index = 0
        self.current_pose: Optional[Pose] = None
        self.route_version = -1
        self.route_active: bool = False

        # 過去位置・滞留検知
        self.pose_hist: Deque[Tuple[float, Pose]] = deque()
        self.stagnation_hold_start: Optional[float] = None
        self.stagnation_grace_until: float = 0.0

        # 回避管理
        self.avoid_active: bool = False
        self.avoid_subgoals: Deque[Tuple[str, Pose]] = deque()
        self.avoid_attempt_count = 0
        self.last_applied_offset_m: float = 0.0

        # 出力キャッシュ（/active_target保険再送等に使いたい場合）
        self.last_target: Optional[Pose] = None

        # reroute待機
        self.reroute_wait_start: Optional[float] = None
        self.reroute_wait_deadline: Optional[float] = None

        # 起動時自動開始フラグ
        self.start_immediately: bool = True

        # 直近の滞留理由
        self.last_stagnation_reason = ""
        
        # ---- Control（manual_start / sig_recog / road_blocked）----
        self._ctrl_lock = threading.Lock()
        self._manual_start_mb: Optional[bool] = None
        self._sig_recog_mb: Optional[int] = None  # 1=GO, 2=NOGO, その他=未定義
        self._road_blocked_mb: Optional[bool] = None
        self._road_blocked_state: Optional[bool] = None
        self._road_blocked_last_update: Optional[float] = None

        # ---- 受け渡し箱（mailboxes）とロック ----
        # 非タイマー(update_*) → [inbox_lock] → mailbox へ書き込み
        # tick() 冒頭で snapshot（取り出し＆クリア）して以降は無ロックで処理
        self._inbox_lock = threading.Lock()
        self._route_mailbox: Optional[Route] = None
        self._pose_mailbox: Optional[Pose] = None

        # Hint は Node 側で統計済み最新値を update_hint_stat で受領
        self._hint_lock = threading.Lock()

        # Hint サンプルの時間窓キャッシュ（Coreで統計）
        self._hint_cache: Deque[HintSample] = deque()
        self.hint_cache_window_sec = 5.0
        self.hint_majority_true_ratio = 0.8
        self.hint_min_samples = 5

        # 統計結果（update_hintで逐次更新）
        self._hint_front_blocked_majority = False
        self._hint_left_offset_median = 0.0
        self._hint_right_offset_median = 0.0
        self._hint_enough = False
        self._hint_front_clearance_latest = float('inf')

    # ========================= 受け渡しAPI（非タイマー） =========================
    def update_route(self, route: Route) -> None:
        """最新ルートを mailbox に上書き。状態は変更しない（tickで適用）。"""
        if not route.waypoints:
            self.log("[FollowerCore] 空のrouteを無視します。")
            return
        with self._inbox_lock:
            self._route_mailbox = route

    def update_pose(self, pose: Pose) -> None:
        """最新ポーズを mailbox に上書き。状態は変更しない（tickで適用）。"""
        with self._inbox_lock:
            self._pose_mailbox = pose

    def update_hint_stat(self, fb_major: bool, med_l: float, med_r: float, enough: bool) -> None:
        """Hint統計の最新値をセット（Node側で統計済み）。"""
        with self._hint_lock:
            self._hint_front_blocked_majority = bool(fb_major)
            self._hint_left_offset_median = float(med_l)
            self._hint_right_offset_median = float(med_r)
            self._hint_enough = bool(enough)

    # tick 内で使用するスナップショット取得（ロックは極小）
    def _get_hint_snapshot(self) -> Tuple[bool, float, float, bool]:
        with self._hint_lock:
            return (
                self._hint_front_blocked_majority,
                self._hint_left_offset_median,
                self._hint_right_offset_median,
                self._hint_enough,
            )

    def update_hint(self, hint: 'HintSample') -> None:
        """Hintサンプル受信時に逐次統計を更新（Core側で時間窓・多数決・中央値）。"""
        now = float(hint.t)
        with self._hint_lock:
            # 追加
            self._hint_cache.append(hint)
            self._hint_front_clearance_latest = float(hint.front_clearance)
            # 窓外を削除
            wnd = float(self.hint_cache_window_sec)
            while self._hint_cache and (now - float(self._hint_cache[0].t)) > wnd:
                self._hint_cache.popleft()
            n = len(self._hint_cache)
            if n < int(self.hint_min_samples):
                self._hint_front_blocked_majority = False
                self._hint_left_offset_median = 0.0
                self._hint_right_offset_median = 0.0
                self._hint_enough = False
                return
            fb_vals = [1 if s.front_blocked else 0 for s in self._hint_cache]
            self._hint_front_blocked_majority = (sum(fb_vals) / n) >= float(self.hint_majority_true_ratio)
            fb_samples = [s for s in self._hint_cache if s.front_blocked]
            if fb_samples:
                self._hint_left_offset_median = float(median(s.left_offset for s in fb_samples))
                self._hint_right_offset_median = float(median(s.right_offset for s in fb_samples))
            else:
                self._hint_left_offset_median = 0.0
                self._hint_right_offset_median = 0.0
            self._hint_enough = True

    def update_control_inputs(
        self,
        manual_start: Optional[bool] = None,
        sig_recog: Optional[int] = None,
        road_blocked: Optional[bool] = None,
    ) -> None:
        """/manual_start(Bool)、/sig_recog(Int32)、/road_blocked(Bool) の最新値をホールドする。"""
        with self._ctrl_lock:
            if manual_start is not None:
                self._manual_start_mb = bool(manual_start)
            if sig_recog is not None:
                self._sig_recog_mb = int(sig_recog)
            if road_blocked is not None:
                blocked = bool(road_blocked)
                self._road_blocked_mb = blocked
                self._road_blocked_state = blocked
                self._road_blocked_last_update = time.time()

    def _get_control_snapshot(self) -> Tuple[Optional[bool], Optional[int], Optional[bool]]:
        with self._ctrl_lock:
            return self._manual_start_mb, self._sig_recog_mb, self._road_blocked_mb

    def _consume_control_inputs(self) -> None:
        """manual_start・sig_recog のラッチ値を消費済みとしてクリアする。"""
        with self._ctrl_lock:
            self._manual_start_mb = None
            self._sig_recog_mb = None

    def _clear_road_blocked_mailbox(self) -> None:
        """road_blocked の保持値を外部イベントなしにクリアする。"""

        with self._ctrl_lock:
            self._road_blocked_mb = None
            self._road_blocked_state = None
            self._road_blocked_last_update = None

    def _is_road_blocked_sustained(self) -> bool:
        """road_blocked=True が所定時間継続しているかを確認する。"""
        with self._ctrl_lock:
            active = self._road_blocked_state is True
            last_update = self._road_blocked_last_update

        if not active or last_update is None:
            return False
        return (time.time() - last_update) >= self.road_blocked_confirmation_sec

    # ========================= 周期処理（唯一の状態更新点） =========================
    def tick(self) -> FollowerOutput:
        """1周期の更新処理。
        手順:
          1) mailbox を snapshot & クリア（ごく短いロック区間）
          2) Route → Pose → Hint の順に適用
          3) 本処理（到達判定・回避・滞留など）
        """
        # --- 1) 受け渡し箱 snapshot ---
        with self._inbox_lock:
            route_msg = self._route_mailbox
            pose_msg = self._pose_mailbox
            self._route_mailbox = None
            self._pose_mailbox = None

        # --- 2-1) Route 適用（先に適用して以降の計算前提を更新） ---
        if route_msg is not None:
            self._apply_route(route_msg)

        # --- 2-2) Pose 適用（最新自己位置を更新し履歴に積む） ---
        if pose_msg is not None:
            self._apply_pose(pose_msg)

        # --- 2-3) Hint スナップショット ---
        fb_major, med_l, med_r, enough = self._get_hint_snapshot()

        # --- 3) Control スナップショット ---
        manual_start, sig_recog, road_blocked = self._get_control_snapshot()

        # --- 4) 本処理 ---
        return self._tick_main(
            fb_major,
            med_l,
            med_r,
            enough,
            manual_start,
            sig_recog,
            road_blocked,
        )

    # ========================= 内部適用処理 =========================
    def _apply_route(self, route: Route) -> None:
        """新ルート適用。関連状態を初期化し、滞留グレース期間を設定。"""
        self.route = route
        self.route_active = True
        self.route_version = route.version
        # リルート判定に使用した road_blocked のラッチ値を、新ルート適用のタイミングで破棄する。
        self._clear_road_blocked_mailbox()
        if route.waypoints:
            self.index = max(0, min(route.start_index, len(route.waypoints) - 1))
        else:
            self.index = 0
        self.pose_hist.clear()
        self.avoid_active = False
        self.avoid_subgoals.clear()
        self.avoid_attempt_count = 0
        self.last_applied_offset_m = 0.0
        self.last_stagnation_reason = ""
        self.reroute_wait_start = None
        self.reroute_wait_deadline = None
        self.stagnation_grace_until = time.time() + self.stagnation_grace_sec
        if route.waypoints:
            self.last_target = route.waypoints[self.index].pose
        else:
            self.last_target = None
        self.log(f"[FollowerCore] Route適用 version={route.version} waypoints={len(route.waypoints)} start_index={route.start_index}")

    def _apply_pose(self, pose: Pose) -> None:
        """最新ポーズを反映し、履歴をメンテナンス。"""
        self.current_pose = pose
        now = time.time()
        self.pose_hist.append((now, pose))
        # 古い履歴掃除（窓の2〜3倍 or 10s の大きい方）
        limit = max(3.0 * self.window_sec, 10.0)
        while self.pose_hist and (now - self.pose_hist[0][0]) > limit:
            self.pose_hist.popleft()

    # ========================= 本処理（状態遷移・判定） =========================
    def _tick_main(
        self,
        fb_major: bool,
        med_l: float,
        med_r: float,
        enough: bool,
        manual_start: Optional[bool],
        sig_recog: Optional[int],
        road_blocked: Optional[bool],
    ) -> FollowerOutput:
        # ERROR/FINISHED は状態監視のみ
        if self.status in (FollowerStatus.ERROR, FollowerStatus.FINISHED):
            return FollowerOutput(None, self._make_state_dict())

        # IDLEでルートが設定されたらRUNNINGに遷移
        if self.status == FollowerStatus.IDLE:
            if self.route_active:
                if self.start_immediately or bool(manual_start):
                    if not self.start_immediately:
                        self._consume_control_inputs()
                    self.log(f"[FollowerCore] IDLE -> RUNNING index={self.index}")
                    next_wp = self.route.waypoints[self.index]
                    self.status = FollowerStatus.RUNNING
                    return FollowerOutput(next_wp.pose, self._make_state_dict())
            return FollowerOutput(None, self._make_state_dict())

        # RUNNING/AVOIDING/WAITING_STOP/WAITING_REROUTE
        if self.route is None or not self.route.waypoints:
            self.status = FollowerStatus.ERROR
            return FollowerOutput(None, self._make_state_dict())

        # 現在地必須
        if self.current_pose is None:
            return FollowerOutput(self.last_target, self._make_state_dict())

        cur_wp = self.route.waypoints[self.index]
        cur_pose = self.current_pose

        # --- 1) 回避中の処理 ---
        if self.avoid_active and self.avoid_subgoals:
            label, pose = self.avoid_subgoals[0]
            if self._euclid_xy(cur_pose, pose) < self.arrival_threshold:
                self.log(f"[FollowerCore] Avoidance sub-goal reached: {label}")
                self.avoid_subgoals.popleft()
                if self.avoid_subgoals:
                    next_label, next_pose = self.avoid_subgoals[0]
                    self.log(f"[FollowerCore] Proceed to next avoidance step: {next_label}")
                    self.last_target = next_pose
                    return FollowerOutput(next_pose, self._make_state_dict())
                else:
                    # すべて消化 → 本来WPへ復帰
                    self.avoid_active = False
                    self.status = FollowerStatus.RUNNING
                    if self.route is not None and self.route.waypoints:
                        rejoin_index = self._decide_rejoin_index_after_avoidance(
                            self.current_pose, self.route, self.index
                        )
                        self.index = rejoin_index
                        rejoin_wp = self.route.waypoints[self.index]
                        self.last_target = rejoin_wp.pose
                        self.log(
                            "[FollowerCore] Avoidance completed. Back to main route. "
                            f"rejoin_index={self.index}"
                        )
                        return FollowerOutput(rejoin_wp.pose, self._make_state_dict())

                    self.last_target = cur_wp.pose
                    self.log("[FollowerCore] Avoidance completed. Back to main route.")
                    return FollowerOutput(cur_wp.pose, self._make_state_dict())

            # 回避継続中：滞留チェック（グレース期間は対象外）
            if time.time() >= self.stagnation_grace_until and self._check_stagnation_tick(exclude_stop=False):
                self.log("[FollowerCore] Re-stagnation during avoidance -> WAITING_REROUTE")
                self.avoid_active = False
                self.last_stagnation_reason = "avoidance_failed"
                self._enter_waiting_reroute()
            return FollowerOutput(self.last_target, self._make_state_dict())

        # --- 2) WAITING_STOP：manual_start / sig_recog による解除 ---
        if self.status == FollowerStatus.WAITING_STOP:
            can_resume = False
            if cur_wp.signal_stop:
                can_resume = bool(manual_start) or (sig_recog == 1)
            else:
                can_resume = bool(manual_start)
            if not can_resume:
                return FollowerOutput(self.last_target, self._make_state_dict())

            # 解除：次WP or FINISHED
            self._consume_control_inputs()
            if self.index < len(self.route.waypoints) - 1:
                self.index += 1
                next_wp = self.route.waypoints[self.index]
                self.avoid_active = False
                self.avoid_subgoals.clear()
                self.avoid_attempt_count = 0
                self.last_applied_offset_m = 0.0
                self.last_target = next_wp.pose
                self.status = FollowerStatus.RUNNING
                self.stagnation_grace_until = time.time() + 1.0  # 短い猶予で誤検知防止
                self.log(f"[FollowerCore] STOP解除 -> RUNNING index={self.index}")
                return FollowerOutput(next_wp.pose, self._make_state_dict())
            else:
                self.status = FollowerStatus.FINISHED
                self.log("[FollowerCore] STOP解除 -> FINISHED (last waypoint)")
                return FollowerOutput(None, self._make_state_dict())

        # --- 3) WAITING_REROUTE の解除・タイムアウト管理 ---
        if self.status == FollowerStatus.WAITING_REROUTE:
            # 解除判定
            if self.route_active:
                self.log(f"[FollowerCore] WAITING_REROUTE -> RUNNING index={self.index}")
                next_wp = self.route.waypoints[self.index]
                self.last_target = next_wp.pose
                self.status = FollowerStatus.RUNNING
                return FollowerOutput(next_wp.pose, self._make_state_dict())
            # タイムアウト管理
            now = time.time()
            if self.reroute_wait_deadline and now >= self.reroute_wait_deadline:
                self.log("[FollowerCore] WAITING_REROUTE deadline reached -> ERROR")
                self.status = FollowerStatus.ERROR
            elif self.reroute_wait_start and (now - self.reroute_wait_start) > self.reroute_timeout_sec:
                self.log("[FollowerCore] WAITING_REROUTE timeout -> ERROR")
                self.status = FollowerStatus.ERROR
            return FollowerOutput(self.last_target, self._make_state_dict())

        # --- 4) RUNNING の到達判定 ---
        if self.status == FollowerStatus.RUNNING:
            dist = self._euclid_xy(cur_pose, cur_wp.pose)
            if dist < self.arrival_threshold:
                # STOP系
                if cur_wp.line_stop or cur_wp.signal_stop:
                    self.status = FollowerStatus.WAITING_STOP
                    self._consume_control_inputs()
                    self.log(f"[FollowerCore] Reached STOP waypoint -> WAITING_STOP (line_stop={cur_wp.line_stop}, signal_stop={cur_wp.signal_stop})")
                    return FollowerOutput(self.last_target, self._make_state_dict())
                # 次WPへ
                if self.index < len(self.route.waypoints) - 1:
                    self.index += 1
                    next_wp = self.route.waypoints[self.index]
                    self.avoid_active = False
                    self.avoid_subgoals.clear()
                    self.avoid_attempt_count = 0
                    self.last_applied_offset_m = 0.0
                    self.last_target = next_wp.pose
                    self.log(f"[FollowerCore] Proceed to next waypoint index={self.index}, pose=({next_wp.pose.x:.2f}, {next_wp.pose.y:.2f})")
                    return FollowerOutput(next_wp.pose, self._make_state_dict())
                # 終端
                self.status = FollowerStatus.FINISHED
                self.log("[FollowerCore] Final waypoint reached -> FINISHED")
                return FollowerOutput(None, self._make_state_dict())

            # --- 未到達：滞留判定（グレース期間内はスキップ） ---
            exclude_stop = False
            if cur_wp.line_stop or cur_wp.signal_stop:
                # 停止系WPでも停止線に到達する前は滞留判定を有効にする
                exclude_stop = dist < self.arrival_threshold

            if time.time() >= self.stagnation_grace_until:
                if self._check_stagnation_tick(exclude_stop=exclude_stop):
                    self.status = FollowerStatus.STAGNATION_DETECTED
                    # 滞留検知をトリガとして road_blocked を最優先で確認する。
                    if self._is_road_blocked_sustained():
                        self.last_stagnation_reason = "road_blocked"
                        self._consume_control_inputs()
                        self._enter_waiting_reroute()
                        return FollowerOutput(self.last_target, self._make_state_dict())

                    # Hint統計（Node側集約）に基づく判断。
                    # waypointの開放度を先に確認し、横方向余裕なしの場合は即座にRUNNINGへ復帰する。
                    right_open = max(
                        float(getattr(cur_wp, "right_open", 0.0)),
                        float(getattr(cur_wp, "right_isopen", 0.0)),
                    )
                    left_open = max(
                        float(getattr(cur_wp, "left_open", 0.0)),
                        float(getattr(cur_wp, "left_isopen", 0.0)),
                    )

                    if right_open <= 0.0 and left_open <= 0.0:
                        # waypointに横方向余裕がない場合は回避やreport_stuckを行わず継続する。
                        self.status = FollowerStatus.RUNNING
                        self.last_stagnation_reason = ""
                        self.log(
                            "[FollowerCore] No lateral opening at waypoint -> continue RUNNING"
                        )
                        return FollowerOutput(self.last_target, self._make_state_dict())

                    if not enough:
                        self.last_stagnation_reason = "no_hint"
                        self._enter_waiting_reroute()
                        return FollowerOutput(self.last_target, self._make_state_dict())

                    if not fb_major:
                        # 前方遮蔽が継続していないため監視継続のみ行う。
                        self.status = FollowerStatus.RUNNING
                        self.last_stagnation_reason = ""
                        self.log("[FollowerCore] Stagnation cleared by hint -> continue RUNNING")
                        return FollowerOutput(self.last_target, self._make_state_dict())

                    self.last_stagnation_reason = "front_blocked"
                    success = self._start_avoidance_sequence(cur_wp, cur_pose, med_l, med_r)
                    if success:
                        self.status = FollowerStatus.AVOIDING
                        return FollowerOutput(self.last_target, self._make_state_dict())

                    self.last_stagnation_reason = "no_space"
                    self._enter_waiting_reroute()
                    return FollowerOutput(self.last_target, self._make_state_dict())

            # 到達していないが滞留でもない → 目標の再提示（保険）
            return FollowerOutput(self.last_target, self._make_state_dict())

        # それ以外の状態はここには来ないはずだが保険
        return FollowerOutput(self.last_target, self._make_state_dict())

    # ========================= 補助：滞留・回避・状態遷移 =========================
    def _enter_waiting_reroute(self) -> None:
        """WAITING_REROUTE へ遷移し待機窓をセット（Node側で /report_stuck を呼ぶ想定）"""
        self.status = FollowerStatus.WAITING_REROUTE
        self.route_active = False
        now = time.time()
        self.reroute_wait_start = now
        self.reroute_wait_deadline = now + self.reroute_timeout_sec

    def notify_reroute_failed(self, note: str = "") -> None:
        """/report_stuck が失敗した際にノード層から呼び出す。"""
        self.route_active = False
        self.avoid_active = False
        self.avoid_subgoals.clear()
        self.reroute_wait_start = None
        self.reroute_wait_deadline = None
        if note:
            self.last_stagnation_reason = note
        # リルート失敗時も road_blocked のラッチ値はここで明示的にリセットする。
        self._clear_road_blocked_mailbox()
        self.status = FollowerStatus.ERROR
        self.log(f"[FollowerCore] WAITING_REROUTE failed -> ERROR note='{note}'")

    def _check_stagnation_tick(self, exclude_stop: bool) -> bool:
        """滞留判定。
        条件:
          - 直近 window_sec の移動距離 < progress_epsilon_m
          - 直近 window_sec の平均速度 < min_speed_mps
          - 上記が stagnation_duration_sec 継続
          - exclude_stop=True の場合は判定しない
        """
        if exclude_stop or not self.pose_hist:
            self.stagnation_hold_start = None
            return False

        now = time.time()
        window = self.window_sec
        eps = self.progress_epsilon_m
        min_v = self.min_speed_mps
        need = self.stagnation_duration_sec

        # 窓内サンプル抽出（新しい→古い）
        recent: List[Tuple[float, Pose]] = [tp for tp in reversed(self.pose_hist) if (now - tp[0]) <= window]
        if len(recent) < 2:
            self.stagnation_hold_start = None
            return False

        _, p_new = recent[0]
        _, p_old = recent[-1]
        # 厳密なdt計算ではなく窓長を採用（保守的に判断）
        dt = max(window, 1e-6)
        dist = self._euclid_xy(p_new, p_old)
        speed = dist / dt
        cond = (dist < eps) and (speed < min_v)

        if cond:
            if self.stagnation_hold_start is None:
                self.stagnation_hold_start = now
            elif (now - self.stagnation_hold_start) >= need:
                self.stagnation_hold_start = None
                self.log(f"[FollowerCore] Stagnation detected dist={dist:.2f}m speed={speed:.2f}m/s window={window}s")
                return True
        else:
            self.stagnation_hold_start = None
        return False

    def _start_avoidance_sequence(self, wp: Waypoint, cur_pose: Pose,
                                  med_l: float, med_r: float) -> bool:
        """Hint統計とWP上限を考慮して L字回避サブゴール（2点）を生成・開始。"""
        # Waypoint 上限（0 or 負は0扱い）
        left_limit = max(float(getattr(wp, "left_open", 0.0)), 0.0)
        right_limit = max(float(getattr(wp, "right_open", 0.0)), 0.0)

        # 実効上限（グローバル上限とWP上限のmin）
        left_max = min(left_limit, self.avoid_max_offset_m)
        right_max = min(right_limit, self.avoid_max_offset_m)

        def pick_offset(hint_open: float, side_max: float) -> float:
            if hint_open <= 0.0 or side_max <= 0.0:
                return 0.0
            allowed = min(hint_open, side_max)
            return max(self.avoid_min_offset_m, min(allowed, side_max))

        off_L = pick_offset(med_l, left_max)
        off_R = pick_offset(med_r, right_max)

        options: List[Tuple[str, float]] = []
        if off_L > 0.0:
            options.append(("L", off_L))
        if off_R > 0.0:
            options.append(("R", off_R))
        if not options:
            return False

        # 最小オフセット優先（同値なら左優先）
        options.sort(key=lambda x: (x[1], 0 if x[0] == "L" else 1))
        side, offset = options[0]

        yaw = cur_pose.yaw
        back_offset = max(self.avoid_back_offset_m, 0.0)
        forward = back_offset + self.avoid_forward_clearance_m

        # 起点を現在姿勢から進行方向後方へシフトして余裕を確保する。
        pivot_x = cur_pose.x - math.cos(yaw) * back_offset
        pivot_y = cur_pose.y - math.sin(yaw) * back_offset

        # (1) 横シフト点 p1
        offset_y = +offset if side == "L" else -offset
        dx1 = -math.sin(yaw) * offset_y
        dy1 = math.cos(yaw) * offset_y
        p1_x = pivot_x + dx1
        p1_y = pivot_y + dy1
        p1 = Pose(p1_x, p1_y, self._yaw_between(pivot_x, pivot_y, p1_x, p1_y))

        # (2) 前進点 p2
        dx2 = math.cos(yaw) * forward
        dy2 = math.sin(yaw) * forward
        p2 = Pose(p1.x + dx2, p1.y + dy2,
                  self._yaw_between(p1.x, p1.y, p1.x + dx2, p1.y + dy2))

        # 登録して開始
        self.avoid_subgoals.clear()
        self.avoid_subgoals.append((f"avoid_shift_{side}", p1))
        self.avoid_subgoals.append((f"avoid_forward_{side}", p2))
        self.avoid_active = True
        self.avoid_attempt_count = min(self.avoid_attempt_count + 1, self.max_avoidance_attempts_per_wp)
        self.last_applied_offset_m = -offset if side == "L" else offset
        self.last_target = p1
        self.log(f"[FollowerCore] Start avoidance: side={side} offset={offset:.2f}m")
        return True

    # ========================= 汎用ヘルパ =========================
    def _euclid_xy(self, a: Pose, b: Pose) -> float:
        return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))

    def _yaw_between(self, x0: float, y0: float, x1: float, y1: float) -> float:
        return math.atan2(y1 - y0, x1 - x0)

    def _make_state_dict(self) -> dict:
        """/follower_state 相当の簡易状態（Node側でROS msgへ変換）。"""
        cur_label = ""
        segment_length = 0.0
        if self.route and 0 <= self.index < len(self.route.waypoints):
            cur_wp = self.route.waypoints[self.index]
            cur_label = cur_wp.label
            if self.index > 0:
                prev_wp = self.route.waypoints[self.index - 1]
                segment_length = self._euclid_xy(prev_wp.pose, cur_wp.pose)

        return {
            "status": self.status.name,
            "active_waypoint_index": int(self.index),
            "route_version": int(self.route_version),
            "active_waypoint_label": cur_label,
            "segment_length_m": float(segment_length),
            "avoid_count": int(self.avoid_attempt_count),
            "reason": str(self.last_stagnation_reason),
            "front_blocked": bool(self._hint_front_blocked_majority),
            "left_offset_m": float(self._hint_left_offset_median),
            "right_offset_m": float(self._hint_right_offset_median),
            "front_clearance_m": float(self._hint_front_clearance_latest),
        }

    def get_current_waypoint_label(self) -> str:
        """現在追従中のウェイポイントラベルを返す。"""
        if self.route and 0 <= self.index < len(self.route.waypoints):
            return self.route.waypoints[self.index].label
        return ""

    def get_current_pose(self) -> Optional[Pose]:
        """現在の自己位置（Pose）を返す。"""
        return self.current_pose

    def get_hint_front_blocked(self) -> bool:
        """直近Hintで前方が塞がれているかを返す。"""
        with self._hint_lock:
            return bool(self._hint_front_blocked_majority)

    # ========================= L字回避復帰支援 =========================
    def _project_point_to_segment(
        self, p: Pose, a: Pose, b: Pose
    ) -> Tuple[float, float, float]:
        """点pを線分abへ射影し、範囲内にクランプした点と距離を返す。"""
        ax = float(a.x)
        ay = float(a.y)
        bx = float(b.x)
        by = float(b.y)
        px = float(p.x)
        py = float(p.y)

        vx = bx - ax
        vy = by - ay
        seg_len_sq = vx * vx + vy * vy
        if seg_len_sq <= 1e-9:
            dist = math.hypot(px - ax, py - ay)
            return ax, ay, dist

        t = ((px - ax) * vx + (py - ay) * vy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        proj_x = ax + vx * t
        proj_y = ay + vy * t
        dist = math.hypot(px - proj_x, py - proj_y)
        return proj_x, proj_y, dist

    def _decide_rejoin_index_after_avoidance(
        self, p2: Pose, route: Route, index: int
    ) -> int:
        """L字回避完了後に自然復帰するウェイポイントindexを決定する。"""
        waypoints = route.waypoints
        if not waypoints:
            return index

        n = len(waypoints)
        base_index = max(0, min(index, n - 1))
        if base_index >= n - 1:
            return n - 1

        e_x = 0.0
        e_y = 0.0
        for seg_index in range(base_index, n - 1):
            seg_dx = waypoints[seg_index + 1].pose.x - waypoints[seg_index].pose.x
            seg_dy = waypoints[seg_index + 1].pose.y - waypoints[seg_index].pose.y
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len > 1e-6:
                e_x = seg_dx / seg_len
                e_y = seg_dy / seg_len
                break

        if abs(e_x) < 1e-9 and abs(e_y) < 1e-9:
            # ルート方向が求まらない場合は従来通りp2のyawを使用する。
            e_x = math.cos(p2.yaw)
            e_y = math.sin(p2.yaw)

        best_segment: Optional[int] = None
        best_distance = float("inf")

        for i in range(base_index, n - 1):
            proj_x, proj_y, dist = self._project_point_to_segment(
                p2, waypoints[i].pose, waypoints[i + 1].pose
            )
            vec_x = proj_x - p2.x
            vec_y = proj_y - p2.y
            s_i = vec_x * e_x + vec_y * e_y
            if s_i < -0.1:
                continue
            if dist < best_distance:
                best_distance = dist
                best_segment = i

        if best_segment is None:
            j_base = base_index
        else:
            j_base = min(best_segment + 1, n - 1)

        rejoin_index = j_base
        for s in range(base_index, j_base + 1):
            wp = waypoints[s]
            if wp.line_stop or wp.signal_stop:
                rejoin_index = s
                break

        return rejoin_index

