#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manager_fsm.py
非ROS依存のFSM実装（Phase2準拠・改訂版）

※ 本ファイルは、元の `route_manager_fsm.py` を **名称変更**（リネーム）したものです。
   仕様・コメントは移植の観点から**省略や要約をせず**に保持しています。
   Node層から注入される各コールバックのI/Fも元実装と同一です。

改訂ポイント:
- ReportStuck/Update/GetRoute の各コールバックに timeout を適用（await が返らない「無反応」を防止）
- 状態ガードを追加（不正な状態でのイベントを拒否）
- イベント直列化（asyncio.Lock）で同時実行を抑止
- 型ヒント/Google Python Style/日本語コメント整備

Node層からは以下のコールバックを注入する:
- set_get_callback(cb)
- set_update_callback(cb)
- set_replan_callback(cb)

各cbは `async def cb(data) -> ServiceResult` を想定する。
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Optional, Protocol


class ServiceResult(Protocol):
    """サービス呼び出し結果のプロトコル。"""

    @property
    def success(self) -> bool: ...

    @property
    def message(self) -> str: ...


# コールバック型（ROS非依存、Node層が実装）
GetRouteCallback = Callable[[Optional[Any]], Awaitable[ServiceResult]]
UpdateRouteCallback = Callable[[Optional[Any]], Awaitable[ServiceResult]]
ReplanCallback = Callable[[Optional[Any]], Awaitable[ServiceResult]]
TransitionHook = Callable[[str, str], None]


@dataclass
class SimpleServiceResult:
    """簡易サービス結果（単体試験用）。"""

    success: bool
    message: str = ""


class RouteManagerFSM:
    """route_manager の非ROS依存FSM。"""

    # 状態
    S_IDLE = "IDLE"
    S_REQUESTING = "REQUESTING"
    S_ACTIVE = "ACTIVE"
    S_WAITING_REROUTE = "WAITING_REROUTE"
    S_UPDATING = "UPDATING"
    S_COMPLETED = "COMPLETED"
    S_ERROR = "ERROR"

    # イベント
    E_REQUEST_INITIAL_ROUTE = "REQUEST_INITIAL_ROUTE"
    E_REPORT_STUCK = "REPORT_STUCK"
    E_UPDATE_ROUTE = "UPDATE_ROUTE"
    E_FOLLOWER_UPDATE = "FOLLOWER_UPDATE"
    E_FORCE_ERROR = "FORCE_ERROR"

    def __init__(
        self,
        logger: Callable[[str], None],
        get_timeout_sec: float | None = None,
        update_timeout_sec: float | None = None,
        replan_timeout_sec: float | None = None,
    ) -> None:
        """コンストラクタ。

        Args:
            logger: ログ出力関数（Node層から注入）。
            get_timeout_sec: GetRoute待ちの個別タイムアウト（Noneは無制限）。
            update_timeout_sec: UpdateRoute待ちの個別タイムアウト。
            replan_timeout_sec: Replan待ちの個別タイムアウト。
        """
        self._state: str = self.S_IDLE
        self._logger = logger

        # Node層から注入されるコールバック
        self._get_cb: Optional[GetRouteCallback] = None
        self._update_cb: Optional[UpdateRouteCallback] = None
        self._replan_cb: Optional[ReplanCallback] = None

        # 遷移フック（監視やメトリクス用）
        self._transition_hooks: list[TransitionHook] = []

        # イベント直列化用Lock（同時イベントを抑止）
        self._lock = asyncio.Lock()

        # タイムアウト設定（秒）
        self._get_timeout = get_timeout_sec
        self._update_timeout = update_timeout_sec
        self._replan_timeout = replan_timeout_sec

    # ----------------------- 注入ポイント -----------------------
    def set_get_callback(self, cb: GetRouteCallback) -> None:
        """GetRoute実行のコールバックを設定する。"""
        self._get_cb = cb

    def set_update_callback(self, cb: UpdateRouteCallback) -> None:
        """UpdateRoute実行のコールバックを設定する。"""
        self._update_cb = cb

    def set_replan_callback(self, cb: ReplanCallback) -> None:
        """再計画（ReportStuck後）のコールバックを設定する。"""
        self._replan_cb = cb

    def add_transition_hook(self, hook: TransitionHook) -> None:
        """状態遷移時に呼ばれるフックを追加する。"""
        self._transition_hooks.append(hook)

    # ----------------------- 状態アクセス -----------------------
    @property
    def state(self) -> str:
        """現在の状態名を返す。"""
        return self._state

    # ----------------------- メイン処理 -------------------------
    async def handle_event(self, event: str, data: Optional[Any] = None) -> SimpleServiceResult:
        """イベントを受け取り、必要なら非同期処理を実行して状態遷移する。"""
        async with self._lock:  # 直列化
            try:
                # ログ追加：イベント受付
                #self._logger(f"[FSM] handle_event start: event={event}, state={self._state}")
                # 状態ガード（不正イベントは拒否して早期return）
                if not self._allow_event(event):
                    self._logger(f"FSM: Event '{event}' is not allowed in state '{self._state}'")
                    return SimpleServiceResult(False, "Event not allowed in current state")

                if event == self.E_REQUEST_INITIAL_ROUTE:
                    result = await self._on_request_initial_route(data)
                    self._logger(f"[FSM] handle_event done: event={event}, result={result.success}, msg='{result.message}'")
                    return result
                if event == self.E_REPORT_STUCK:
                    result = await self._on_report_stuck(data)
                    self._logger(f"[FSM] handle_event done: event={event}, result={result.success}, msg='{result.message}'")
                    return result
                if event == self.E_UPDATE_ROUTE:
                    result = await self._on_update_route(data)
                    self._logger(f"[FSM] handle_event done: event={event}, result={result.success}, msg='{result.message}'")
                    return result
                if event == self.E_FOLLOWER_UPDATE:
                    # FOLLOWER_UPDATEは完走検知のみ（dataに finished: bool を期待）
                    finished = bool(getattr(data, "finished", False) if data is not None else False)
                    if finished and self._state != self.S_COMPLETED:
                        self._logger(f"[FSM] follower update: finished={finished}")
                        await self._transition(self.S_COMPLETED)
                    return SimpleServiceResult(True, "Follower update consumed")
                if event == self.E_FORCE_ERROR:
                    await self._transition(self.S_ERROR)
                    self._logger("[FSM] forced ERROR")
                    return SimpleServiceResult(False, "Forced to ERROR")

                # 未定義イベント
                self._logger(f"FSM: Unhandled event '{event}' on state '{self._state}'")
                return SimpleServiceResult(False, "Unhandled event")
            except asyncio.TimeoutError:
                # cb側の無反応をERRORにフォールバック
                self._logger(f"FSM: Timeout on event '{event}' at state '{self._state}'")
                await self._transition(self.S_ERROR)
                return SimpleServiceResult(False, "Timeout")
            except Exception as exc:
                # 予期せぬ例外はERRORへ
                self._logger(f"FSM: Exception on event '{event}': {exc}")
                await self._transition(self.S_ERROR)
                return SimpleServiceResult(False, f"Exception: {exc}")

    # ----------------------- 個別ハンドラ -----------------------
    async def _on_request_initial_route(self, data: Optional[Any]) -> SimpleServiceResult:
        """初期経路要求。GetRouteを実行し、成功でACTIVEへ、失敗でERRORへ。"""
        self._logger("[FSM] on_request_initial_route: begin")
        await self._transition(self.S_REQUESTING)
        if self._get_cb is None:
            await self._transition(self.S_ERROR)
            self._logger("[FSM] on_request_initial_route: get_cb not set -> ERROR")
            return SimpleServiceResult(False, "GetRoute callback not set")

        res = await self._await_with_timeout(self._get_cb(data), self._get_timeout)
        self._logger(f"[FSM] on_request_initial_route: get_cb result success={getattr(res,'success',False)} msg='{getattr(res,'message','')}'")
        if getattr(res, "success", False):
            await self._transition(self.S_ACTIVE)
            return SimpleServiceResult(True, getattr(res, "message", ""))
        await self._transition(self.S_ERROR)
        return SimpleServiceResult(False, getattr(res, "message", ""))

    async def _on_report_stuck(self, data: Optional[Any]) -> SimpleServiceResult:
        """スタック通知。Replanを実行し、成功でACTIVE、失敗でERROR。"""
        self._logger("[FSM] on_report_stuck: begin")
        await self._transition(self.S_WAITING_REROUTE)
        if self._replan_cb is None:
            await self._transition(self.S_ERROR)
            self._logger("[FSM] on_report_stuck: replan_cb not set -> ERROR")
            return SimpleServiceResult(False, "Replan callback not set")

        res = await self._await_with_timeout(self._replan_cb(data), self._replan_timeout)
        self._logger(f"[FSM] on_report_stuck: replan result success={getattr(res,'success',False)} msg='{getattr(res,'message','')}'")
        if getattr(res, "success", False):
            await self._transition(self.S_ACTIVE)
            return SimpleServiceResult(True, getattr(res, "message", ""))
        await self._transition(self.S_ERROR)
        return SimpleServiceResult(False, getattr(res, "message", ""))

    async def _on_update_route(self, data: Optional[Any]) -> SimpleServiceResult:
        """明示的な部分経路更新要求。UpdateRouteを実行し、成功でACTIVE、失敗でERROR。"""
        self._logger("[FSM] on_update_route: begin")
        await self._transition(self.S_UPDATING)
        if self._update_cb is None:
            await self._transition(self.S_ERROR)
            self._logger("[FSM] on_update_route: update_cb not set -> ERROR")
            return SimpleServiceResult(False, "UpdateRoute callback not set")

        res = await self._await_with_timeout(self._update_cb(data), self._update_timeout)
        self._logger(f"[FSM] on_update_route: update result success={getattr(res,'success',False)} msg='{getattr(res,'message','')}'")
        if getattr(res, "success", False):
            await self._transition(self.S_ACTIVE)
            return SimpleServiceResult(True, getattr(res, "message", ""))
        await self._transition(self.S_ERROR)
        return SimpleServiceResult(False, getattr(res, "message", ""))

    # ----------------------- ユーティリティ ---------------------
    def _allow_event(self, event: str) -> bool:
        """現在状態で許容されるイベントか判定する（最小限のガード）。"""
        s = self._state
        if event == self.E_REQUEST_INITIAL_ROUTE:
            return s in (self.S_IDLE, self.S_ERROR)  # 再試行を許す
        if event == self.E_REPORT_STUCK:
            return s in (self.S_ACTIVE, self.S_WAITING_REROUTE)
        if event == self.E_UPDATE_ROUTE:
            return s in (self.S_ACTIVE, self.S_UPDATING)
        if event in (self.E_FOLLOWER_UPDATE, self.E_FORCE_ERROR):
            return True
        return False

    async def _await_with_timeout(self, awaitable: Awaitable[Any], timeout: float | None) -> Any:
        """awaitable にタイムアウトを適用して待機する。"""
        if timeout is None or timeout <= 0:
            self._logger("[FSM] await_with_timeout: no-timeout wait")
            return await awaitable
        self._logger(f"[FSM] await_with_timeout: timeout={timeout}s")
        return await asyncio.wait_for(awaitable, timeout=timeout)

    async def _transition(self, new_state: str) -> None:
        """状態遷移し、フックを呼び出す。"""
        if new_state == self._state:
            return
        old = self._state
        self._state = new_state
        self._logger(f"FSM: {old} -> {new_state}")
        for hook in list(self._transition_hooks):
            try:
                hook(old, new_state)
            except Exception:
                # フック側の例外はFSMの健全性に影響させない
                pass
