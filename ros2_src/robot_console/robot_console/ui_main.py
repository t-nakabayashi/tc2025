"""tkinter を利用した robot_console GUI 実装。"""

from __future__ import annotations

import base64
import logging
import math
import subprocess
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any, Dict, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML が無い環境に対応
    yaml = None  # type: ignore[assignment]

try:
    from PIL import Image, ImageTk  # type: ignore
    PIL_IMAGETK_AVAILABLE = True
except ImportError:  # pragma: no cover - 実行環境依存
    try:
        from PIL import Image  # type: ignore
    except ImportError:  # pragma: no cover - Pillow すら無い環境では画像機能を無効化
        Image = None  # type: ignore
    ImageTk = None  # type: ignore
    PIL_IMAGETK_AVAILABLE = False

try:
    import cv2  # type: ignore

    CV2_AVAILABLE = True
except ImportError:  # pragma: no cover - OpenCV が無い環境向けフォールバック
    cv2 = None  # type: ignore
    CV2_AVAILABLE = False

if CV2_AVAILABLE:  # pragma: no branch - OpenCV がある場合のみ NumPy を使用
    import numpy as np
else:  # pragma: no cover - OpenCV 無し環境では未使用
    np = None  # type: ignore

from .gui_core import CAMERA_DISPLAY_SIZE, GuiCore
from .utils import GuiSnapshot, NodeLaunchState, NodeLaunchStatus, resize_with_letter_box

LOGGER = logging.getLogger(__name__)

FOLLOWER_CARD_KEYS = (
    'state',
    'index',
    'label',
    'target_label',
    'stagnation',
    'front_clearance',
    'offsets',
)

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
REFRESH_INTERVAL_MS = 200
SIDEBAR_WIDTH = 200
JST = timezone(timedelta(hours=9))
EVENT_BANNER_TTL = timedelta(seconds=60)
STICKY_BANNER_TEXTS = {'信号: STOP', '停止線: STOP'}

def _format_time(value: Optional[datetime]) -> str:
    """日時をHH:MM:SS形式の文字列に変換する。"""

    if value is None:
        return "--:--:--"
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(JST).strftime("%H:%M:%S")


def _format_duration_seconds(seconds: float) -> str:
    """経過秒を HH:MM:SS 形式へ変換する。"""

    if seconds <= 0.0:
        return "00:00:00"
    total = int(seconds)
    hours = total // 3600
    minutes = (total % 3600) // 60
    secs = total % 60
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def compute_route_progress(
    route: RouteStateView, follower: FollowerStateView
) -> Tuple[float, int, int]:
    """ルート進捗バーと表示値を算出する。

    辞書やGUI側のバインディングに依存せず、純粋にデータモデルから進捗率と
    表示インデックスを決定するためのヘルパーである。揺り戻しによる属性変更が
    発生しても、この関数をテストすることで回帰を検出できるようにする。

    Args:
        route (RouteStateView): ルート全体の状態情報。
        follower (FollowerStateView): フォロワの現在位置情報。

    Returns:
        Tuple[float, int, int]:
            - 進捗率（0.0〜1.0）。
            - 人が読む用の現在インデックス（1始まり、存在しない場合は0）。
            - サニタイズ済み経路数（負値を0に丸めたもの）。
    """

    total_waypoints = max(route.total_waypoints, 0)
    if total_waypoints <= 0:
        return 0.0, 0, 0

    follower_index = max(follower.active_waypoint_index, -1)
    route_index = max(route.current_index, -1)

    progress_source = follower_index
    if progress_source < 0 and route_index >= 0:
        progress_source = route_index

    display_index = min(max(progress_source + 1, 0), total_waypoints)
    progress_ratio = display_index / total_waypoints if total_waypoints else 0.0

    return progress_ratio, display_index, total_waypoints


class ImagePanel(ttk.LabelFrame):
    """画像表示とキャプション、必要に応じてオーバレイを備えたパネル。"""

    def __init__(
        self,
        master: tk.Widget,
        title: str,
        *,
        size: Tuple[int, int],
        enable_overlay: bool = False,
    ) -> None:
        super().__init__(master, text=title, padding=(6, 6))
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=0)

        self._target_size = size
        self._canvas = tk.Canvas(
            self,
            background="#1f1f1f",
            highlightthickness=0,
            width=size[0],
            height=size[1],
        )
        self._canvas.grid(row=0, column=0, sticky="nsew")
        self._canvas.bind("<Configure>", lambda _event: self._relayout())

        self._caption_var = tk.StringVar(value="")
        caption = ttk.Label(self, textvariable=self._caption_var, anchor="e")
        caption.grid(row=1, column=0, sticky="ew", pady=(4, 0))

        self._photo: Optional[tk.PhotoImage] = None
        self._image_item: Optional[int] = None
        self._text_item: Optional[int] = None
        self._overlay_label: Optional[tk.Label] = None
        if enable_overlay:
            self._overlay_label = tk.Label(
                self._canvas,
                background="#000000",
                foreground="#ffffff",
                font=("Helvetica", 10, "bold"),
                padx=8,
                pady=4,
                justify="left",
                anchor="nw",
            )

    @property
    def target_size(self) -> Tuple[int, int]:
        """キャンバスに想定する画像サイズを返す。"""

        return self._target_size

    def get_display_size(self) -> Tuple[int, int]:
        """現在のキャンバスサイズを取得し、未配置時はターゲットサイズを返す。"""

        width = self._canvas.winfo_width()
        height = self._canvas.winfo_height()
        if width <= 1:
            width = self._target_size[0]
        if height <= 1:
            height = self._target_size[1]
        return width, height

    def update_image(self, photo: Optional[tk.PhotoImage], *, alt_text: str = "画像未取得") -> None:
        """画像と代替テキストを設定する。"""

        if self._image_item is not None:
            self._canvas.delete(self._image_item)
            self._image_item = None
        if self._text_item is not None:
            self._canvas.delete(self._text_item)
            self._text_item = None

        self._photo = photo
        display_width, _ = self.get_display_size()
        if photo is None:
            self._text_item = self._canvas.create_text(
                0,
                0,
                text=alt_text,
                fill="#f0f0f0",
                font=("Helvetica", 12, "bold"),
                justify="center",
                anchor="center",
                width=max(display_width - 20, 50),
            )
        else:
            self._image_item = self._canvas.create_image(0, 0, image=photo, anchor="center")
        self._relayout()

    def update_caption(self, text: str) -> None:
        """キャプションテキストを更新する。"""

        self._caption_var.set(text)

    def update_overlay(self, text: str) -> None:
        """オーバレイテキストを更新する。"""

        if self._overlay_label is None:
            return
        if text:
            self._overlay_label.configure(text=text, background="#000000", foreground="#ffffff")
            self._overlay_label.place(x=12, y=12, anchor="nw")
        else:
            self._overlay_label.place_forget()

    def _relayout(self) -> None:
        """キャンバス中心に画像もしくはテキストを配置し直す。"""

        width, height = self.get_display_size()
        center_x = width / 2
        center_y = height / 2
        if self._image_item is not None:
            self._canvas.coords(self._image_item, center_x, center_y)
        if self._text_item is not None:
            self._canvas.coords(self._text_item, center_x, center_y)
            self._canvas.itemconfigure(self._text_item, width=max(width - 24, 50))


@dataclass
class RouteCardVars:
    """ルート進捗カードで利用する tk 変数群。"""

    manager: tk.StringVar
    route_status: tk.StringVar
    progress: tk.DoubleVar
    progress_text: tk.StringVar
    version: tk.StringVar
    detail: tk.StringVar


class UiMain:
    """GuiCore のスナップショットを可視化する tkinter アプリ。"""

    def __init__(self, gui_core: GuiCore) -> None:
        self._core = gui_core
        self._root = tk.Tk()
        self._root.title("robot_console")
        self._root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self._root.minsize(WINDOW_WIDTH, WINDOW_HEIGHT)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close_request)

        self._configure_style()

        self._image_render_available = PIL_IMAGETK_AVAILABLE or CV2_AVAILABLE
        self._image_warning_message = (
            'Pillow ImageTk と OpenCV の両方が利用できないため画像パネルを無効化しました。\n'
            'python3-pil.imagetk もしくは python3-opencv をインストールしてください。'
        ) if not self._image_render_available else ''

        self._sidebar_visible = True

        self._manual_value = tk.BooleanVar(value=False)
        self._manual_status_var = tk.StringVar(value='現在値: False / 最終送信: --:--:--')
        self._sig_value = tk.IntVar(value=1)
        self._sig_status_var = tk.StringVar(value='最終送信: --:--:--')
        self._road_value = tk.BooleanVar(value=False)
        self._road_status_var = tk.StringVar(value='現在値: False / 最終送信: --:--:--')
        self._obstacle_clearance = tk.DoubleVar(value=3.0)
        self._obstacle_left = tk.DoubleVar(value=0.0)
        self._obstacle_right = tk.DoubleVar(value=0.0)
        self._obstacle_blocked = tk.BooleanVar(value=False)
        self._obstacle_override_state = tk.StringVar(
            value='状態: front_blocked=OFF 前方距離:0.00m 左:+0.00m 右:+0.00m 更新: --:--:--'
        )
        self._event_banner = tk.StringVar(value='')
        self._frame_image_path_var = tk.StringVar(value='')
        self._frame_image_status_var = tk.StringVar(value='最終送信: --:--:--')
        self._obstacle_override_active = False
        self._obstacle_toggle_btn: Optional[ttk.Button] = None
        self._image_warning_label: Optional[ttk.Label] = None
        self._image_warning_parent: Optional[ttk.Frame] = None

        self._initialize_runtime_state()

        self._route_state_vars: RouteCardVars = self._create_route_state_vars()
        self._follower_vars = self._create_follower_vars()
        self._velocity_vars = {
            'linear': tk.StringVar(value='0.00 m/s'),
            'angular': tk.StringVar(value='0.0 deg/s'),
            'distance': tk.StringVar(value='0.0 m'),
            'elapsed': tk.StringVar(value='00:00:00'),
        }
        self._target_vars = {
            'distance': tk.StringVar(value='0.0 m'),
            'progress': tk.DoubleVar(value=0.0),
            'progress_percent': tk.StringVar(value='0.0%'),
        }

        self._launch_widgets: Dict[str, Dict[str, object]] = {}
        self._param_texts: Dict[str, tk.Text] = {}
        self._param_last_display: Dict[str, str] = {}
        self._file_cache: Dict[str, Tuple[float, str]] = {}
        self._log_texts: Dict[str, tk.Text] = {}
        self._log_open_buttons: Dict[str, ttk.Button] = {}
        self._log_file_paths: Dict[str, Optional[str]] = {}

        self._build_layout()
        self._on_obstacle_params_changed()
        self._schedule_update()

    def _initialize_runtime_state(self) -> None:
        """更新処理および停止監視で利用する内部状態を初期化する。"""

        self._line_stop_active_since: Optional[datetime] = None
        self._last_line_stop_state: bool = False
        self._last_target_progress_ratio: Optional[float] = None
        self._sticky_stop_banner: Optional[Tuple[str, Optional[datetime]]] = None
        self._latest_snapshot: Optional[GuiSnapshot] = None
        self._closing: bool = False
        self._shutdown_pending: bool = False
        self._update_job: Optional[str] = None
        self._shutdown_check_job: Optional[str] = None
        self._stagnation_retry_baseline: Optional[int] = None
        self._stagnation_display_reason: str = '滞留なし'
        self._stagnation_ignored_reason: Optional[str] = None
        self._obstacle_override_active = False
        self._sig_value_initialized: bool = False
        self._sig_last_command: Optional[int] = None
        self._segment_distances: Dict[Tuple[int, int, str], float] = {}
        self._total_distance_m: float = 0.0
        self._route_manager_previous_status: Optional[NodeLaunchStatus] = None
        self._follower_run_started_at: Optional[datetime] = None
        self._follower_last_state: Optional[str] = None
        self._follower_elapsed_seconds: float = 0.0

    def _create_route_state_vars(self) -> RouteCardVars:
        """ルート進捗カードで利用する tk 変数を生成する。"""

        return RouteCardVars(
            manager=tk.StringVar(value='UNKNOWN'),
            route_status=tk.StringVar(value='UNKNOWN'),
            progress=tk.DoubleVar(value=0.0),
            progress_text=tk.StringVar(value='0 / 0'),
            version=tk.StringVar(value='バージョン: 0'),
            detail=tk.StringVar(value='最新イベントなし'),
        )

    def _create_follower_vars(self) -> Dict[str, tk.StringVar]:
        """フォロワ状態カードで利用する tk 変数を生成する。"""

        follower_label = tk.StringVar(value='現在: -')
        follower_vars: Dict[str, tk.StringVar] = {
            'state': tk.StringVar(value='UNKNOWN'),
            'index': tk.StringVar(value='Index: 0'),
            'label': follower_label,
            'target_label': follower_label,
            'stagnation': tk.StringVar(value='滞留なし'),
            'front_clearance': tk.StringVar(value='障害物なし'),
            'offsets': tk.StringVar(value='左:+0.00m / 右:+0.00m'),
        }
        missing = set(FOLLOWER_CARD_KEYS) - set(follower_vars.keys())
        if missing:
            raise AssertionError(f'フォロワカードのキーが不足しています: {missing}')
        return follower_vars

    def _build_layout(self) -> None:
        self._notebook = ttk.Notebook(self._root)
        self._dashboard_tab = ttk.Frame(self._notebook)
        self._params_tab = ttk.Frame(self._notebook)
        self._logs_tab = ttk.Frame(self._notebook)
        self._notebook.add(self._dashboard_tab, text='ダッシュボード')
        self._notebook.add(self._params_tab, text='パラメータ一覧')
        self._notebook.add(self._logs_tab, text='コンソールログ')
        self._notebook.pack(fill=tk.BOTH, expand=True)

        self._build_dashboard(self._dashboard_tab)
        self._build_params(self._params_tab)
        self._build_logs(self._logs_tab)

    def _configure_style(self) -> None:
        """ダッシュボード全体のスタイルを設定する。"""

        style = ttk.Style(self._root)
        try:
            style.theme_use('clam')
        except tk.TclError:
            # 利用可能なテーマが無い場合は既定のまま利用する。
            pass
        style.configure('Card.TLabelframe', background='#f7f9fc')
        style.configure('Card.TLabelframe.Label', font=('Helvetica', 11, 'bold'))
        style.configure('Bold.TLabel', font=('Helvetica', 11, 'bold'))

    def _build_dashboard(self, parent: tk.Widget) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)

        wrapper = ttk.Frame(parent, padding=10)
        wrapper.grid(row=0, column=0, sticky='nsew')
        wrapper.columnconfigure(0, weight=1)
        wrapper.columnconfigure(1, weight=0)
        wrapper.rowconfigure(0, weight=1)

        main_area = ttk.Frame(wrapper)
        main_area.grid(row=0, column=0, sticky='nsew')
        main_area.columnconfigure(0, weight=1)
        main_area.rowconfigure(1, weight=1)

        top_bar = ttk.Frame(main_area)
        top_bar.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        top_bar.columnconfigure(0, weight=1)
        ttk.Label(top_bar, text='robot_console ダッシュボード', style='Bold.TLabel').grid(
            row=0,
            column=0,
            sticky='w',
        )
        self._sidebar_toggle_btn = ttk.Button(
            top_bar,
            text='◀ ノード起動パネル',
            command=self._toggle_sidebar,
            width=18,
        )
        self._sidebar_toggle_btn.grid(row=0, column=1, sticky='e')

        dashboard_body = ttk.Frame(main_area)
        dashboard_body.grid(row=1, column=0, sticky='nsew')
        dashboard_body.columnconfigure(0, weight=1)
        dashboard_body.rowconfigure(1, weight=1)

        self._build_state_summary(dashboard_body)
        self._build_image_row(dashboard_body)
        self._build_control_panel(dashboard_body)

        self._sidebar_container = ttk.Frame(wrapper)
        self._sidebar_container.grid(row=0, column=1, sticky='ns', padx=(12, 0))
        self._sidebar_container.columnconfigure(0, weight=1)
        self._sidebar_container.rowconfigure(0, weight=1)

        self._sidebar_canvas = tk.Canvas(self._sidebar_container, highlightthickness=0)
        self._sidebar_canvas.grid(row=0, column=0, sticky='nsew')
        self._sidebar_canvas.configure(width=SIDEBAR_WIDTH)
        scrollbar = ttk.Scrollbar(
            self._sidebar_container,
            orient=tk.VERTICAL,
            command=self._sidebar_canvas.yview,
        )
        scrollbar.grid(row=0, column=1, sticky='ns')
        self._sidebar_canvas.configure(yscrollcommand=scrollbar.set)

        self._sidebar_inner = ttk.Frame(self._sidebar_canvas, padding=(6, 10, 6, 10))
        self._sidebar_window = self._sidebar_canvas.create_window(
            (0, 0),
            window=self._sidebar_inner,
            anchor='nw',
        )

        def _on_configure(event: tk.Event) -> None:  # type: ignore[override]
            self._sidebar_canvas.configure(scrollregion=self._sidebar_canvas.bbox('all'))

        self._sidebar_inner.bind('<Configure>', _on_configure)
        self._sidebar_canvas.bind(
            '<Configure>',
            lambda event: self._sidebar_canvas.itemconfigure(
                self._sidebar_window,
                width=event.width,
            ),
        )

        self._build_launch_sidebar(self._sidebar_inner)
        self._update_sidebar_visibility()

    def _toggle_sidebar(self) -> None:
        """ノード起動パネルの表示・非表示を切り替える。"""

        self._sidebar_visible = not self._sidebar_visible
        self._update_sidebar_visibility()

    def _update_sidebar_visibility(self) -> None:
        """サイドバーの表示状態とトグルボタンの文言を更新する。"""

        if self._sidebar_visible:
            self._sidebar_container.grid()
            self._sidebar_toggle_btn.configure(text='◀ ノード起動パネル')
        else:
            self._sidebar_container.grid_remove()
            self._sidebar_toggle_btn.configure(text='▶ ノード起動パネル')

    def _build_state_summary(self, parent: ttk.Frame) -> None:
        summary = ttk.Frame(parent)
        summary.grid(row=0, column=0, sticky='nsew')
        summary.columnconfigure(0, weight=1, uniform='summary')
        summary.columnconfigure(1, weight=1, uniform='summary')
        summary.columnconfigure(2, weight=1, uniform='summary')

        route_frame = ttk.LabelFrame(
            summary,
            text='ルート進捗 (route_state)',
            style='Card.TLabelframe',
        )
        route_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        route_frame.columnconfigure(1, weight=1)
        ttk.Label(route_frame, text='マネージャ状態').grid(row=0, column=0, sticky='w')
        ttk.Label(route_frame, textvariable=self._route_state_vars.manager).grid(
            row=0,
            column=1,
            sticky='w',
        )
        ttk.Label(route_frame, text='ルート状態').grid(row=1, column=0, sticky='w')
        ttk.Label(route_frame, textvariable=self._route_state_vars.route_status).grid(
            row=1,
            column=1,
            sticky='w',
        )
        ttk.Label(route_frame, text='バージョン').grid(row=2, column=0, sticky='w')
        ttk.Label(route_frame, textvariable=self._route_state_vars.version).grid(
            row=2,
            column=1,
            sticky='w',
        )
        ttk.Label(route_frame, text='進捗率').grid(row=3, column=0, sticky='w')
        progress_frame = ttk.Frame(route_frame)
        progress_frame.grid(row=3, column=1, sticky='ew', pady=2)
        progress_frame.columnconfigure(0, weight=1)
        ttk.Progressbar(
            progress_frame,
            maximum=100,
            variable=self._route_state_vars.progress,
        ).grid(row=0, column=0, sticky='ew')
        ttk.Label(
            progress_frame,
            textvariable=self._route_state_vars.progress_text,
        ).grid(row=0, column=1, sticky='e', padx=(8, 0))
        ttk.Label(route_frame, text='最終イベント').grid(row=4, column=0, sticky='nw')
        ttk.Label(
            route_frame,
            textvariable=self._route_state_vars.detail,
            justify='left',
            wraplength=220,
        ).grid(row=4, column=1, sticky='w')

        follower_frame = ttk.LabelFrame(
            summary,
            text='フォロワ状態 (follower_state)',
            style='Card.TLabelframe',
        )
        follower_frame.grid(row=0, column=1, sticky='nsew', padx=(0, 8))
        follower_frame.columnconfigure(1, weight=1)
        ttk.Label(follower_frame, text='フォロワ状態').grid(row=0, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['state']).grid(
            row=0,
            column=1,
            sticky='w',
        )
        ttk.Label(follower_frame, text='目標ウェイポイント').grid(row=1, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['target_label']).grid(
            row=1,
            column=1,
            sticky='w',
        )
        ttk.Label(follower_frame, text='滞留要因').grid(row=2, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['stagnation']).grid(
            row=2,
            column=1,
            sticky='w',
        )
        ttk.Label(follower_frame, text='前方障害物まで').grid(row=3, column=0, sticky='w')
        ttk.Label(
            follower_frame,
            textvariable=self._follower_vars['front_clearance'],
        ).grid(row=3, column=1, sticky='w')
        ttk.Label(follower_frame, text='回避オフセット').grid(row=4, column=0, sticky='w')
        ttk.Label(follower_frame, textvariable=self._follower_vars['offsets']).grid(
            row=4,
            column=1,
            sticky='w',
        )
        metrics = ttk.Frame(summary)
        metrics.grid(row=0, column=2, sticky='nsew')
        metrics.columnconfigure(0, weight=1)
        metrics.rowconfigure(0, weight=1)
        metrics.rowconfigure(1, weight=1)

        velocity_frame = ttk.LabelFrame(
            metrics,
            text='ロボット速度 (cmd_vel)',
            style='Card.TLabelframe',
        )
        velocity_frame.grid(row=0, column=0, sticky='nsew', pady=(0, 6))
        velocity_frame.columnconfigure(0, weight=1)
        velocity_frame.columnconfigure(1, weight=1)

        speed_frame = ttk.Frame(velocity_frame)
        speed_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 4))
        speed_frame.columnconfigure(0, weight=1)
        speed_frame.columnconfigure(1, weight=1)
        ttk.Label(speed_frame, text='並進速度').grid(row=0, column=0, sticky='w')
        ttk.Label(speed_frame, textvariable=self._velocity_vars['linear']).grid(
            row=0,
            column=1,
            sticky='e',
        )
        ttk.Label(speed_frame, text='角速度').grid(row=1, column=0, sticky='w', pady=(6, 0))
        ttk.Label(speed_frame, textvariable=self._velocity_vars['angular']).grid(
            row=1,
            column=1,
            sticky='e',
            pady=(6, 0),
        )

        distance_frame = ttk.Frame(velocity_frame)
        distance_frame.grid(row=0, column=1, sticky='nsew', padx=(4, 0))
        distance_frame.columnconfigure(0, weight=1)
        distance_frame.columnconfigure(1, weight=1)
        ttk.Label(distance_frame, text='走行距離').grid(row=0, column=0, sticky='w')
        ttk.Label(distance_frame, textvariable=self._velocity_vars['distance']).grid(
            row=0,
            column=1,
            sticky='e',
        )
        ttk.Label(distance_frame, text='走行時間').grid(row=1, column=0, sticky='w', pady=(6, 0))
        ttk.Label(distance_frame, textvariable=self._velocity_vars['elapsed']).grid(
            row=1,
            column=1,
            sticky='e',
            pady=(6, 0),
        )

        target_frame = ttk.LabelFrame(metrics, text='目標までの距離', style='Card.TLabelframe')
        target_frame.grid(row=1, column=0, sticky='nsew')
        target_frame.columnconfigure(0, weight=1)
        target_frame.columnconfigure(1, weight=1)
        ttk.Label(target_frame, text='目標ウェイポイントまで').grid(
            row=0,
            column=0,
            sticky='w',
        )
        ttk.Label(target_frame, textvariable=self._target_vars['distance']).grid(
            row=0,
            column=1,
            sticky='e',
        )
        ttk.Label(target_frame, text='進捗').grid(row=1, column=0, sticky='w', pady=(6, 0))
        target_progress = ttk.Frame(target_frame)
        target_progress.grid(row=1, column=1, sticky='ew', pady=(6, 0))
        target_progress.columnconfigure(0, weight=1)
        ttk.Progressbar(
            target_progress,
            maximum=100,
            variable=self._target_vars['progress'],
        ).grid(row=0, column=0, sticky='ew')
        ttk.Label(target_progress, textvariable=self._target_vars['progress_percent']).grid(
            row=0,
            column=1,
            sticky='e',
            padx=(8, 0),
        )

    def _build_image_row(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=1, column=0, sticky='nsew', pady=(8, 8))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)
        frame.rowconfigure(0, weight=1)

        self._route_panel = ImagePanel(frame, 'ルート地図', size=(640, 360))
        self._route_panel.grid(row=0, column=0, sticky='nsew', padx=(0, 8))

        self._obstacle_panel = ImagePanel(frame, '障害物ビュー', size=(400, 400), enable_overlay=True)
        self._obstacle_panel.grid(row=0, column=1, sticky='nsew', padx=4)

        self._camera_panel = ImagePanel(frame, '外部カメラ', size=CAMERA_DISPLAY_SIZE)
        self._camera_panel.grid(row=0, column=2, sticky='nsew', padx=(8, 0))

        if not self._image_render_available:
            warning = self._image_warning_message
            for panel in (self._route_panel, self._obstacle_panel, self._camera_panel):
                panel.update_image(None, alt_text=warning)

    def _build_control_panel(self, parent: ttk.Frame) -> None:
        container = ttk.Frame(parent)
        container.grid(row=2, column=0, sticky='ew')
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=2)

        banner_frame = ttk.LabelFrame(
            container,
            text='手動・信号・封鎖イベント',
            style='Card.TLabelframe',
            padding=8,
        )
        banner_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        banner_frame.columnconfigure(0, weight=1)
        banner_frame.rowconfigure(0, weight=1)

        self._banner_default_bg = '#f7f9fc'
        self._banner_default_fg = '#2c3e50'
        self._banner_label = tk.Label(
            banner_frame,
            textvariable=self._event_banner,
            font=('Helvetica', 14, 'bold'),
            anchor='center',
            bg=self._banner_default_bg,
            fg=self._banner_default_fg,
            padx=8,
            pady=16,
            wraplength=220,
            justify='center',
        )
        self._banner_label.grid(row=0, column=0, sticky='nsew')
        control_frame = ttk.LabelFrame(container, text='制御コマンド', padding=4)
        control_frame.grid(row=0, column=1, sticky='nsew')
        control_frame.columnconfigure(0, weight=1)

        notebook = ttk.Notebook(control_frame)
        notebook.grid(row=0, column=0, sticky='nsew')

        manual_tab = ttk.Frame(notebook, padding=6)
        manual_tab.columnconfigure(0, weight=1)
        manual_row = ttk.Frame(manual_tab)
        manual_row.grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(manual_row, text='True', value=True, variable=self._manual_value).grid(
            row=0,
            column=0,
            sticky='w',
        )
        ttk.Radiobutton(manual_row, text='False', value=False, variable=self._manual_value).grid(
            row=0,
            column=1,
            sticky='w',
            padx=(12, 0),
        )
        ttk.Button(
            manual_row,
            text='manual_start を送信',
            command=self._on_send_manual,
        ).grid(row=0, column=2, sticky='w', padx=(12, 0))
        ttk.Label(manual_tab, textvariable=self._manual_status_var).grid(
            row=1,
            column=0,
            sticky='w',
            pady=(6, 0),
        )
        notebook.add(manual_tab, text='manual_start')

        sig_tab = ttk.Frame(notebook, padding=6)
        sig_tab.columnconfigure(0, weight=1)
        sig_row = ttk.Frame(sig_tab)
        sig_row.grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(sig_row, text='GO', value=1, variable=self._sig_value).grid(
            row=0,
            column=0,
            sticky='w',
        )
        ttk.Radiobutton(sig_row, text='STOP', value=2, variable=self._sig_value).grid(
            row=0,
            column=1,
            sticky='w',
            padx=(12, 0),
        )
        ttk.Button(
            sig_row,
            text='sig_recog を送信',
            command=self._on_send_sig,
        ).grid(row=0, column=2, sticky='w', padx=(12, 0))
        ttk.Label(sig_tab, textvariable=self._sig_status_var).grid(
            row=1,
            column=0,
            sticky='w',
            pady=(6, 0),
        )
        notebook.add(sig_tab, text='sig_recog')

        obstacle_tab = ttk.Frame(notebook, padding=6)
        obstacle_tab.columnconfigure(0, weight=1)
        value_row = ttk.Frame(obstacle_tab)
        value_row.grid(row=0, column=0, sticky='ew')
        value_row.columnconfigure(7, weight=1)
        ttk.Checkbutton(
            value_row,
            text='front_blocked',
            variable=self._obstacle_blocked,
            command=self._on_obstacle_params_changed,
        ).grid(row=0, column=0, sticky='w', padx=(0, 12))
        ttk.Label(value_row, text='前方距離[m]').grid(row=0, column=1, sticky='e')
        clearance_spin = ttk.Spinbox(
            value_row,
            from_=0.1,
            to=50.0,
            increment=0.1,
            textvariable=self._obstacle_clearance,
            width=6,
            command=self._on_obstacle_params_changed,
        )
        clearance_spin.grid(row=0, column=2, sticky='w', padx=(4, 12))
        ttk.Label(value_row, text='左[m]').grid(row=0, column=3, sticky='e')
        left_spin = ttk.Spinbox(
            value_row,
            from_=-5.0,
            to=5.0,
            increment=0.1,
            textvariable=self._obstacle_left,
            width=6,
            command=self._on_obstacle_params_changed,
        )
        left_spin.grid(row=0, column=4, sticky='w', padx=(4, 12))
        ttk.Label(value_row, text='右[m]').grid(row=0, column=5, sticky='e')
        right_spin = ttk.Spinbox(
            value_row,
            from_=-5.0,
            to=5.0,
            increment=0.1,
            textvariable=self._obstacle_right,
            width=6,
            command=self._on_obstacle_params_changed,
        )
        right_spin.grid(row=0, column=6, sticky='w', padx=(4, 0))

        self._obstacle_toggle_btn = ttk.Button(
            value_row,
            command=self._on_toggle_obstacle_override,
        )
        self._obstacle_toggle_btn.grid(row=0, column=8, sticky='e', padx=(12, 0))
        self._update_obstacle_override_button()

        for spin in (clearance_spin, left_spin, right_spin):
            spin.bind('<FocusOut>', self._on_obstacle_params_changed)
            spin.bind('<Return>', self._on_obstacle_params_changed)
        ttk.Label(
            obstacle_tab,
            textvariable=self._obstacle_override_state,
            anchor='w',
            justify='left',
        ).grid(
            row=1,
            column=0,
            sticky='w',
            pady=(6, 0),
        )
        notebook.add(obstacle_tab, text='obstacle_hint')

        road_tab = ttk.Frame(notebook, padding=6)
        road_tab.columnconfigure(0, weight=1)
        road_row = ttk.Frame(road_tab)
        road_row.grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(road_row, text='True', value=True, variable=self._road_value).grid(
            row=0,
            column=0,
            sticky='w',
        )
        ttk.Radiobutton(road_row, text='False', value=False, variable=self._road_value).grid(
            row=0,
            column=1,
            sticky='w',
            padx=(12, 0),
        )
        ttk.Button(
            road_row,
            text='road_blocked を送信',
            command=self._on_send_road,
        ).grid(row=0, column=2, sticky='w', padx=(12, 0))
        ttk.Label(road_tab, textvariable=self._road_status_var).grid(
            row=1,
            column=0,
            sticky='w',
            pady=(6, 0),
        )
        notebook.add(road_tab, text='road_blocked')

        frame_image_tab = ttk.Frame(notebook, padding=6)
        frame_image_tab.columnconfigure(0, weight=1)
        path_row = ttk.Frame(frame_image_tab)
        path_row.columnconfigure(0, weight=1)
        path_row.grid(row=0, column=0, sticky='ew')
        ttk.Entry(path_row, textvariable=self._frame_image_path_var).grid(
            row=0,
            column=0,
            sticky='ew',
        )
        ttk.Button(
            path_row,
            text='frame_image_path を送信',
            command=self._on_send_frame_image_path,
        ).grid(row=0, column=1, sticky='e', padx=(8, 0))
        ttk.Label(frame_image_tab, textvariable=self._frame_image_status_var).grid(
            row=1,
            column=0,
            sticky='w',
            pady=(6, 0),
        )
        notebook.add(frame_image_tab, text='frame_image_path')

        self._apply_imagetk_warning_if_needed(control_frame)

    def _build_launch_sidebar(self, parent: ttk.Frame) -> None:
        for child in parent.winfo_children():
            child.destroy()
        parent.columnconfigure(0, weight=1)

        ttk.Label(parent, text='ノード起動パネル', style='Bold.TLabel').grid(
            row=0,
            column=0,
            sticky='w',
            pady=(0, 6),
        )

        button_row = ttk.Frame(parent)
        button_row.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        ttk.Button(button_row, text='全起動', command=self._on_launch_all).grid(
            row=0,
            column=0,
            sticky='ew',
            padx=(0, 4),
        )
        ttk.Button(button_row, text='全停止', command=self._core.request_stop_all).grid(
            row=0,
            column=1,
            sticky='ew',
        )

        snapshot = self._core.snapshot()
        self._launch_widgets.clear()
        for idx, (profile_id, state) in enumerate(snapshot.launch_states.items(), start=2):
            card = ttk.LabelFrame(parent, text=state.display_name, padding=6)
            card.grid(row=idx, column=0, sticky='ew', pady=4)
            card.columnconfigure(0, weight=1)

            status_var = tk.StringVar(value=self._format_launch_status(state.status))
            ttk.Label(card, textvariable=status_var).grid(row=0, column=0, sticky='w')

            param_var = tk.StringVar(value=state.selected_param_display or '')
            combo = ttk.Combobox(
                card,
                textvariable=param_var,
                values=state.available_params,
                state='readonly',
            )
            combo.grid(row=1, column=0, sticky='ew', pady=2)
            combo.bind(
                '<<ComboboxSelected>>',
                lambda _e, pid=profile_id, var=param_var: self._core.update_selected_param(
                    pid,
                    var.get(),
                ),
            )

            current_row = 2
            override_widgets: Dict[str, Dict[str, object]] = {}
            if state.user_arguments:
                handled_keys = set()
                if profile_id == 'route_manager' and {
                    'start_label',
                    'goal_label',
                }.issubset(set(state.user_arguments)):
                    current_row = self._build_route_manager_label_inputs(
                        card,
                        profile_id,
                        state,
                        override_widgets,
                        current_row,
                    )
                    handled_keys.update({'start_label', 'goal_label'})
                for arg_key in state.user_arguments:
                    if arg_key in handled_keys:
                        continue
                    label_text = self._format_override_label(arg_key)
                    ttk.Label(card, text=label_text).grid(row=current_row, column=0, sticky='w')
                    current_row += 1
                    var = tk.StringVar(value=state.override_inputs.get(arg_key, ''))
                    entry = ttk.Entry(card, textvariable=var)
                    entry.grid(row=current_row, column=0, sticky='ew', pady=1)
                    callback = self._create_override_callback(profile_id, arg_key, var)
                    trace_id = var.trace_add('write', callback)
                    override_widgets[arg_key] = {
                        'var': var,
                        'callback': callback,
                        'trace': trace_id,
                        'entry': entry,
                        'last_synced': var.get(),
                    }
                    current_row += 1

            launch_mode_var: Optional[tk.Variable] = tk.BooleanVar(
                value=state.use_alternate_launch
            )
            if state.alternate_launch_file:
                if profile_id == 'yolo_detector':
                    launch_mode_var = tk.StringVar(
                        value='yolo' if state.use_alternate_launch else 'yolo_ncnn'
                    )
                    mode_frame = ttk.Frame(card)
                    mode_frame.grid(row=current_row, column=0, sticky='w', pady=(2, 0))

                    def _on_change_mode(*_args: object) -> None:
                        self._core.update_launch_file_selection(
                            profile_id, launch_mode_var.get() == 'yolo'
                        )

                    for column, (label, value) in enumerate(
                        (('yolo_ncnn', 'yolo_ncnn'), ('yolo', 'yolo')),
                        start=0,
                    ):
                        ttk.Radiobutton(
                            mode_frame,
                            text=label,
                            value=value,
                            variable=launch_mode_var,
                            command=_on_change_mode,
                        ).grid(row=0, column=column, sticky='w', padx=(0, 8))
                    current_row += 1
                else:
                    toggle_label = state.launch_toggle_label or '別モードを使用'
                    launch_mode_var = tk.BooleanVar(value=state.use_alternate_launch)
                    callback = self._create_launch_toggle_callback(
                        profile_id, launch_mode_var
                    )
                    chk_launch = ttk.Checkbutton(
                        card,
                        text=toggle_label,
                        variable=launch_mode_var,
                        command=callback,
                    )
                    chk_launch.grid(row=current_row, column=0, sticky='w')
                    current_row += 1
            else:
                launch_mode_var = None

            simulator_var = tk.BooleanVar(value=state.simulator_enabled)
            if state.simulator_launch_file:
                chk = ttk.Checkbutton(
                    card,
                    text='Simulator',
                    variable=simulator_var,
                    command=lambda pid=profile_id, var=simulator_var: self._core.update_simulator_enabled(
                        pid,
                        var.get(),
                    ),
                )
                chk.grid(row=current_row, column=0, sticky='w')
                current_row += 1

            button_frame = ttk.Frame(card)
            button_frame.grid(row=current_row, column=0, sticky='ew', pady=2)
            button_frame.columnconfigure(0, weight=1)
            button_frame.columnconfigure(1, weight=1)
            ttk.Button(
                button_frame,
                text='起動',
                command=lambda pid=profile_id: self._on_launch_node(pid),
            ).grid(row=0, column=0, sticky='ew', padx=(0, 4))
            ttk.Button(
                button_frame,
                text='停止',
                command=lambda pid=profile_id: self._core.request_stop(pid),
            ).grid(row=0, column=1, sticky='ew')
            current_row += 1

            self._launch_widgets[profile_id] = {
                'status': status_var,
                'param': param_var,
                'sim': simulator_var,
                'combo': combo,
                'overrides': override_widgets,
                'launch_toggle': launch_mode_var,
            }

    def _create_override_callback(self, profile_id: str, key: str, var: tk.StringVar):
        def _callback(*_args: object) -> None:
            self._core.update_launch_override(profile_id, key, var.get())

        return _callback

    def _create_launch_toggle_callback(self, profile_id: str, var: tk.BooleanVar):
        def _callback(*_args: object) -> None:
            self._core.update_launch_file_selection(profile_id, var.get())

        return _callback

    def _build_route_manager_label_inputs(
        self,
        card: ttk.LabelFrame,
        profile_id: str,
        state: NodeLaunchState,
        override_widgets: Dict[str, Dict[str, object]],
        start_row: int,
    ) -> int:
        """route_manager 専用の start/goal 入力欄を横並びで構築する。"""

        container = ttk.Frame(card)
        container.grid(row=start_row, column=0, sticky='ew', pady=1)
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)

        def _create_entry(
            column: int, key: str, padding: Tuple[int, int]
        ) -> None:
            frame = ttk.Frame(container)
            frame.grid(row=0, column=column, sticky='ew', padx=padding)
            frame.columnconfigure(0, weight=1)
            label_text = self._format_override_label(key)
            ttk.Label(frame, text=label_text).grid(row=0, column=0, sticky='w')
            var = tk.StringVar(value=state.override_inputs.get(key, ''))
            entry = ttk.Entry(frame, textvariable=var)
            entry.grid(row=1, column=0, sticky='ew', pady=(2, 0))
            callback = self._create_override_callback(profile_id, key, var)
            trace_id = var.trace_add('write', callback)
            override_widgets[key] = {
                'var': var,
                'callback': callback,
                'trace': trace_id,
                'entry': entry,
                'last_synced': var.get(),
            }

        _create_entry(0, 'start_label', (0, 2))
        _create_entry(1, 'goal_label', (2, 0))
        return start_row + 1

    @staticmethod
    def _format_override_label(key: str) -> str:
        label_map = {
            'start_label': 'Start Label',
            'goal_label': 'Goal Label',
            'checkpoint_labels': 'Checkpoint Labels (comma separated)',
        }
        return label_map.get(key, key)

    def _on_launch_node(self, profile_id: str) -> None:
        """個別起動要求時に既存状態を確認してメッセージを表示する。"""

        snapshot = self._get_latest_snapshot()
        state = None
        if snapshot is not None:
            state = snapshot.launch_states.get(profile_id)
        if state and state.status == NodeLaunchStatus.RUNNING:
            name = state.display_name or profile_id
            messagebox.showinfo(
                'ノード再起動',
                f"{name} は既に起動中です。再起動を実行します。",
            )
        self._core.request_launch(profile_id)

    def _on_launch_all(self) -> None:
        """全起動ボタン押下時の処理を実装する。"""

        snapshot = self._get_latest_snapshot()
        running_nodes = []
        if snapshot is not None:
            for state in snapshot.launch_states.values():
                if state.status == NodeLaunchStatus.RUNNING:
                    running_nodes.append(state.display_name or state.profile_id)
        if running_nodes:
            lines = '\n'.join(f'・{name}' for name in running_nodes)
            message = '以下のノードは既に起動中です。再起動を実行します。\n' + lines
            messagebox.showinfo('ノード再起動', message)
        self._core.request_launch_all()

    def _build_logs(self, parent: ttk.Frame) -> None:
        columns = 2
        snapshot = self._core.snapshot()
        ordered = [
            'route_planner',
            'route_manager',
            'route_follower',
            'robot_navigator',
            'obstacle_monitor',
            'yolo_detector',
        ]

        count = sum(1 for pid in ordered if pid in snapshot.launch_states)
        rows = max((count + columns - 1) // columns, 1)
        for col in range(columns):
            parent.columnconfigure(col, weight=1)
        for row in range(rows):
            parent.rowconfigure(row, weight=1)

        self._log_texts.clear()
        self._log_open_buttons.clear()
        self._log_file_paths.clear()
        index = 0
        for profile_id in ordered:
            state = snapshot.launch_states.get(profile_id)
            if state is None:
                continue
            row = index // columns
            col = index % columns
            index += 1
            frame = ttk.LabelFrame(parent, padding=6)
            frame.grid(row=row, column=col, sticky='nsew', padx=6, pady=6)
            frame.columnconfigure(0, weight=1)
            frame.columnconfigure(1, weight=0)
            frame.rowconfigure(0, weight=1)
            frame.rowconfigure(1, weight=0)

            header = ttk.Frame(frame)
            header.columnconfigure(0, weight=1)
            header.columnconfigure(1, weight=0, minsize=50)
            header.columnconfigure(2, weight=0)
            ttk.Label(header, text=state.display_name).grid(
                row=0, column=0, sticky='w'
            )
            button = ttk.Button(
                header,
                text='ログファイルを開く',
                command=lambda pid=profile_id: self._open_log_file(pid),
            )
            button.grid(row=0, column=2, sticky='e', padx=(0, 30))
            frame.configure(labelwidget=header)

            def _sync_header_width(event: tk.Event, hdr: ttk.Frame = header) -> None:
                """フレーム幅に合わせてヘッダー幅を更新する。"""

                hdr.configure(width=max(event.width, 0))

            frame.bind('<Configure>', _sync_header_width, add='+')

            text = tk.Text(frame, wrap='none', state='disabled')
            text.grid(row=0, column=0, sticky='nsew', pady=(6, 0))
            v_scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
            v_scrollbar.grid(row=0, column=1, sticky='ns', pady=(6, 0))
            h_scrollbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
            h_scrollbar.grid(row=1, column=0, sticky='ew')
            text.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)
            self._log_texts[profile_id] = text
            button.state(['disabled'])
            self._log_open_buttons[profile_id] = button
            self._log_file_paths[profile_id] = None

    def _build_params(self, parent: ttk.Frame) -> None:
        columns = 2
        snapshot = self._core.snapshot()
        ordered = [
            'route_planner',
            'route_manager',
            'route_follower',
            'robot_navigator',
            'obstacle_monitor',
            'yolo_detector',
        ]

        count = sum(1 for pid in ordered if pid in snapshot.launch_states)
        rows = max((count + columns - 1) // columns, 1)
        for col in range(columns):
            parent.columnconfigure(col, weight=1)
        for row in range(rows):
            parent.rowconfigure(row, weight=1)

        self._param_texts.clear()
        self._param_last_display.clear()
        index = 0
        for profile_id in ordered:
            state = snapshot.launch_states.get(profile_id)
            if state is None:
                continue
            row = index // columns
            col = index % columns
            index += 1
            frame = ttk.LabelFrame(parent, text=state.display_name, padding=6)
            frame.grid(row=row, column=col, sticky='nsew', padx=6, pady=6)
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)
            text = tk.Text(frame, wrap='none', state='disabled')
            text.grid(row=0, column=0, sticky='nsew')
            v_scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
            v_scrollbar.grid(row=0, column=1, sticky='ns')
            h_scrollbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
            h_scrollbar.grid(row=1, column=0, sticky='ew')
            text.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)
            self._param_texts[profile_id] = text

    # ---------- コマンドハンドラ ----------
    def _on_send_manual(self) -> None:
        value = bool(self._manual_value.get())
        self._core.request_manual_start(value)

    def _on_send_sig(self) -> None:
        go = self._sig_value.get() == 1
        self._core.request_sig_recog(go)
        self._sig_last_command = 1 if go else 2

    def _on_send_road(self) -> None:
        value = bool(self._road_value.get())
        self._core.request_road_blocked(value)

    def _on_send_frame_image_path(self) -> None:
        path = self._frame_image_path_var.get()
        self._core.request_frame_image_path(path)
        timestamp = datetime.now(timezone.utc)
        self._frame_image_status_var.set(f"最終送信: {_format_time(timestamp)}")

    def _on_toggle_obstacle_override(self) -> None:
        if self._obstacle_override_active:
            self._core.stop_obstacle_override()
            self._obstacle_override_active = False
        else:
            self._core.start_obstacle_override(
                self._obstacle_blocked.get(),
                self._obstacle_clearance.get(),
                self._obstacle_left.get(),
                self._obstacle_right.get(),
            )
            self._obstacle_override_active = True
        self._update_obstacle_override_button()

    def _on_obstacle_params_changed(self, *_args) -> None:
        """障害物ヒント送信パラメータ変更時のフック。"""

        # GUI 上では受信状態のみを表示するため、変更時の追加処理は行わない。
        return

    def _update_obstacle_override_button(self) -> None:
        """障害物ヒント送出ボタンの文言を更新する。"""

        if self._obstacle_toggle_btn is None:
            return
        label = '送出停止' if self._obstacle_override_active else '送出開始'
        self._obstacle_toggle_btn.configure(text=label)

    # ---------- 更新処理 ----------
    def _schedule_update(self) -> None:
        if self._closing:
            return
        try:
            self._update_job = self._root.after(REFRESH_INTERVAL_MS, self._refresh)
        except tk.TclError:
            self._update_job = None

    def _refresh(self) -> None:
        try:
            snapshot = self._core.snapshot()
            self._latest_snapshot = snapshot
            self._apply_snapshot(snapshot)
        except Exception as exc:  # pylint: disable=broad-except
            LOGGER.exception("スナップショット更新処理で例外が発生しました: %s", exc)
        finally:
            if not self._closing:
                self._schedule_update()

    def _apply_snapshot(self, snapshot: GuiSnapshot) -> None:
        route = snapshot.route_state
        follower = snapshot.follower_state
        manager_state = route.manager_state.upper() if route.manager_state else 'UNKNOWN'
        route_status = route.route_status.upper() if route.route_status else 'UNKNOWN'
        self._route_state_vars.manager.set(manager_state)
        self._route_state_vars.route_status.set(route_status)
        progress_ratio, display_index, total_waypoints = compute_route_progress(
            route, follower
        )
        self._route_state_vars.progress.set(progress_ratio * 100.0)
        self._route_state_vars.progress_text.set(
            f"{display_index} / {total_waypoints}"
        )
        self._route_state_vars.version.set(f"バージョン: {route.route_version}")
        detail_entries = []
        if route.last_replan_reason:
            detail_entries.append(f"Ev: {route.last_replan_reason}")
        manager_tokens = []
        if route.manager_decision:
            manager_tokens.append(route.manager_decision)
        if route.manager_cause:
            manager_tokens.append(route.manager_cause)
        if manager_tokens:
            manager_text = ' / '.join(manager_tokens)
            detail_entries.append(f"Mgr: {manager_text}")
        self._route_state_vars.detail.set(
            '\n'.join(detail_entries) or '最新イベントなし'
        )

        follower_state_text = follower.state
        if follower.state == 'WAITING_STOP':
            if follower.signal_stop_active:
                follower_state_text = f'{follower.state} (信号)'
            elif follower.line_stop_active:
                follower_state_text = f'{follower.state} (停止線)'
        self._follower_vars['state'].set(follower_state_text)
        self._follower_vars['index'].set(f"Index: {follower.active_waypoint_index}")
        current_label = follower.active_waypoint_label or route.current_label or '-'
        label_text = f"現在: {current_label}"
        self._follower_vars['label'].set(label_text)
        reason = (follower.stagnation_reason or '').strip()
        if follower.state == 'RUNNING':
            if self._stagnation_display_reason != '滞留なし':
                self._stagnation_ignored_reason = self._stagnation_display_reason
            else:
                self._stagnation_ignored_reason = None
            self._stagnation_retry_baseline = follower.retry_count
            self._stagnation_display_reason = '滞留なし'
        else:
            if reason:
                if (
                    self._stagnation_retry_baseline is None
                    or follower.retry_count > self._stagnation_retry_baseline
                    or (
                        self._stagnation_display_reason != reason
                        and reason != self._stagnation_ignored_reason
                    )
                ):
                    self._stagnation_display_reason = reason
                    self._stagnation_retry_baseline = follower.retry_count
                    self._stagnation_ignored_reason = None
            else:
                self._stagnation_display_reason = '滞留なし'
                self._stagnation_retry_baseline = follower.retry_count
                self._stagnation_ignored_reason = None
        self._follower_vars['stagnation'].set(self._stagnation_display_reason)

        front_clearance = follower.front_clearance_m
        if math.isinf(front_clearance):
            front_clearance_text = '障害物なし'
        else:
            front_clearance_text = f"{front_clearance:.2f}m"
        if follower.front_blocked:
            front_clearance_text += '(blocked)'
        self._follower_vars['front_clearance'].set(front_clearance_text)
        offsets = f"左:{follower.left_offset_m:+.2f}m / 右:{follower.right_offset_m:+.2f}m"
        self._follower_vars['offsets'].set(offsets)

        self._velocity_vars['linear'].set(f"{snapshot.cmd_vel.linear_mps:.2f} m/s")
        self._velocity_vars['angular'].set(f"{snapshot.cmd_vel.angular_dps:.1f} deg/s")
        self._update_route_manager_status(snapshot)
        self._update_distance_tracker(snapshot)
        self._update_follower_run_time(snapshot.follower_state)

        target = snapshot.target_distance
        baseline = max(target.baseline_distance_m, 0.0)
        remaining = max(target.current_distance_m, 0.0)
        ratio: float
        if baseline > 0.0:
            completed = baseline - remaining
            completed = max(min(completed, baseline), 0.0)
            ratio = completed / baseline
            ratio = max(min(ratio, 1.0), 0.0)
            self._last_target_progress_ratio = ratio
        else:
            if remaining <= 0.0:
                ratio = 0.0
                self._last_target_progress_ratio = 0.0
            elif self._last_target_progress_ratio is not None:
                ratio = self._last_target_progress_ratio
            else:
                ratio = 0.0
        self._target_vars['distance'].set(f"{target.current_distance_m:.1f} m")
        self._target_vars['progress'].set(ratio * 100.0)
        self._target_vars['progress_percent'].set(f"{ratio * 100.0:.1f}%")

        self._update_line_stop_tracker(snapshot.follower_state.line_stop_active)
        banner_text, banner_bg, banner_fg, banner_ts = self._resolve_banner(snapshot)
        display_text = ''
        if banner_text:
            display_text = f"{banner_text}\n更新: {_format_time(banner_ts)}"
        self._event_banner.set(display_text)
        self._banner_label.configure(bg=banner_bg, fg=banner_fg)

        manual = snapshot.manual_signal
        self._manual_status_var.set(
            f"現在値: {manual.manual_start} / 最終送信: {_format_time(manual.manual_timestamp)}"
        )
        if manual.sig_recog in (1, 2):
            if not self._sig_value_initialized:
                self._sig_value.set(manual.sig_recog)
                self._sig_value_initialized = True
            elif (
                manual.sig_recog != self._sig_last_command
                and self._sig_value.get() != manual.sig_recog
            ):
                self._sig_value.set(manual.sig_recog)
        sig_label = {1: 'GO', 2: 'STOP'}.get(manual.sig_recog, '未定義')
        if manual.sig_recog is None:
            self._sig_status_var.set('最終送信: --:--:--')
        else:
            self._sig_status_var.set(
                f"最終送信: {sig_label} @{_format_time(manual.sig_timestamp)}"
            )
        source_label = self._translate_road_source(manual.road_blocked_source)
        self._road_status_var.set(
            (
                "現在値: "
                f"{manual.road_blocked} / 最終送信: "
                f"{_format_time(manual.road_blocked_timestamp)} / 入力元: {source_label}"
            )
        )

        hint = snapshot.obstacle_hint
        self._obstacle_override_active = hint.override_active
        obstacle_status = (
            f"状態: front_blocked={'ON' if hint.front_blocked else 'OFF'} "
            f"前方距離:{hint.front_clearance_m:.2f}m "
            f"左:{hint.left_offset_m:+.2f}m 右:{hint.right_offset_m:+.2f}m "
            f"更新:{_format_time(hint.updated_at)}"
        )
        self._obstacle_override_state.set(obstacle_status)
        self._update_obstacle_override_button()

        self._update_images(snapshot)
        self._update_launch_states(snapshot)
        self._update_param_views(snapshot)
        self._update_logs(snapshot)
        self._update_log_buttons(snapshot)
        if self._shutdown_pending and not self._has_active_nodes(snapshot):
            self._finalize_shutdown()

    def _update_route_manager_status(self, snapshot: GuiSnapshot) -> None:
        """route_manager の起動状態に応じて距離とタイマーを初期化する。"""

        state = snapshot.launch_states.get('route_manager')
        current = state.status if state else None
        if current == NodeLaunchStatus.RUNNING and (
            self._route_manager_previous_status != NodeLaunchStatus.RUNNING
        ):
            self._reset_distance_and_timer()
        self._route_manager_previous_status = current

    def _reset_distance_and_timer(self) -> None:
        """距離・時間メトリクスを初期化し表示を更新する。"""

        self._segment_distances.clear()
        self._total_distance_m = 0.0
        self._velocity_vars['distance'].set('0.0 m')
        self._follower_run_started_at = None
        self._follower_elapsed_seconds = 0.0
        self._velocity_vars['elapsed'].set('00:00:00')
        self._follower_last_state = None

    def _update_distance_tracker(self, snapshot: GuiSnapshot) -> None:
        """進捗分母に利用した距離の合計を更新する。"""

        follower = snapshot.follower_state
        segment_length = max(follower.segment_length_m, 0.0)
        baseline = max(snapshot.target_distance.baseline_distance_m, 0.0)
        distance_value = segment_length if segment_length > 0.0 else baseline
        token = (
            snapshot.route_state.route_version,
            follower.active_waypoint_index,
            follower.active_waypoint_label or '',
        )
        if follower.active_waypoint_index >= 0 and distance_value > 0.0:
            previous = self._segment_distances.get(token)
            if previous is None:
                self._segment_distances[token] = distance_value
                self._total_distance_m += distance_value
            elif not math.isclose(previous, distance_value, rel_tol=1e-6, abs_tol=1e-6):
                self._total_distance_m += distance_value - previous
                self._segment_distances[token] = distance_value
        self._velocity_vars['distance'].set(f"{self._total_distance_m:.1f} m")

    def _update_follower_run_time(self, follower: FollowerStateView) -> None:
        """フォロワが走行を開始してからの経過時間を算出する。"""

        current_state = (follower.state or '').upper()
        now_utc = datetime.now(timezone.utc)
        if self._follower_last_state == 'IDLE' and current_state == 'RUNNING':
            self._follower_run_started_at = now_utc
            self._follower_elapsed_seconds = 0.0
        if current_state == 'RUNNING':
            if self._follower_run_started_at is None:
                self._follower_run_started_at = now_utc
                self._follower_elapsed_seconds = 0.0
            else:
                delta = now_utc - self._follower_run_started_at
                self._follower_elapsed_seconds = max(delta.total_seconds(), 0.0)
        elif current_state == 'IDLE':
            self._follower_run_started_at = None
            self._follower_elapsed_seconds = 0.0
        self._follower_last_state = current_state
        self._velocity_vars['elapsed'].set(
            _format_duration_seconds(self._follower_elapsed_seconds)
        )

    def _update_line_stop_tracker(self, active: bool) -> None:
        """停止線待ちの立ち上がり時刻を記録する。"""

        now_utc = datetime.now(timezone.utc)
        if active:
            if not self._last_line_stop_state or self._line_stop_active_since is None:
                self._line_stop_active_since = now_utc
        else:
            self._line_stop_active_since = None
        self._last_line_stop_state = active

    def _resolve_banner(self, snapshot: GuiSnapshot) -> Tuple[str, str, str, Optional[datetime]]:
        """ダッシュボードのイベントバナー表示内容と配色を決定する。"""

        base_text, timestamp = self._build_banner(snapshot)
        if not base_text:
            return base_text, self._banner_default_bg, self._banner_default_fg, timestamp

        background = self._banner_default_bg
        foreground = self._banner_default_fg

        if base_text.startswith('道路封鎖'):
            background = '#c0392b'
            foreground = '#ffffff'
        elif base_text.startswith('信号: GO'):
            background = '#2980b9'
            foreground = '#ffffff'
        elif base_text.startswith('信号: STOP'):
            background = '#d35400'
            foreground = '#ffffff'
        elif base_text.startswith('停止線: STOP'):
            background = '#8e44ad'
            foreground = '#ffffff'
        elif base_text.startswith('manual_start'):
            background = '#16a085'
            foreground = '#ffffff'

        return base_text, background, foreground, timestamp

    def _build_banner(self, snapshot: GuiSnapshot) -> Tuple[str, Optional[datetime]]:
        current_time = datetime.now(timezone.utc)
        events: List[Tuple[datetime, int, str, Optional[datetime]]] = []

        def add_event(text: str, timestamp: Optional[datetime]) -> None:
            raw_ts = timestamp or current_time
            ignore_ttl = text in STICKY_BANNER_TEXTS
            if not ignore_ttl and not self._is_recent(raw_ts, current_time):
                return
            if raw_ts.tzinfo is None:
                normalized = raw_ts.replace(tzinfo=timezone.utc)
            else:
                normalized = raw_ts.astimezone(timezone.utc)
            order = len(events)
            events.append((normalized, order, text, raw_ts))

        manual = snapshot.manual_signal
        follower = snapshot.follower_state

        if manual.road_blocked:
            add_event('道路封鎖アラート', manual.road_blocked_timestamp)

        if follower.state == 'WAITING_STOP':
            if follower.signal_stop_active:
                add_event('信号: STOP', manual.sig_timestamp)
            elif follower.line_stop_active:
                add_event('停止線: STOP', self._line_stop_active_since)

        if manual.sig_recog == 1:
            add_event('信号: GO', manual.sig_timestamp)
        elif manual.sig_recog == 2:
            add_event('信号: STOP', manual.sig_timestamp)

        if manual.manual_start:
            add_event('manual_start: True', manual.manual_timestamp)

        if not events:
            if self._sticky_stop_banner is not None:
                return self._sticky_stop_banner
            return '', None

        events.sort(key=lambda item: (item[0], item[1]))
        _, _, text, original_ts = events[-1]
        if text in STICKY_BANNER_TEXTS:
            self._sticky_stop_banner = (text, original_ts)
        elif self._sticky_stop_banner is not None and self._sticky_stop_banner[0] != text:
            self._sticky_stop_banner = None
        return text, original_ts

    @staticmethod
    def _is_recent(timestamp: Optional[datetime], current_time: datetime) -> bool:
        """指定時刻がイベントバナー表示期間内かを判定する。"""

        if timestamp is None:
            return False
        if timestamp.tzinfo is None:
            ts_utc = timestamp.replace(tzinfo=timezone.utc)
        else:
            ts_utc = timestamp.astimezone(timezone.utc)
        delta = current_time - ts_utc
        if delta.total_seconds() < 0:
            return True
        return delta <= EVENT_BANNER_TTL

    @staticmethod
    def _translate_road_source(source: Optional[str]) -> str:
        """road_blocked の入力元を日本語ラベルへ変換する。"""

        mapping = {
            'console': 'GUI',
            'external': '外部',
        }
        if not source:
            return '不明'
        return mapping.get(source, source)

    def _update_images(self, snapshot: GuiSnapshot) -> None:
        if not self._image_render_available:
            return

        def _build_photo(
            source: Optional[Image.Image],
            panel: ImagePanel,
        ) -> Optional[tk.PhotoImage]:
            if source is None:
                return None
            working = source.copy() if hasattr(source, 'copy') else source
            try:
                target_size = panel.get_display_size()
                if hasattr(working, 'size') and working.size != target_size:
                    working = resize_with_letter_box(
                        working, target_size, background_color=(31, 31, 31)
                    )
            except Exception:  # pragma: no cover - サイズ調整失敗時は元画像を使用
                pass
            return self._create_photo_image(working)

        route_source = snapshot.images.route_map
        route_placeholder = self._is_placeholder_image(route_source)
        route_photo = _build_photo(route_source, self._route_panel)
        self._route_panel.update_image(route_photo, alt_text='画像未取得')
        self._route_panel.update_caption(
            'ルート地図:表示中'
            if route_photo and not route_placeholder
            else 'ルート地図:未受信'
        )

        obstacle_source = snapshot.images.obstacle_view
        obstacle_placeholder = self._is_placeholder_image(obstacle_source)
        obstacle_photo = _build_photo(obstacle_source, self._obstacle_panel)
        self._obstacle_panel.update_image(obstacle_photo, alt_text='画像未取得')
        self._obstacle_panel.update_overlay('')
        self._obstacle_panel.update_caption(
            '障害物ビュー:表示中'
            if obstacle_photo and not obstacle_placeholder
            else '障害物ビュー:未受信'
        )

        camera_source = snapshot.images.external_camera
        camera_placeholder = self._is_placeholder_image(camera_source)
        camera_photo = _build_photo(camera_source, self._camera_panel)
        self._camera_panel.update_image(camera_photo, alt_text='画像未取得')
        topic_name = snapshot.images.camera_mode or '外部カメラ'
        camera_status = '表示中' if camera_photo and not camera_placeholder else '未受信'
        self._camera_panel.update_caption(f"{topic_name}:{camera_status}")

    def _create_photo_image(self, image: Image.Image) -> Optional[tk.PhotoImage]:
        """Pillow Image から tk.PhotoImage を生成する。"""

        if PIL_IMAGETK_AVAILABLE and ImageTk is not None:
            return ImageTk.PhotoImage(image)
        if CV2_AVAILABLE and cv2 is not None and np is not None and Image is not None:
            return self._create_photo_image_via_cv(image)
        return None

    @staticmethod
    def _is_placeholder_image(image: Optional[Image.Image]) -> bool:
        """プレースホルダ画像かどうかを判定する。"""

        if image is None:
            return False
        info = getattr(image, 'info', None)
        if not isinstance(info, dict):
            return False
        return bool(info.get('placeholder_tag'))

    def _create_photo_image_via_cv(self, image: Image.Image) -> Optional[tk.PhotoImage]:
        """OpenCV を用いて Pillow Image を PNG 化し tk.PhotoImage を生成する。"""

        rgb_image = image.convert('RGB')
        array = np.array(rgb_image)
        bgr_array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        success, buffer = cv2.imencode('.png', bgr_array)
        if not success:
            return None
        data = base64.b64encode(buffer.tobytes()).decode('ascii')
        return tk.PhotoImage(data=data, format='png')

    def _update_launch_states(self, snapshot: GuiSnapshot) -> None:
        for profile_id, widgets in self._launch_widgets.items():
            state = snapshot.launch_states.get(profile_id)
            if not state:
                continue
            status_var = widgets['status']  # type: ignore[assignment]
            if isinstance(status_var, tk.StringVar):
                status_var.set(self._format_launch_status(state.status))
            param_var = widgets['param']  # type: ignore[assignment]
            if isinstance(param_var, tk.StringVar) and state.selected_param_display != param_var.get():
                param_var.set(state.selected_param_display or '')
            sim_var = widgets['sim']  # type: ignore[assignment]
            if isinstance(sim_var, tk.BooleanVar):
                sim_var.set(state.simulator_enabled)
            launch_toggle_var = widgets.get('launch_toggle')  # type: ignore[index]
            if isinstance(launch_toggle_var, tk.BooleanVar):
                launch_toggle_var.set(state.use_alternate_launch)
            elif isinstance(launch_toggle_var, tk.StringVar):
                desired_value = 'yolo' if state.use_alternate_launch else 'yolo_ncnn'
                launch_toggle_var.set(desired_value)
            combo_widget = widgets.get('combo')  # type: ignore[index]
            if isinstance(combo_widget, ttk.Combobox):
                current_values = tuple(combo_widget['values'])
                desired_values = tuple(state.available_params)
                if current_values != desired_values:
                    combo_widget['values'] = state.available_params
            override_entries = widgets.get('overrides')  # type: ignore[index]
            if isinstance(override_entries, dict):
                for key, holder in override_entries.items():
                    var = None
                    last_synced: Optional[str] = None
                    if isinstance(holder, dict):
                        candidate = holder.get('var')
                        if isinstance(candidate, tk.StringVar):
                            var = candidate
                        synced_candidate = holder.get('last_synced')
                        if isinstance(synced_candidate, str):
                            last_synced = synced_candidate
                    elif isinstance(holder, tk.StringVar):
                        var = holder
                    if isinstance(var, tk.StringVar):
                        desired = state.override_inputs.get(key, '')
                        if last_synced is not None and desired == last_synced:
                            continue
                        if var.get() != desired:
                            var.set(desired)
                        if isinstance(holder, dict):
                            holder['last_synced'] = desired

    def _update_logs(self, snapshot: GuiSnapshot) -> None:
        for profile_id, text_widget in self._log_texts.items():
            logs = snapshot.console_logs.get(profile_id)
            if logs is None:
                continue
            try:
                x_first, _ = text_widget.xview()
            except tk.TclError:
                x_first = 0.0
            try:
                y_first, y_last = text_widget.yview()
            except tk.TclError:
                y_first, y_last = 0.0, 1.0
            follow_tail = y_last >= 0.999
            text_widget.configure(state='normal')
            text_widget.delete('1.0', tk.END)
            text_widget.insert(tk.END, ''.join(logs))
            if follow_tail:
                text_widget.see(tk.END)
            else:
                text_widget.yview_moveto(max(min(y_first, 1.0), 0.0))
            text_widget.xview_moveto(max(min(x_first, 1.0), 0.0))
            text_widget.configure(state='disabled')

    def _update_log_buttons(self, snapshot: GuiSnapshot) -> None:
        for profile_id, button in self._log_open_buttons.items():
            path = snapshot.console_log_paths.get(profile_id)
            if path:
                self._log_file_paths[profile_id] = path
                try:
                    button.state(['!disabled'])
                except tk.TclError:
                    continue
            else:
                self._log_file_paths[profile_id] = None
                try:
                    button.state(['disabled'])
                except tk.TclError:
                    continue

    def _update_param_views(self, snapshot: GuiSnapshot) -> None:
        for profile_id, text_widget in self._param_texts.items():
            state = snapshot.launch_states.get(profile_id)
            if state is None:
                continue
            display_text = self._generate_param_display(profile_id, state)
            last_text = self._param_last_display.get(profile_id)
            if display_text == last_text:
                continue
            try:
                x_first, _ = text_widget.xview()
            except tk.TclError:
                x_first = 0.0
            try:
                y_first, y_last = text_widget.yview()
            except tk.TclError:
                y_first, y_last = 0.0, 1.0
            follow_tail = y_last >= 0.999
            text_widget.configure(state='normal')
            text_widget.delete('1.0', tk.END)
            text_widget.insert(tk.END, display_text)
            if follow_tail:
                text_widget.see(tk.END)
            else:
                text_widget.yview_moveto(max(min(y_first, 1.0), 0.0))
            text_widget.xview_moveto(max(min(x_first, 1.0), 0.0))
            text_widget.configure(state='disabled')
            self._param_last_display[profile_id] = display_text

    def _open_log_file(self, profile_id: str) -> None:
        path = self._log_file_paths.get(profile_id)
        if not path:
            messagebox.showinfo('ログファイル未保存', 'このノードのログファイルはまだ作成されていません。')
            return
        file_path = Path(path)
        if not file_path.exists():
            messagebox.showinfo(
                'ログファイル未存在', f'ログファイルが見つかりません: {file_path}'
            )
            return
        try:
            subprocess.Popen(['gedit', str(file_path)])
        except OSError as exc:
            messagebox.showerror('ログファイル起動エラー', f'gedit の起動に失敗しました: {exc}')

    def _generate_param_display(self, profile_id: str, state: NodeLaunchState) -> str:
        target_path, error_message = self._resolve_param_target(profile_id, state)
        if error_message:
            return error_message
        if target_path is None:
            return '表示対象のファイルが特定できません。'
        content = self._read_text_file(target_path)
        if content is None:
            return f'ファイルを読み取れませんでした: {target_path}'
        return f"# 表示ファイル: {target_path}\n\n{content}"

    def _resolve_param_target(
        self, profile_id: str, state: NodeLaunchState
    ) -> Tuple[Optional[Path], Optional[str]]:
        selected_param = state.selected_param
        if not selected_param:
            return None, 'パラメータファイルが選択されていません。'
        param_path = Path(selected_param)
        if not param_path.exists():
            return None, f'パラメータファイルが存在しません: {param_path}'
        if profile_id == 'route_planner':
            route_path, error_message = self._resolve_route_planner_route_path(param_path)
            return route_path, error_message
        return param_path, None

    def _resolve_route_planner_route_path(
        self, param_path: Path
    ) -> Tuple[Optional[Path], Optional[str]]:
        raw_text = self._read_text_file(param_path)
        if raw_text is None:
            return None, f'パラメータファイルを読み取れません: {param_path}'
        config_path_str = self._extract_config_yaml_path(raw_text)
        if not config_path_str:
            return None, f'config_yaml_path が設定されていません: {param_path}'
        candidate = Path(config_path_str).expanduser()
        if candidate.is_absolute():
            if candidate.exists():
                return candidate, None
            return None, f'ルート定義ファイルが存在しません: {candidate}'

        base_dirs = [param_path.parent]
        parent_parent = param_path.parent.parent
        if parent_parent != param_path.parent:
            base_dirs.append(parent_parent)

        for base_dir in base_dirs:
            joined = (base_dir / candidate).expanduser()
            if joined.exists():
                return joined, None

        fallback = (base_dirs[0] / candidate).expanduser()
        return None, f'ルート定義ファイルが存在しません: {fallback}'

    def _extract_config_yaml_path(self, raw_text: str) -> Optional[str]:
        if yaml is not None:
            try:
                parsed = yaml.safe_load(raw_text)
            except yaml.YAMLError:  # type: ignore[attr-defined]
                parsed = None
            if parsed is not None:
                result = self._search_config_path(parsed)
                if result:
                    return result

        for line in raw_text.splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            if stripped.startswith('config_yaml_path'):
                _, _, remainder = stripped.partition(':')
                value = remainder.strip().strip("'\"")
                if value:
                    return value
        return None

    def _search_config_path(self, node: Any) -> Optional[str]:
        if isinstance(node, dict):
            value = node.get('config_yaml_path')
            if isinstance(value, str) and value:
                return value
            for child in node.values():
                result = self._search_config_path(child)
                if result:
                    return result
        elif isinstance(node, list):
            for child in node:
                result = self._search_config_path(child)
                if result:
                    return result
        return None

    def _read_text_file(self, path: Path) -> Optional[str]:
        cache_key = str(path)
        try:
            stat_result = path.stat()
        except OSError:
            self._file_cache.pop(cache_key, None)
            return None
        cached = self._file_cache.get(cache_key)
        mtime = stat_result.st_mtime
        if cached and cached[0] == mtime:
            return cached[1]
        try:
            text = path.read_text(encoding='utf-8')
        except (OSError, UnicodeDecodeError):
            self._file_cache.pop(cache_key, None)
            return None
        self._file_cache[cache_key] = (mtime, text)
        return text

    def _apply_imagetk_warning_if_needed(self, parent: Optional[ttk.Frame] = None) -> None:
        """画像描画に必要な依存が無い場合に警告ラベルを表示する。"""

        if parent is not None:
            self._image_warning_parent = parent

        parent_frame = self._image_warning_parent
        if parent_frame is None:
            return

        if self._image_render_available:
            if self._image_warning_label is not None:
                self._image_warning_label.destroy()
                self._image_warning_label = None
            return

        if self._image_warning_label is None:
            self._image_warning_label = ttk.Label(
                parent_frame,
                text=self._image_warning_message,
                foreground='#c0392b',
                wraplength=360,
                justify='left',
            )
            self._image_warning_label.grid(row=1, column=0, sticky='ew', pady=(8, 0))
        else:
            self._image_warning_label.configure(text=self._image_warning_message)

    def _format_launch_status(self, status: NodeLaunchStatus) -> str:
        mapping = {
            NodeLaunchStatus.STOPPED: '停止',
            NodeLaunchStatus.STARTING: '起動中',
            NodeLaunchStatus.RUNNING: '稼働中',
            NodeLaunchStatus.STOPPING: '停止中',
            NodeLaunchStatus.ERROR: 'エラー',
        }
        return mapping.get(status, '不明')

    def _on_close_request(self) -> None:
        """ウィンドウクローズ要求を受け、必要に応じてノード停止を実行する。"""

        if self._closing or self._shutdown_pending:
            return
        snapshot = self._get_latest_snapshot()
        if not self._has_active_nodes(snapshot):
            self._finalize_shutdown()
            return
        self._shutdown_pending = True
        self._core.request_stop_all()
        self._schedule_shutdown_check()

    def _get_latest_snapshot(self) -> Optional[GuiSnapshot]:
        """最新のスナップショットを取得し、取得できなければ直近値を返す。"""

        try:
            snapshot = self._core.snapshot()
        except Exception:  # pylint: disable=broad-except
            return self._latest_snapshot
        self._latest_snapshot = snapshot
        return snapshot

    def _schedule_shutdown_check(self) -> None:
        """ノード停止完了を確認するポーリングを設定する。"""

        if self._closing:
            return
        try:
            self._shutdown_check_job = self._root.after(
                REFRESH_INTERVAL_MS,
                self._wait_for_shutdown_completion,
            )
        except tk.TclError:
            self._shutdown_check_job = None

    def _wait_for_shutdown_completion(self) -> None:
        """ノードがすべて停止したかを確認し、停止済みなら終了する。"""

        self._shutdown_check_job = None
        if self._closing:
            return
        snapshot = self._get_latest_snapshot()
        if snapshot is None or self._has_active_nodes(snapshot):
            if self._shutdown_pending:
                self._schedule_shutdown_check()
            return
        self._finalize_shutdown()

    def _finalize_shutdown(self) -> None:
        """更新ループを停止し、Tk アプリケーションを終了する。"""

        if self._closing:
            return
        self._closing = True
        self._shutdown_pending = False
        self._cancel_after_job(self._update_job)
        self._update_job = None
        self._cancel_after_job(self._shutdown_check_job)
        self._shutdown_check_job = None
        try:
            self._root.destroy()
        except tk.TclError:
            LOGGER.debug("ウィンドウ破棄済みのため destroy をスキップしました。")

    def _cancel_after_job(self, job_id: Optional[str]) -> None:
        """after で登録したジョブがあればキャンセルする。"""

        if job_id is None:
            return
        try:
            self._root.after_cancel(job_id)
        except tk.TclError:
            LOGGER.debug("after_cancel 失敗 (既に破棄済み): %s", job_id)

    @staticmethod
    def _has_active_nodes(snapshot: Optional[GuiSnapshot]) -> bool:
        """スナップショット内に稼働中のノードが存在するかを判定する。"""

        if snapshot is None:
            return False
        for state in snapshot.launch_states.values():
            if UiMain._is_state_active(state):  # pylint: disable=protected-access
                return True
        return False

    @staticmethod
    def _is_state_active(state: NodeLaunchState) -> bool:
        """個別ノード状態が稼働中かどうかを判定する。"""

        if state.status in (
            NodeLaunchStatus.STARTING,
            NodeLaunchStatus.RUNNING,
            NodeLaunchStatus.STOPPING,
        ):
            return True
        if state.process_id is not None:
            return True
        if state.simulator_process_id is not None:
            return True
        return False

    def run(self) -> None:
        """tkinter メインループを開始する。"""

        self._root.mainloop()


__all__ = ['UiMain']
