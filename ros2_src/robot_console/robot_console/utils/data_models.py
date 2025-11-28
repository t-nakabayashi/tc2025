"""robot_console で使用するデータモデル定義。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import Deque, Dict, List, Optional
import threading


class NodeLaunchStatus(Enum):
    """ノード起動管理の状態を表す列挙型。"""

    STOPPED = auto()
    STARTING = auto()
    RUNNING = auto()
    STOPPING = auto()
    ERROR = auto()


@dataclass
class RouteStateView:
    """route_state と manager_status を統合した表示用データ。"""

    manager_state: str = "unknown"
    route_status: str = "unknown"
    current_index: int = 0
    total_waypoints: int = 0
    progress: float = 0.0
    current_label: str = ""
    last_replan_reason: str = ""
    last_replan_time: Optional[datetime] = None
    route_version: int = 0
    manager_decision: str = ""
    manager_cause: str = ""
    manager_updated_at: Optional[datetime] = None


@dataclass
class FollowerStateView:
    """follower_state の要約情報。"""

    state: str = "unknown"
    active_waypoint_index: int = 0
    active_waypoint_label: str = ""
    front_blocked: bool = False
    front_clearance_m: float = float('inf')
    segment_length_m: float = 0.0
    left_offset_m: float = 0.0
    right_offset_m: float = 0.0
    stagnation_reason: str = ""
    retry_count: int = 0
    signal_stop_active: bool = False
    line_stop_active: bool = False


@dataclass
class ObstacleHintView:
    """障害物ヒントの可視化用データ。"""

    front_blocked: bool = False
    front_clearance_m: float = 0.0
    left_offset_m: float = 0.0
    right_offset_m: float = 0.0
    updated_at: Optional[datetime] = None
    override_active: bool = False


@dataclass
class ManualSignalView:
    """手動開始・信号・封鎖の最新状態。"""

    manual_start: bool = False
    manual_timestamp: Optional[datetime] = None
    sig_recog: Optional[int] = None
    sig_timestamp: Optional[datetime] = None
    road_blocked: bool = False
    road_blocked_timestamp: Optional[datetime] = None
    road_blocked_source: str = "console"


@dataclass
class TargetDistanceView:
    """ターゲット距離表示用の情報。"""

    current_distance_m: float = 0.0
    baseline_distance_m: float = 0.0
    updated_at: Optional[datetime] = None


@dataclass
class CmdVelView:
    """cmd_vel の数値表示用情報。"""

    linear_mps: float = 0.0
    angular_dps: float = 0.0
    updated_at: Optional[datetime] = None


@dataclass
class ImageBundle:
    """GUI に表示する画像群。"""

    route_map: Optional[object] = None
    obstacle_view: Optional[object] = None
    external_camera: Optional[object] = None
    obstacle_overlay: str = ""
    camera_mode: str = "drive"


@dataclass
class NodeLaunchState:
    """ノード起動カードの状態。"""

    profile_id: str = ""
    display_name: str = ""
    package: str = ""
    launch_file: str = ""
    alternate_launch_file: Optional[str] = None
    launch_toggle_label: Optional[str] = None
    use_alternate_launch: bool = False
    param_argument: Optional[str] = None
    available_params: List[str] = field(default_factory=list)
    selected_param: Optional[str] = None
    param_display_map: Dict[str, str] = field(default_factory=dict)
    selected_param_display: Optional[str] = None
    simulator_launch_file: Optional[str] = None
    simulator_enabled: bool = False
    user_arguments: List[str] = field(default_factory=list)
    override_inputs: Dict[str, str] = field(default_factory=dict)
    status: NodeLaunchStatus = NodeLaunchStatus.STOPPED
    last_action_time: Optional[datetime] = None
    process_id: Optional[int] = None
    simulator_process_id: Optional[int] = None
    error_message: str = ""


class GuiCommandType(Enum):
    """GUI から発行されるコマンドの種類。"""

    MANUAL_START = auto()
    SIG_RECOG = auto()
    ROAD_BLOCKED = auto()
    OBSTACLE_HINT_OVERRIDE = auto()
    OBSTACLE_HINT_STOP = auto()
    FRAME_IMAGE_PATH = auto()
    LAUNCH_NODE = auto()
    STOP_NODE = auto()
    LAUNCH_ALL = auto()
    STOP_ALL = auto()
    UPDATE_PARAM = auto()
    TOGGLE_SIMULATOR = auto()
    UPDATE_OVERRIDE = auto()
    SWITCH_LAUNCH_FILE = auto()


@dataclass
class GuiCommand:
    """GuiCore から ROS ノード層へ伝えるコマンド。"""

    command_type: GuiCommandType
    payload: Dict[str, object] = field(default_factory=dict)


@dataclass
class GuiSnapshot:
    """GUI スレッドに渡すスナップショット。"""

    route_state: RouteStateView
    follower_state: FollowerStateView
    obstacle_hint: ObstacleHintView
    manual_signal: ManualSignalView
    target_distance: TargetDistanceView
    cmd_vel: CmdVelView
    images: ImageBundle
    launch_states: Dict[str, NodeLaunchState]
    console_logs: Dict[str, List[str]]
    console_log_paths: Dict[str, Optional[str]] = field(default_factory=dict)


class ConsoleLogBuffer:
    """リングバッファ方式でログを保持する。"""

    def __init__(self, capacity: int = 2000) -> None:
        self._capacity = capacity
        self._lines: Deque[str] = deque(maxlen=capacity)
        self._lock = threading.Lock()

    @property
    def capacity(self) -> int:
        """最大行数を返す。"""

        return self._capacity

    def append(self, line: str) -> None:
        """ログ行を追加する。"""

        with self._lock:
            self._lines.append(line)

    def snapshot(self) -> List[str]:
        """現在のログをリストで取得する。"""

        with self._lock:
            return list(self._lines)

    def clear(self) -> None:
        """ログをクリアする。"""

        with self._lock:
            self._lines.clear()


def clone_launch_state(state: NodeLaunchState) -> NodeLaunchState:
    """NodeLaunchState の浅いコピーを作成する。"""

    return NodeLaunchState(
        profile_id=state.profile_id,
        display_name=state.display_name,
        package=state.package,
        launch_file=state.launch_file,
        alternate_launch_file=state.alternate_launch_file,
        launch_toggle_label=state.launch_toggle_label,
        use_alternate_launch=state.use_alternate_launch,
        param_argument=state.param_argument,
        available_params=list(state.available_params),
        selected_param=state.selected_param,
        param_display_map=dict(state.param_display_map),
        selected_param_display=state.selected_param_display,
        simulator_launch_file=state.simulator_launch_file,
        simulator_enabled=state.simulator_enabled,
        user_arguments=list(state.user_arguments),
        override_inputs=dict(state.override_inputs),
        status=state.status,
        last_action_time=state.last_action_time,
        process_id=state.process_id,
        simulator_process_id=state.simulator_process_id,
        error_message=state.error_message,
    )
