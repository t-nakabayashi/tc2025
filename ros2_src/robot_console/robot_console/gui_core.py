"""GUI スレッドと ROS スレッドを橋渡しする GuiCore 実装。"""

from __future__ import annotations

import json
import os
import platform
import queue
import signal
import subprocess
import threading
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Union

try:
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_prefixes,
        get_package_share_directory,
    )
except ImportError:  # pragma: no cover - 互換性維持のためにフォールバックを用意
    from ament_index_python.packages import (  # type: ignore[misc]
        PackageNotFoundError,
        get_package_share_directory,
    )

    get_package_prefixes = None  # type: ignore[assignment]
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image as ImageMsg
try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML が無い環境では JSON のみ対応
    yaml = None  # type: ignore[assignment]

from .utils import (
    CmdVelView,
    ConsoleLogBuffer,
    FollowerStateView,
    GuiCommand,
    GuiCommandType,
    GuiSnapshot,
    ImageBundle,
    ManualSignalView,
    NodeLaunchState,
    NodeLaunchStatus,
    ObstacleHintView,
    RouteStateView,
    TargetDistanceView,
    clone_launch_state,
    convert_image_message,
    create_placeholder_image,
    now,
    resize_with_letter_box,
)

CAMERA_DISPLAY_SIZE = (480, 360)


@dataclass
class NodeLaunchProfile:
    """起動対象ノードのメタデータ。"""

    profile_id: str
    display_name: str
    package: str
    launch_file: str
    alternate_launch_file: Optional[str] = None
    launch_toggle_label: Optional[str] = None
    use_alternate_launch: bool = False
    default_param: Optional[str] = None
    param_package: Optional[str] = None
    param_argument: Optional[str] = 'param_file'
    simulator_launch_file: Optional[str] = None
    user_arguments: Optional[List[str]] = None


class NodeLaunchManager:
    """subprocess ベースでノードを起動しログを収集する。"""

    def __init__(
        self,
        status_callback: Callable[[str, NodeLaunchStatus, Optional[int], Optional[str]], None],
        log_callback: Callable[[str, str], None],
        log_directory: Optional[Union[str, Path]] = None,
    ) -> None:
        self._status_callback = status_callback
        self._log_callback = log_callback
        self._lock = threading.Lock()
        self._processes: Dict[str, subprocess.Popen[str]] = {}
        self._sim_processes: Dict[str, subprocess.Popen[str]] = {}
        self._threads: List[threading.Thread] = []
        self._log_meta_lock = threading.Lock()
        self._log_paths: Dict[str, Path] = {}
        self._log_locks: Dict[str, threading.Lock] = {}
        self._log_stream_counters: Dict[str, int] = {}
        self._latest_log_paths: Dict[str, Path] = {}
        self._base_log_directory: Optional[Path] = None
        self._run_log_directory: Optional[Path] = None
        if log_directory:
            base_dir = Path(log_directory).expanduser()
            self._base_log_directory = base_dir

    def launch(
        self,
        profile: NodeLaunchProfile,
        param_path: Optional[str],
        simulator_enabled: bool,
        overrides: Optional[Dict[str, str]] = None,
        launch_file_override: Optional[str] = None,
    ) -> None:
        """指定ノードを起動する。"""

        with self._lock:
            self._cleanup_finished_process_locked()
            already_running = profile.profile_id in self._processes
        if already_running:
            self.stop(profile.profile_id)
        with self._lock:
            self._cleanup_finished_process_locked()
            if profile.profile_id in self._processes:
                raise RuntimeError(f"{profile.profile_id} の停止完了を待っています")
            self._status_callback(profile.profile_id, NodeLaunchStatus.STARTING, None, None)

            launch_file = launch_file_override or profile.launch_file
            args = [
                "ros2",
                "launch",
                profile.package,
                launch_file,
            ]
            if param_path and profile.param_argument:
                args.append(f"{profile.param_argument}:={param_path}")
            if overrides:
                for key, value in overrides.items():
                    args.append(f"{key}:={value}")

            process = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                **self._build_popen_options(),
            )
            self._processes[profile.profile_id] = process
            self._status_callback(profile.profile_id, NodeLaunchStatus.RUNNING, process.pid, None)
            self._start_reader_threads(profile.profile_id, process)
            self._start_monitor_thread(
                dict_key=profile.profile_id,
                status_id=profile.profile_id,
                process=process,
                is_simulator=False,
            )

        if simulator_enabled and profile.simulator_launch_file:
            sim_args = [
                "ros2",
                "launch",
                profile.package,
                profile.simulator_launch_file,
            ]
            sim_process = subprocess.Popen(
                sim_args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                **self._build_popen_options(),
            )
            with self._lock:
                self._sim_processes[profile.profile_id] = sim_process
            self._start_monitor_thread(
                dict_key=profile.profile_id,
                status_id=f"{profile.profile_id}:sim",
                process=sim_process,
                is_simulator=True,
            )
            self._start_reader_threads(f"{profile.profile_id}:sim", sim_process)
            self._status_callback(
                f"{profile.profile_id}:sim",
                NodeLaunchStatus.RUNNING,
                sim_process.pid,
                None,
            )

    def stop(self, profile_id: str) -> None:
        """ノードを停止する。"""

        with self._lock:
            process = self._processes.get(profile_id)
            sim_process = self._sim_processes.get(profile_id)

        if process is not None:
            self._status_callback(profile_id, NodeLaunchStatus.STOPPING, process.pid, None)
            self._terminate_process(process)
        else:
            self._status_callback(profile_id, NodeLaunchStatus.STOPPED, None, None)

        if sim_process is not None:
            self._status_callback(
                f"{profile_id}:sim",
                NodeLaunchStatus.STOPPING,
                sim_process.pid,
                None,
            )
            self._terminate_process(sim_process)
        else:
            self._status_callback(f"{profile_id}:sim", NodeLaunchStatus.STOPPED, None, None)
        with self._lock:
            self._cleanup_finished_process_locked()

    def is_running(self, profile_id: str) -> bool:
        """指定ノードが稼働中かどうかを判定する。"""

        with self._lock:
            self._cleanup_finished_process_locked()
            if profile_id in self._processes:
                return True
            return profile_id in self._sim_processes

    def _terminate_process(self, process: subprocess.Popen[str]) -> None:
        """プロセスに対し SIGINT → SIGTERM → SIGKILL の順で停止要求を行う。"""

        if process.poll() is not None:
            return
        self._send_signal(process, signal.SIGINT)
        if self._wait_process(process, timeout=5.0):
            return
        self._send_signal(process, signal.SIGTERM)
        if self._wait_process(process, timeout=3.0):
            return
        self._send_signal(process, signal.SIGKILL)
        self._wait_process(process, timeout=1.0, suppress_timeout=True)

    def _build_popen_options(self) -> Dict[str, object]:
        """サブプロセス起動時のオプションを構築する。"""

        options: Dict[str, object] = {}
        if os.name == 'nt':
            options['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            options['preexec_fn'] = os.setsid
        return options

    def _send_signal(self, process: subprocess.Popen[str], sig: signal.Signals) -> None:
        """プロセスグループにシグナルを送信し、失敗時は個別プロセスに送る。"""

        if process.poll() is not None:
            return
        if os.name != 'nt':
            try:
                os.killpg(os.getpgid(process.pid), sig)
                return
            except (ProcessLookupError, PermissionError, AttributeError, OSError):
                pass
        try:
            process.send_signal(sig)
        except (ProcessLookupError, AttributeError, ValueError):
            try:
                process.terminate()
            except (ProcessLookupError, AttributeError, ValueError):
                return

    @staticmethod
    def _wait_process(
        process: subprocess.Popen[str], timeout: float, suppress_timeout: bool = False
    ) -> bool:
        """指定時間でプロセス終了を待ち、終了したら True を返す。"""

        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            if suppress_timeout:
                return False
            return False

    def _start_reader_threads(self, profile_id: str, process: subprocess.Popen[str]) -> None:
        """stdout/stderr を読み取るスレッドを起動する。"""

        streams = []
        if process.stdout is not None:
            streams.append((process.stdout, 'INFO'))
        if process.stderr is not None:
            streams.append((process.stderr, 'ERR'))

        if streams:
            self._prepare_log_file(profile_id, process.pid, len(streams))

        def _reader(stream, prefix: str) -> None:
            try:
                for line in iter(stream.readline, ''):
                    formatted = f"[{prefix}] {line.rstrip()}\n"
                    self._log_callback(profile_id, formatted)
                    self._append_log_line(profile_id, formatted)
            finally:
                stream.close()
                self._notify_stream_closed(profile_id)

        for stream, prefix in streams:
            thread = threading.Thread(
                target=_reader,
                args=(stream, prefix),
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)

    def _cleanup_finished_process_locked(self) -> None:
        """監視用辞書から終了済みプロセスを除去する。"""

        for key, process in list(self._processes.items()):
            if process.poll() is not None:
                self._processes.pop(key, None)
        for key, process in list(self._sim_processes.items()):
            if process.poll() is not None:
                self._sim_processes.pop(key, None)

    def _prepare_log_file(self, profile_id: str, pid: int, stream_count: int) -> None:
        """ログファイルの作成準備を行う。"""

        if self._base_log_directory is None:
            return
        run_dir = self._ensure_run_log_directory()
        if run_dir is None:
            return
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        sanitized = profile_id.replace(':', '_')
        file_name = f"{sanitized}-{pid}-{timestamp}.log"
        path = run_dir / file_name
        with self._log_meta_lock:
            self._log_paths[profile_id] = path
            self._log_locks[profile_id] = threading.Lock()
            self._log_stream_counters[profile_id] = stream_count
            self._latest_log_paths[profile_id] = path

    def _append_log_line(self, profile_id: str, line: str) -> None:
        """ログ行をファイルへ追記する。"""

        with self._log_meta_lock:
            path = self._log_paths.get(profile_id)
            lock = self._log_locks.get(profile_id)
        if path is None or lock is None:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        with lock:
            with path.open('a', encoding='utf-8') as handle:
                handle.write(line)

    def _notify_stream_closed(self, profile_id: str) -> None:
        """ストリーム終了時にファイル管理情報を更新する。"""

        with self._log_meta_lock:
            counter = self._log_stream_counters.get(profile_id)
            if counter is None:
                return
            if counter <= 1:
                self._log_stream_counters.pop(profile_id, None)
                self._log_locks.pop(profile_id, None)
                self._log_paths.pop(profile_id, None)
            else:
                self._log_stream_counters[profile_id] = counter - 1

    def _ensure_run_log_directory(self) -> Optional[Path]:
        """ROS2 標準に倣ったランログディレクトリを生成する。"""

        if self._base_log_directory is None:
            return None
        if self._run_log_directory is not None:
            return self._run_log_directory
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        hostname = platform.node() or 'unknown_host'
        run_dir = self._base_log_directory / f"{timestamp}_{hostname}"
        run_dir.mkdir(parents=True, exist_ok=True)
        self._run_log_directory = run_dir
        return run_dir

    def _start_monitor_thread(
        self,
        dict_key: str,
        status_id: str,
        process: subprocess.Popen[str],
        is_simulator: bool,
    ) -> None:
        """終了状態を監視し辞書の整合性を保つスレッドを起動する。"""

        def _monitor() -> None:
            return_code = process.wait()
            with self._lock:
                target = self._sim_processes if is_simulator else self._processes
                target.pop(dict_key, None)
            status = NodeLaunchStatus.STOPPED
            message = None
            if return_code != 0:
                status = NodeLaunchStatus.ERROR
                message = f"プロセスが異常終了しました (return code={return_code})"
            self._status_callback(status_id, status, None, message)

        thread = threading.Thread(target=_monitor, daemon=True)
        thread.start()
        self._threads.append(thread)

    def get_latest_log_path(self, profile_id: str) -> Optional[Path]:
        """指定ノードの直近のログファイルパスを返す。"""

        with self._log_meta_lock:
            path = self._latest_log_paths.get(profile_id)
        return path


class GuiCore:
    """GUI 更新と ROS I/O を仲介する。"""

    def __init__(
        self,
        launch_profiles: Optional[List[NodeLaunchProfile]] = None,
        log_directory: Optional[Union[str, Path]] = None,
    ) -> None:
        self._route_state = RouteStateView()
        self._follower_state = FollowerStateView()
        self._obstacle_hint = ObstacleHintView()
        self._manual_signal = ManualSignalView()
        self._target_distance = TargetDistanceView()
        self._cmd_vel = CmdVelView()
        self._images = ImageBundle()
        self._placeholders = {
            'route': create_placeholder_image((640, 360), 'No Image'),
            'obstacle': create_placeholder_image((400, 400), 'No Image'),
            'camera_drive': create_placeholder_image(CAMERA_DISPLAY_SIZE, 'No Image'),
            'camera_signal': create_placeholder_image(CAMERA_DISPLAY_SIZE, 'No Image'),
        }
        self._images.route_map = self._placeholders['route']
        self._images.obstacle_view = self._placeholders['obstacle']
        self._images.external_camera = self._placeholders['camera_drive']
        self._launch_states: Dict[str, NodeLaunchState] = {}
        self._log_buffers: Dict[str, ConsoleLogBuffer] = {}
        self._lock = threading.Lock()
        self._command_queue: queue.Queue[GuiCommand] = queue.Queue()
        self._current_target: Optional[PoseStamped] = None
        self._current_pose: Optional[PoseWithCovarianceStamped] = None
        self._drive_camera_frame = self._placeholders['camera_drive']
        self._signal_camera_frame = self._placeholders['camera_signal']
        self._camera_signal_forced = False
        self._route_signal_stop_flags: List[bool] = []
        self._route_line_stop_flags: List[bool] = []
        self._launch_manager = NodeLaunchManager(
            self._on_launch_status,
            self._on_log_received,
            log_directory=log_directory,
        )
        self._profiles = launch_profiles or default_launch_profiles()
        self._initialize_launch_states()
        self._load_additional_params()

    def _initialize_launch_states(self) -> None:
        for profile in self._profiles:
            state = NodeLaunchState(
                profile_id=profile.profile_id,
                display_name=profile.display_name,
                package=profile.package,
                launch_file=profile.launch_file,
                alternate_launch_file=profile.alternate_launch_file,
                launch_toggle_label=profile.launch_toggle_label,
                use_alternate_launch=profile.use_alternate_launch,
                param_argument=profile.param_argument,
                simulator_launch_file=profile.simulator_launch_file,
                selected_param=profile.default_param,
            )
            if profile.user_arguments:
                state.user_arguments = list(profile.user_arguments)
                for key in state.user_arguments:
                    state.override_inputs[key] = ''
            params = self._discover_params(profile)
            display_map = self._build_param_display_map(params)
            state.param_display_map = display_map
            state.available_params = sorted(display_map.keys())
            if state.selected_param:
                display = self._resolve_param_display(display_map, state.selected_param)
                if display is None:
                    display = self._register_param_option(state, state.selected_param)
                state.selected_param_display = display
            elif state.available_params:
                first_display = state.available_params[0]
                state.selected_param_display = first_display
                state.selected_param = state.param_display_map.get(first_display)
            self._launch_states[profile.profile_id] = state
            self._log_buffers[profile.profile_id] = ConsoleLogBuffer()

    def _load_additional_params(self) -> None:
        config_root = Path(__file__).resolve().parent.parent / 'config' / 'node_launch_profiles.yaml'
        if not config_root.exists():
            return
        raw_text = config_root.read_text(encoding='utf-8')
        content = None
        if yaml is not None:
            try:
                content = yaml.safe_load(raw_text)
            except yaml.YAMLError:  # type: ignore[attr-defined]
                content = None
        if content is None:
            try:
                content = json.loads(raw_text)
            except json.JSONDecodeError:
                return
        if not isinstance(content, dict):
            return
        for profile_id, profile_conf in content.items():
            if profile_id not in self._launch_states:
                continue
            additional = profile_conf.get('additional_params', [])
            state = self._launch_states[profile_id]
            for item in additional:
                if not isinstance(item, str):
                    continue
                self._register_param_option(state, item)

    def _discover_params(self, profile: NodeLaunchProfile) -> List[str]:
        params: List[str] = []
        search_dirs: List[Path] = []
        package_candidates = []
        if profile.param_package:
            package_candidates.append(profile.param_package)
        if profile.package not in package_candidates:
            package_candidates.append(profile.package)

        for package_name in package_candidates:
            share_dirs: List[Path] = []
            if get_package_prefixes:
                try:
                    for prefix in get_package_prefixes(package_name):
                        share_dirs.append(Path(prefix).expanduser() / 'share' / package_name)
                except PackageNotFoundError:
                    pass
            if not share_dirs:
                try:
                    share_dirs.append(Path(get_package_share_directory(package_name)))
                except PackageNotFoundError:
                    continue

            for share_dir in {path.resolve() for path in share_dirs}:
                for sub in ('params', 'config'):
                    candidate = share_dir / sub
                    if candidate.exists():
                        search_dirs.append(candidate)
        extra_roots = {
            Path(__file__).resolve().parent.parent
            / 'config'
            / 'node_params'
            / profile.package
        }
        if profile.param_package and profile.param_package != profile.package:
            extra_roots.add(
                Path(__file__).resolve().parent.parent
                / 'config'
                / 'node_params'
                / profile.param_package
            )
        for extra_root in extra_roots:
            if extra_root.exists():
                search_dirs.append(extra_root)
        for directory in search_dirs:
            for path in directory.glob('**/*.yaml'):
                params.append(str(path))
        params = sorted(set(params))
        return params

    @staticmethod
    def _build_param_display_map(params: List[str]) -> Dict[str, str]:
        grouped: Dict[str, List[str]] = {}
        for path in params:
            name = Path(path).name or path
            grouped.setdefault(name, []).append(path)
        display_map: Dict[str, str] = {}
        for name, entries in grouped.items():
            if len(entries) == 1:
                display_map[name] = entries[0]
            else:
                for index, entry in enumerate(sorted(entries), start=1):
                    display_map[f"{name} ({index})"] = entry
        return display_map

    @staticmethod
    def _resolve_param_display(mapping: Dict[str, str], path: Optional[str]) -> Optional[str]:
        if path is None:
            return None
        for display, actual in mapping.items():
            if actual == path:
                return display
        return None

    def _register_param_option(self, state: NodeLaunchState, param_path: str) -> str:
        display = self._resolve_param_display(state.param_display_map, param_path)
        if display:
            return display
        base = Path(param_path).name or param_path
        candidate = base
        suffix = 2
        while candidate in state.param_display_map:
            candidate = f"{base} ({suffix})"
            suffix += 1
        state.param_display_map[candidate] = param_path
        state.available_params = sorted(state.param_display_map.keys())
        return candidate

    def _build_launch_overrides(
        self, profile: NodeLaunchProfile, state: NodeLaunchState
    ) -> Dict[str, str]:
        overrides: Dict[str, str] = {}
        if profile.profile_id == 'route_manager':
            start = state.override_inputs.get('start_label', '').strip()
            if start:
                overrides['start_label'] = start
            goal = state.override_inputs.get('goal_label', '').strip()
            if goal:
                overrides['goal_label'] = goal
            checkpoint_raw = state.override_inputs.get('checkpoint_labels', '')
            labels = self._parse_label_list(checkpoint_raw)
            if labels:
                overrides['checkpoint_labels'] = ','.join(labels)
        return overrides

    @staticmethod
    def _resolve_launch_file(profile: NodeLaunchProfile, state: NodeLaunchState) -> str:
        if state.use_alternate_launch and state.alternate_launch_file:
            return state.alternate_launch_file
        return profile.launch_file

    @staticmethod
    def _parse_label_list(raw_value: str) -> List[str]:
        text = raw_value.replace('\n', ',').strip()
        if not text:
            return []
        if text.startswith('[') and text.endswith(']'):
            text = text[1:-1]
        labels: List[str] = []
        for chunk in text.split(','):
            label = chunk.strip().strip("\"'")
            if label:
                labels.append(label)
        return labels

    # ---------- 状態更新メソッド ----------
    def update_route_state(self, msg) -> None:
        with self._lock:
            self._route_state.route_version = msg.route_version
            self._route_state.total_waypoints = msg.total_waypoints
            self._route_state.current_index = msg.current_index
            self._route_state.progress = (
                msg.current_index / msg.total_waypoints if msg.total_waypoints else 0.0
            )
            self._route_state.route_status = self._translate_route_status(msg.status)
            message = getattr(msg, 'message', '')
            if message:
                self._route_state.last_replan_reason = message
                self._route_state.last_replan_time = now()
            self._route_state.current_label = getattr(msg, 'current_label', self._route_state.current_label)

    def update_manager_status(self, msg) -> None:
        with self._lock:
            self._route_state.manager_state = msg.state
            decision = str(getattr(msg, 'decision', '') or '').strip()
            if decision.lower() == 'none':
                decision = ''
            cause = str(getattr(msg, 'last_cause', '') or '').strip()
            self._route_state.manager_decision = decision
            self._route_state.manager_cause = cause
            self._route_state.manager_updated_at = now()

    def update_route(self, msg) -> None:
        image = convert_image_message(msg.route_image)
        signal_stop_flags = [
            bool(getattr(wp, 'signal_stop', False)) for wp in getattr(msg, 'waypoints', [])
        ]
        line_stop_flags = [
            bool(getattr(wp, 'line_stop', False)) for wp in getattr(msg, 'waypoints', [])
        ]
        with self._lock:
            if image is not None and (image.width > 10 and image.height > 10):
                self._images.route_map = resize_with_letter_box(image, (640, 360))
            else:
                self._images.route_map = self._placeholders['route']
            self._route_state.route_version = msg.version
            self._route_signal_stop_flags = signal_stop_flags
            self._route_line_stop_flags = line_stop_flags
            self._route_state.current_label = getattr(msg, 'current_label', self._route_state.current_label)

    def update_follower_state(self, msg) -> None:
        with self._lock:
            self._follower_state.state = msg.state
            self._follower_state.active_waypoint_index = msg.active_waypoint_index
            self._follower_state.active_waypoint_label = msg.active_waypoint_label
            self._follower_state.segment_length_m = getattr(msg, 'segment_length_m', 0.0)
            self._follower_state.front_blocked = getattr(msg, 'front_blocked', False)
            self._follower_state.front_clearance_m = getattr(
                msg, 'front_clearance_m', float('inf')
            )
            self._follower_state.left_offset_m = getattr(msg, 'left_offset_m', 0.0)
            self._follower_state.right_offset_m = getattr(msg, 'right_offset_m', 0.0)
            self._follower_state.stagnation_reason = msg.last_stagnation_reason
            self._follower_state.retry_count = msg.avoidance_attempt_count
            fallback_distance = float(getattr(msg, 'active_target_distance_m', 0.0))
            if self._current_target is None or self._current_pose is None:
                self._target_distance.current_distance_m = fallback_distance
                self._target_distance.updated_at = now()
            segment_length = float(getattr(msg, 'segment_length_m', 0.0))
            if segment_length > 0.0:
                self._target_distance.baseline_distance_m = segment_length
            signal_stop_active = bool(getattr(msg, 'signal_stop_active', False))
            if not signal_stop_active and msg.state == 'WAITING_STOP':
                index = msg.active_waypoint_index
                if 0 <= index < len(self._route_signal_stop_flags):
                    signal_stop_active = self._route_signal_stop_flags[index]
            self._follower_state.signal_stop_active = signal_stop_active
            line_stop_active = False
            if msg.state == 'WAITING_STOP':
                index = msg.active_waypoint_index
                if 0 <= index < len(self._route_line_stop_flags):
                    line_stop_active = self._route_line_stop_flags[index]
            self._follower_state.line_stop_active = line_stop_active
            if msg.state == 'WAITING_STOP' and signal_stop_active:
                self._camera_signal_forced = True
            if msg.state == 'FINISHED':
                current_status = (self._route_state.route_status or '').lower()
                if current_status not in {'completed', 'finished'}:
                    self._route_state.route_status = 'completed'
                manager_state = (self._route_state.manager_state or '').lower()
                if manager_state in {'running', 'updating_route', 'holding'}:
                    self._route_state.manager_state = 'finished'
            self._update_camera_frame_locked()

    def update_obstacle_hint(self, msg) -> None:
        with self._lock:
            self._obstacle_hint.front_blocked = msg.front_blocked
            self._obstacle_hint.front_clearance_m = msg.front_clearance_m
            self._obstacle_hint.left_offset_m = msg.left_offset_m
            self._obstacle_hint.right_offset_m = msg.right_offset_m
            self._obstacle_hint.updated_at = now()
            overlay = (
                f"遮蔽:{'あり' if msg.front_blocked else 'なし'} "
                f"前方距離:{msg.front_clearance_m:.1f}m\n"
                f"左:{msg.left_offset_m:.1f}m 右:{msg.right_offset_m:.1f}m"
            )
            self._images.obstacle_overlay = overlay

    def update_obstacle_image(self, msg: ImageMsg) -> None:
        image = convert_image_message(msg)
        with self._lock:
            if image is None:
                self._images.obstacle_view = self._placeholders['obstacle']
            else:
                self._images.obstacle_view = resize_with_letter_box(image, (400, 400))

    def update_manual_start(self, msg) -> None:
        with self._lock:
            self._manual_signal.manual_start = bool(msg.data)
            self._manual_signal.manual_timestamp = now()

    def update_sig_recog(self, msg) -> None:
        with self._lock:
            self._manual_signal.sig_recog = msg.data
            self._manual_signal.sig_timestamp = now()
            if msg.data == 1:
                self._camera_signal_forced = False
            self._update_camera_frame_locked()

    def update_road_blocked(self, msg, source: str = 'external') -> None:
        with self._lock:
            self._manual_signal.road_blocked = bool(msg.data)
            self._manual_signal.road_blocked_timestamp = now()
            self._manual_signal.road_blocked_source = source

    def update_cmd_vel(self, msg) -> None:
        with self._lock:
            self._cmd_vel.linear_mps = msg.linear.x
            self._cmd_vel.angular_dps = msg.angular.z * 180.0 / 3.141592653589793
            self._cmd_vel.updated_at = now()

    def _update_camera_frame_locked(self) -> None:
        mode = 'drive'
        if self._camera_signal_forced or self._manual_signal.sig_recog == 2:
            mode = 'signal'
        if mode == 'signal':
            frame = self._signal_camera_frame or self._placeholders['camera_signal']
        else:
            frame = self._drive_camera_frame or self._placeholders['camera_drive']
        self._images.external_camera = frame
        self._images.camera_mode = mode

    def update_active_target(self, msg: PoseStamped) -> None:
        with self._lock:
            previous_target = self._current_target
            self._current_target = msg
            if previous_target is not None:
                baseline = max(self._compute_distance(msg, previous_target), 0.0)
                self._target_distance.baseline_distance_m = baseline
            else:
                self._target_distance.baseline_distance_m = 0.0
            if self._current_pose is not None:
                self._target_distance.current_distance_m = self._compute_distance(msg, self._current_pose)
                self._target_distance.updated_at = now()

    def update_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._current_pose = msg
            if self._current_target is not None:
                self._target_distance.current_distance_m = self._compute_distance(
                    self._current_target, msg
                )
                self._target_distance.updated_at = now()

    def update_camera_image(self, msg: ImageMsg, mode: str) -> None:
        image = convert_image_message(msg)
        size = CAMERA_DISPLAY_SIZE
        resized = (
            resize_with_letter_box(image, size)
            if image is not None
            else self._placeholders['camera_drive' if mode == 'drive' else 'camera_signal']
        )
        with self._lock:
            if mode == 'drive':
                self._drive_camera_frame = resized
            else:
                self._signal_camera_frame = resized
            self._update_camera_frame_locked()

    def update_sensor_viewer(self, msg: ImageMsg) -> None:
        self.update_obstacle_image(msg)

    def append_console_log(self, profile_id: str, line: str) -> None:
        buffer = self._log_buffers.get(profile_id)
        if buffer:
            buffer.append(line)

    # ---------- コマンド処理 ----------
    def request_manual_start(self, value: bool) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.MANUAL_START, {'value': value}))

    def request_sig_recog(self, go: bool) -> None:
        value = 1 if go else 2
        self._command_queue.put(GuiCommand(GuiCommandType.SIG_RECOG, {'value': value}))

    def request_road_blocked(self, value: bool) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.ROAD_BLOCKED, {'value': value}))

    def request_frame_image_path(self, path: str) -> None:
        self._command_queue.put(
            GuiCommand(GuiCommandType.FRAME_IMAGE_PATH, {'value': path})
        )

    def start_obstacle_override(self, front_blocked: bool, clearance: float, left: float, right: float) -> None:
        payload = {
            'front_blocked': front_blocked,
            'front_clearance_m': clearance,
            'left_offset_m': left,
            'right_offset_m': right,
        }
        with self._lock:
            self._obstacle_hint.override_active = True
        self._command_queue.put(GuiCommand(GuiCommandType.OBSTACLE_HINT_OVERRIDE, payload))

    def stop_obstacle_override(self) -> None:
        with self._lock:
            self._obstacle_hint.override_active = False
        self._command_queue.put(GuiCommand(GuiCommandType.OBSTACLE_HINT_STOP, {}))

    def request_launch(self, profile_id: str) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.LAUNCH_NODE, {'profile_id': profile_id}))

    def request_stop(self, profile_id: str) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.STOP_NODE, {'profile_id': profile_id}))

    def request_launch_all(self) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.LAUNCH_ALL, {}))

    def request_stop_all(self) -> None:
        self._command_queue.put(GuiCommand(GuiCommandType.STOP_ALL, {}))

    def update_selected_param(self, profile_id: str, param_display: Optional[str]) -> None:
        param_path: Optional[str] = None
        if param_display:
            with self._lock:
                state = self._launch_states.get(profile_id)
                if state:
                    param_path = state.param_display_map.get(param_display)
        self._command_queue.put(
            GuiCommand(
                GuiCommandType.UPDATE_PARAM,
                {'profile_id': profile_id, 'param_path': param_path},
            )
        )

    def update_simulator_enabled(self, profile_id: str, enabled: bool) -> None:
        self._command_queue.put(
            GuiCommand(
                GuiCommandType.TOGGLE_SIMULATOR,
                {'profile_id': profile_id, 'enabled': enabled},
            )
        )

    def update_launch_file_selection(self, profile_id: str, use_alternate: bool) -> None:
        self._command_queue.put(
            GuiCommand(
                GuiCommandType.SWITCH_LAUNCH_FILE,
                {'profile_id': profile_id, 'use_alternate': use_alternate},
            )
        )

    def update_launch_override(self, profile_id: str, key: str, value: str) -> None:
        self._command_queue.put(
            GuiCommand(
                GuiCommandType.UPDATE_OVERRIDE,
                {'profile_id': profile_id, 'key': key, 'value': value},
            )
        )

    def get_next_command(self, timeout: Optional[float] = None) -> Optional[GuiCommand]:
        try:
            return self._command_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    # ---------- スナップショット ----------
    def snapshot(self) -> GuiSnapshot:
        with self._lock:
            launch_states = {pid: clone_launch_state(state) for pid, state in self._launch_states.items()}
            logs = {pid: buf.snapshot() for pid, buf in self._log_buffers.items()}
            log_paths = {}
            for pid in launch_states.keys():
                latest = self._launch_manager.get_latest_log_path(pid)
                log_paths[pid] = str(latest) if latest is not None else None
            images = ImageBundle(
                route_map=self._images.route_map,
                obstacle_view=self._images.obstacle_view,
                external_camera=self._images.external_camera,
                obstacle_overlay=self._images.obstacle_overlay,
                camera_mode=self._images.camera_mode,
            )
            snapshot = GuiSnapshot(
                route_state=replace(self._route_state),
                follower_state=replace(self._follower_state),
                obstacle_hint=replace(self._obstacle_hint),
                manual_signal=replace(self._manual_signal),
                target_distance=replace(self._target_distance),
                cmd_vel=replace(self._cmd_vel),
                images=images,
                launch_states=launch_states,
                console_logs=logs,
                console_log_paths=log_paths,
            )
        return snapshot

    # ---------- launch コールバック ----------
    def _on_launch_status(
        self,
        profile_id: str,
        status: NodeLaunchStatus,
        process_id: Optional[int],
        error: Optional[str],
    ) -> None:
        base_id, _, suffix = profile_id.partition(':')
        state = self._launch_states.get(base_id)
        if not state:
            return
        is_simulator = suffix == 'sim'
        with self._lock:
            state.last_action_time = now()
            if is_simulator:
                state.simulator_process_id = process_id
                if status == NodeLaunchStatus.STOPPED:
                    state.simulator_process_id = None
                if status == NodeLaunchStatus.ERROR and error:
                    state.error_message = error
            else:
                state.status = status
                state.process_id = process_id
                if status == NodeLaunchStatus.STOPPED:
                    state.process_id = None
                if error:
                    state.error_message = error
                elif status == NodeLaunchStatus.RUNNING:
                    state.error_message = ''

    def _on_log_received(self, profile_id: str, line: str) -> None:
        base_id, _, suffix = profile_id.partition(':')
        prefix = '[SIM] ' if suffix == 'sim' else ''
        self.append_console_log(base_id, f"{prefix}{line}")

    # ---------- コマンド適用（ROS ノード側で使用） ----------
    def handle_command(self, command: GuiCommand) -> None:
        if command.command_type == GuiCommandType.UPDATE_OVERRIDE:
            self._apply_override_update(command.payload)
            return
        if command.command_type == GuiCommandType.UPDATE_PARAM:
            self._apply_param_update(command.payload)
            return
        if command.command_type == GuiCommandType.TOGGLE_SIMULATOR:
            self._apply_simulator_toggle(command.payload)
            return
        if command.command_type == GuiCommandType.SWITCH_LAUNCH_FILE:
            self._apply_launch_file_toggle(command.payload)
            return
        if command.command_type == GuiCommandType.LAUNCH_NODE:
            self._handle_launch_request(command.payload['profile_id'])
        elif command.command_type == GuiCommandType.STOP_NODE:
            self._handle_stop_request(command.payload['profile_id'])
        elif command.command_type == GuiCommandType.LAUNCH_ALL:
            for profile in self._profiles:
                self._handle_launch_request(profile.profile_id)
        elif command.command_type == GuiCommandType.STOP_ALL:
            for profile in reversed(self._profiles):
                self._handle_stop_request(profile.profile_id)

    def _handle_launch_request(self, profile_id: str) -> None:
        profile = next((p for p in self._profiles if p.profile_id == profile_id), None)
        if not profile:
            return
        with self._lock:
            state = self._launch_states.get(profile_id)
            if not state:
                return
            param_path = state.selected_param
            simulator_enabled = state.simulator_enabled
            overrides = self._build_launch_overrides(profile, state)
            launch_file = self._resolve_launch_file(profile, state)
        if self._launch_manager.is_running(profile_id):
            try:
                self._launch_manager.stop(profile_id)
            except Exception as exc:  # pylint: disable=broad-except
                self._on_launch_status(profile_id, NodeLaunchStatus.ERROR, None, str(exc))
                return
        try:
            self._launch_manager.launch(
                profile,
                param_path,
                simulator_enabled,
                overrides if overrides else None,
                launch_file_override=launch_file,
            )
        except Exception as exc:  # pylint: disable=broad-except
            self._on_launch_status(profile_id, NodeLaunchStatus.ERROR, None, str(exc))

    def _handle_stop_request(self, profile_id: str) -> None:
        try:
            self._launch_manager.stop(profile_id)
        except Exception as exc:  # pylint: disable=broad-except
            self._on_launch_status(profile_id, NodeLaunchStatus.ERROR, None, str(exc))

    def _apply_param_update(self, payload: Dict[str, object]) -> None:
        profile_id = payload.get('profile_id')
        param_path = payload.get('param_path')
        if not isinstance(profile_id, str):
            return
        with self._lock:
            state = self._launch_states.get(profile_id)
            if not state:
                return
            if param_path:
                path_str = str(param_path)
                state.selected_param = path_str
                display = self._resolve_param_display(state.param_display_map, path_str)
                if display is None:
                    display = self._register_param_option(state, path_str)
                state.selected_param_display = display
            else:
                state.selected_param = None
                state.selected_param_display = None

    def _apply_override_update(self, payload: Dict[str, object]) -> None:
        profile_id = payload.get('profile_id')
        key = payload.get('key')
        value = payload.get('value')
        if not isinstance(profile_id, str) or not isinstance(key, str):
            return
        if not isinstance(value, str):
            value = ''
        with self._lock:
            state = self._launch_states.get(profile_id)
            if not state:
                return
            state.override_inputs[key] = value

    def _apply_simulator_toggle(self, payload: Dict[str, object]) -> None:
        profile_id = payload.get('profile_id')
        enabled = payload.get('enabled')
        if not isinstance(profile_id, str) or not isinstance(enabled, bool):
            return
        with self._lock:
            state = self._launch_states.get(profile_id)
            if not state:
                return
            state.simulator_enabled = enabled

    def _apply_launch_file_toggle(self, payload: Dict[str, object]) -> None:
        profile_id = payload.get('profile_id')
        use_alternate = payload.get('use_alternate')
        if not isinstance(profile_id, str) or not isinstance(use_alternate, bool):
            return
        with self._lock:
            state = self._launch_states.get(profile_id)
            if not state or not state.alternate_launch_file:
                return
            state.use_alternate_launch = use_alternate

    @staticmethod
    def _compute_distance(
        target: PoseStamped,
        pose: Union[PoseStamped, PoseWithCovarianceStamped],
    ) -> float:
        def _resolve_position(
            obj: Union[PoseStamped, PoseWithCovarianceStamped]
        ) -> object:
            pose_field = obj.pose
            inner_pose = getattr(pose_field, 'pose', None)
            if inner_pose is not None and hasattr(inner_pose, 'position'):
                return inner_pose.position
            if hasattr(pose_field, 'position'):
                return pose_field.position
            raise AttributeError('位置情報を取得できませんでした')

        target_position = target.pose.position
        pose_position = _resolve_position(pose)
        dx = target_position.x - pose_position.x
        dy = target_position.y - pose_position.y
        dz = target_position.z - pose_position.z
        return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

    @staticmethod
    def _translate_route_status(status: int) -> str:
        mapping = {
            0: 'unknown',
            1: 'idle',
            2: 'running',
            3: 'updating_route',
            4: 'holding',
            5: 'completed',
            6: 'error',
        }
        return mapping.get(status, 'unknown')


def default_launch_profiles() -> List[NodeLaunchProfile]:
    """既定のノード起動プロファイルを生成する。"""

    return [
        NodeLaunchProfile(
            profile_id='route_planner',
            display_name='Route Planner',
            package='route_planner',
            launch_file='route_planner.launch.py',
        ),
        NodeLaunchProfile(
            profile_id='route_manager',
            display_name='Route Manager',
            package='route_manager',
            launch_file='route_manager.launch.py',
            user_arguments=['start_label', 'goal_label', 'checkpoint_labels'],
        ),
        NodeLaunchProfile(
            profile_id='route_follower',
            display_name='Route Follower',
            package='route_follower',
            launch_file='route_follower.launch.py',
        ),
        NodeLaunchProfile(
            profile_id='robot_navigator',
            display_name='Robot Navigator',
            package='robot_navigator',
            launch_file='robot_navigator.launch.py',
            simulator_launch_file='robot_simulator.launch.py',
        ),
        NodeLaunchProfile(
            profile_id='obstacle_monitor',
            display_name='Obstacle Monitor',
            package='obstacle_monitor',
            launch_file='obstacle_monitor.launch.py',
            param_argument=None,
            simulator_launch_file='laser_scan_simulator.launch.py',
        ),
        NodeLaunchProfile(
            profile_id='yolo_detector',
            display_name='Yolo Detector',
            package='yolo_detector',
            launch_file='yolo_ncnn_with_road_blockage.launch.py',
            alternate_launch_file='yolo_with_road_blockage.launch.py',
            launch_toggle_label='yolo_node モード',
            use_alternate_launch=False,
            param_package='yolo_detector',
            param_argument='route_param_file',
            simulator_launch_file='camera_simulator_node.launch.py',
        ),
    ]
