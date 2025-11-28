"""robot_console ノードの ROS2 実装。"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from route_msgs.msg import (
    FollowerState,
    ManagerStatus,
    ObstacleAvoidanceHint,
    Route,
    RouteState,
)
from sensor_msgs.msg import Image as ImageMsg

from .gui_core import GuiCore
from .ui_main import UiMain
from .utils import GuiCommandType


class RobotConsoleNode(Node):
    """GuiCore と tkinter UI を連携させる ROS ノード。"""

    def __init__(self, gui_core: Optional[GuiCore] = None) -> None:
        super().__init__('robot_console')
        log_dir_param = self.declare_parameter('console_log_directory', '').value
        resolved_log_dir: Optional[Path] = None
        if isinstance(log_dir_param, str) and log_dir_param:
            candidate = Path(log_dir_param).expanduser()
            try:
                candidate.mkdir(parents=True, exist_ok=True)
            except OSError as exc:  # pragma: no cover - 例外時は警告のみ
                self.get_logger().warn(
                    f'コンソールログ出力先 {candidate} を作成できません: {exc}'
                )
            else:
                resolved_log_dir = candidate
                self.get_logger().info(
                    f'コンソールログを {resolved_log_dir} に保存します。'
                )
        self._core = gui_core or GuiCore(log_directory=resolved_log_dir)
        # 先頭にスラッシュを付けないことで launch からの remap を可能にする。
        self._manual_pub = self.create_publisher(Bool, 'manual_start', 10)
        self._sig_pub = self.create_publisher(Int32, 'sig_recog', 10)
        self._road_pub = self.create_publisher(Bool, 'road_blocked', 10)
        self._obstacle_hint_pub = self.create_publisher(
            ObstacleAvoidanceHint, 'obstacle_avoidance_hint', 10
        )
        self._frame_image_path_pub = self.create_publisher(
            String, '/frame_image_path', 10
        )

        self.create_subscription(RouteState, 'route_state', self._core.update_route_state, 10)
        self.create_subscription(ManagerStatus, 'manager_status', self._core.update_manager_status, 10)
        self.create_subscription(Route, 'active_route', self._core.update_route, 10)
        self.create_subscription(FollowerState, 'follower_state', self._core.update_follower_state, 10)
        self.create_subscription(ObstacleAvoidanceHint, 'obstacle_avoidance_hint', self._core.update_obstacle_hint, 10)
        self.create_subscription(ImageMsg, 'sensor_viewer', self._core.update_sensor_viewer, 10)
        self.create_subscription(ImageMsg, 'usb_cam/image_raw', lambda msg: self._core.update_camera_image(msg, 'drive'), 10)
        self.create_subscription(ImageMsg, 'sig_det_imgs', lambda msg: self._core.update_camera_image(msg, 'signal'), 10)
        self.create_subscription(PoseStamped, 'active_target', self._core.update_active_target, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._core.update_amcl_pose,
            10,
        )
        self.create_subscription(Twist, 'cmd_vel', self._core.update_cmd_vel, 10)
        self.create_subscription(Bool, 'manual_start', self._core.update_manual_start, 10)
        self.create_subscription(Int32, 'sig_recog', self._core.update_sig_recog, 10)
        self.create_subscription(Bool, 'road_blocked', self._on_road_blocked, 10)

        self._command_timer = self.create_timer(0.1, self._process_commands)
        self._obstacle_override_timer = None
        self._obstacle_override_payload = None

    @property
    def core(self) -> GuiCore:
        """UiMain から利用する GuiCore インスタンスを返す。"""

        return self._core

    def _process_commands(self) -> None:
        while True:
            command = self._core.get_next_command(timeout=0.0)
            if command is None:
                break
            if command.command_type == GuiCommandType.MANUAL_START:
                msg = Bool(data=bool(command.payload.get('value', False)))
                self._manual_pub.publish(msg)
                self._core.update_manual_start(msg)
            elif command.command_type == GuiCommandType.SIG_RECOG:
                msg = Int32(data=int(command.payload.get('value', 0)))
                self._sig_pub.publish(msg)
                self._core.update_sig_recog(msg)
            elif command.command_type == GuiCommandType.ROAD_BLOCKED:
                msg = Bool(data=bool(command.payload.get('value', False)))
                self._road_pub.publish(msg)
                self._core.update_road_blocked(msg, source='console')
            elif command.command_type == GuiCommandType.FRAME_IMAGE_PATH:
                msg = String(data=str(command.payload.get('value', '')))
                self._frame_image_path_pub.publish(msg)
            elif command.command_type == GuiCommandType.OBSTACLE_HINT_OVERRIDE:
                self._start_obstacle_override(command.payload)
            elif command.command_type == GuiCommandType.OBSTACLE_HINT_STOP:
                self._stop_obstacle_override()
            else:
                self._core.handle_command(command)

    def _start_obstacle_override(self, payload) -> None:
        hint = ObstacleAvoidanceHint()
        hint.front_blocked = bool(payload.get('front_blocked', False))
        hint.front_clearance_m = float(payload.get('front_clearance_m', 0.0))
        hint.left_offset_m = float(payload.get('left_offset_m', 0.0))
        hint.right_offset_m = float(payload.get('right_offset_m', 0.0))
        self._obstacle_override_payload = hint
        if self._obstacle_override_timer is None:
            self._obstacle_override_timer = self.create_timer(2.0, self._publish_obstacle_override)
        self._publish_obstacle_override()

    def _stop_obstacle_override(self) -> None:
        if self._obstacle_override_timer is not None:
            self._obstacle_override_timer.cancel()
            self._obstacle_override_timer = None
        self._obstacle_override_payload = None
        hint = ObstacleAvoidanceHint()
        self._obstacle_hint_pub.publish(hint)

    def _publish_obstacle_override(self) -> None:
        if self._obstacle_override_payload is None:
            return
        self._obstacle_hint_pub.publish(self._obstacle_override_payload)

    def _on_road_blocked(self, msg: Bool) -> None:
        self._core.update_road_blocked(msg, source='external')


def main() -> None:
    rclpy.init()
    node = RobotConsoleNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    ui = UiMain(node.core)
    try:
        ui.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=2.0)


__all__ = ['RobotConsoleNode', 'main']
