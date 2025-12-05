
# -*- coding: utf-8 -*-
"""
robot_navigator_node.py

ROS2 (rclpy) 実装。ROS1 の navigator.py（TimeOptimalController）を踏襲しつつ、
パッケージ/ノード名 robot_navigator に準拠したノード本体の完全実装。

- Google Python Style Guide に沿った docstring / 型ヒントを付与
- Python 3 準拠
- 日本語コメントを簡潔に付与（初見でも理解しやすい粒度）
- 旧実装の制御ロジックと各種パラメータ値を踏襲
- LaserScan や各種トピック名は既定の相対名を用い、launch の remap で変更可能
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from route_msgs.msg import ObstacleAvoidanceHint, Route
from visualization_msgs.msg import Marker


@dataclass
class DebugInfo:
    """デバッグ用の情報レコード。CSV へ出力する内容を保持する。"""

    timestamp: float
    v_current: float
    w_current: float
    v_desired: float
    w_desired: float
    distance_error: float
    angle_diff: float
    angle_scaling: float
    v_desired_scaled: float
    obstacle_distance: Optional[float]


class RobotNavigator(Node):
    """robot_navigator ノード本体（ROS2版）。

    旧 ROS1 実装（TimeOptimalController）のロジック（角度誤差で線速度抑制、
    前方障害での減速/停止、PID による角速度生成など）を踏襲した。
    """

    def __init__(self) -> None:
        """ノードの初期化と通信・タイマー準備を行う。"""
        super().__init__('robot_navigator')

        # --- パラメータ宣言（既定値は従来実装を踏襲） ---
        self.declare_parameter('max_vel', 1.1)
        self.declare_parameter('max_w', 1.8)
        self.declare_parameter('max_acc_v', 1.0)
        self.declare_parameter('max_acc_w', 1.5)
        self.declare_parameter('pos_tol', 0.5)
        self.declare_parameter('pos_tol_exit_margin', 0.2)
        self.declare_parameter('ang_tol', 0.25)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('road_block_hold_sec', 5.0)

        # 旧実装の固定相当をパラメータ化
        self.declare_parameter('robot_width', 0.6)
        self.declare_parameter('safety_distance', 0.8)
        self.declare_parameter('min_obstacle_distance', 0.5)
        self.declare_parameter('obst_max_dist', 5.0)
        source_descriptor = ParameterDescriptor(
            description='障害物距離の取得モードを指定します',
            additional_constraints="must be either 'scan' or 'hint'",
        )
        self.declare_parameter('obstacle_distance_mode', 'hint', source_descriptor)

        self.declare_parameter('marker_frame', 'map')

        # ログ
        self.declare_parameter('log_csv_path', os.path.join(os.path.expanduser('~'), 'control_log.csv'))

        # --- パラメータ取得 ---
        self.max_v: float = float(self.get_parameter('max_vel').value)
        self.max_w: float = float(self.get_parameter('max_w').value)
        self.max_a_v: float = float(self.get_parameter('max_acc_v').value)
        self.max_a_w: float = float(self.get_parameter('max_acc_w').value)
        self.pos_tol: float = float(self.get_parameter('pos_tol').value)
        self.pos_tol_exit_margin: float = float(
            self.get_parameter('pos_tol_exit_margin').value
        )
        if self.pos_tol_exit_margin < 0.0:
            self.get_logger().warn('pos_tol_exit_margin が負値のため 0 に切り上げます。')
            self.pos_tol_exit_margin = 0.0
        self.ang_tol: float = float(self.get_parameter('ang_tol').value)
        self.control_rate_hz: float = float(self.get_parameter('control_rate_hz').value)
        self.dt: float = 1.0 / self.control_rate_hz
        self.road_block_hold_sec: float = float(self.get_parameter('road_block_hold_sec').value)
        if self.road_block_hold_sec < 0.0:
            self.get_logger().warn('road_block_hold_sec が負値のため 0 に切り上げます。')
            self.road_block_hold_sec = 0.0

        self.robot_width: float = float(self.get_parameter('robot_width').value)
        self.safety_distance: float = float(self.get_parameter('safety_distance').value)
        self.min_obstacle_distance: float = float(self.get_parameter('min_obstacle_distance').value)
        self.obst_max_dist: float = float(self.get_parameter('obst_max_dist').value)
        source_param = str(self.get_parameter('obstacle_distance_mode').value)
        self.obstacle_distance_mode: str = source_param.lower()
        if self.obstacle_distance_mode not in {'scan', 'hint'}:
            self.get_logger().warn(
                'obstacle_distance_mode は scan/hint のみを許容します。hint を使用します。'
            )
            self.obstacle_distance_mode = 'hint'
        self.marker_frame: str = str(self.get_parameter('marker_frame').value)

        self.log_csv_path: str = str(self.get_parameter('log_csv_path').value)

        # --- 通信に用いるトピック名（リマップ前の既定値） ---
        scan_topic = 'scan'
        odom_topic = 'odom'
        amcl_pose_topic = 'amcl_pose'
        goal_topic = 'active_target'
        road_block_topic = 'road_blocked'
        active_route_topic = 'active_route'
        # リマップでの変更を確実に反映させるため、既定は相対名とする
        cmd_vel_topic = 'cmd_vel'
        marker_topic = 'direction_marker'
        hint_topic = 'obstacle_avoidance_hint'

        # --- 角速度用 PID（旧実装値を踏襲） ---
        self.kp_w: float = 0.65
        self.ki_w: float = 0.001
        self.kd_w: float = 0.02
        self.integral_w: float = 0.0
        self.prev_yaw_error: float = 0.0
        self.integral_w_limit: float = self.max_w / max(self.ki_w, 1.0e-6)

        # --- 内部状態 ---
        self.current_pose: Optional[Pose] = None
        self.current_velocity: Optional[Twist] = None
        self.current_goal: Optional[Pose] = None
        self.obstacle_distance: Optional[float] = None
        self.prev_cmd_vel: Twist = Twist()  # 前回のcmd_vel（odometry代替用）
        self._goal_sequence: int = 0
        self._road_block_active: bool = False
        self._road_block_stop_until: Optional[float] = None
        self._road_block_goal_sequence: Optional[int] = None
        self._road_block_release_candidate_sequence: Optional[int] = None
        self._road_block_release_by_false: bool = False
        self.current_route_version: Optional[int] = None
        self._road_block_route_version: Optional[int] = None
        self._road_block_release_candidate_route_version: Optional[int] = None
        self._last_goal_stamp: Optional[Tuple[int, int]] = None
        self._road_block_goal_stamp: Optional[Tuple[int, int]] = None
        self._road_block_release_candidate_stamp: Optional[Tuple[int, int]] = None
        # 位置許容範囲へ入ったことをヒステリシス付きで保持するフラグ。
        self._within_goal_pos_tolerance: bool = False

        # --- Publisher/Subscriber の設定 ---
        # /cmd_vel は Reliable/Volatile で十分
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, pub_qos)

        # Marker は 1 深度で十分
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.marker_pub = self.create_publisher(Marker, marker_topic, marker_qos)

        # 購読（amcl, odom, goal は RELIABLE、scan は SensorDataQoS）
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, amcl_pose_topic, self.on_amcl_pose, 10)
        self.create_subscription(PoseStamped, goal_topic, self.on_goal, 10)
        self.create_subscription(Bool, road_block_topic, self.on_road_blocked, 10)
        route_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Route, active_route_topic, self.on_active_route, route_qos)
        if self.obstacle_distance_mode == 'scan':
            self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        else:
            hint_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                ObstacleAvoidanceHint, hint_topic, self.on_hint, hint_qos
            )
        
        # --- リマップ後の名称を保持（ログや診断用） ---
        self.scan_topic_name = self._resolve_topic_name(scan_topic)
        self.odom_topic_name = self._resolve_topic_name(odom_topic)
        self.amcl_pose_topic_name = self._resolve_topic_name(amcl_pose_topic)
        self.goal_topic_name = self._resolve_topic_name(goal_topic)
        self.road_block_topic_name = self._resolve_topic_name(road_block_topic)
        self.active_route_topic_name = self._resolve_topic_name(active_route_topic)
        self.cmd_vel_topic_name = self._resolve_topic_name(cmd_vel_topic)
        self.marker_topic_name = self._resolve_topic_name(marker_topic)
        self.hint_topic_name = self._resolve_topic_name(hint_topic)

        if self.obstacle_distance_mode == 'scan':
            self.get_logger().info(f'障害物距離ソース: {self.scan_topic_name}')
        else:
            self.get_logger().info(f'障害物距離ソース: {self.hint_topic_name}')

        # --- 制御タイマー ---
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.goal_log_timer = self.create_timer(1.0, self._log_goal_status)

        # --- CSV ログ準備 ---
        try:
            self._log_fp = open(self.log_csv_path, 'w', encoding='utf-8')
            header = (
                'timestamp,'
                'v_current,w_current,'
                'v_desired,w_desired,'
                'distance_error,angle_diff,'
                'angle_scaling,v_desired_scaled,'
                'obstacle_distance\n'
            )
            self._log_fp.write(header)
            self._log_fp.flush()
            self.get_logger().info(f'CSVログ出力: {self.log_csv_path}')
        except Exception as e:  # noqa: BLE001
            self._log_fp = None
            self.get_logger().warn(f'CSVログを開けませんでした: {e}')

        self.get_logger().info('robot_navigator 起動（ROS2 rclpy）')

    def _resolve_topic_name(self, name: str) -> str:
        """リマップ適用後のトピック名を取得する。"""

        try:
            return self.resolve_topic_name(name)
        except AttributeError:
            return name

    # -------------------- コールバック群 --------------------
    def on_odom(self, msg: Odometry) -> None:
        """/odom から現在速度を保持する。"""
        self.current_velocity = msg.twist.twist

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """/amcl_pose から現在姿勢（Pose）を保持する。"""
        self.current_pose = msg.pose.pose

    def on_goal(self, msg: PoseStamped) -> None:
        """目標トピック（PoseStamped）から目標姿勢（Pose）を保持する。"""
        stamp = msg.header.stamp
        stamp_tuple = (
            int(getattr(stamp, "sec", 0)),
            int(getattr(stamp, "nanosec", 0)),
        )
        previous_stamp = self._last_goal_stamp
        stamp_changed = previous_stamp != stamp_tuple

        self.current_goal = msg.pose
        if stamp_changed:
            self._goal_sequence += 1
            # 目標が更新された場合は PID 積分項をリセットし、残留バイアスを避ける。
            self.integral_w = 0.0
            self.prev_yaw_error = 0.0
            self._within_goal_pos_tolerance = False

        if self._road_block_active and stamp_changed:
            # road_blocked 停止明けは header.stamp が変化した指示のみ解除候補とみなす。
            self._road_block_release_candidate_sequence = self._goal_sequence
            self._road_block_release_candidate_stamp = stamp_tuple

        self._last_goal_stamp = stamp_tuple

    def on_active_route(self, msg: Route) -> None:
        """active_route メッセージを受信し、バージョン更新を監視する。"""
        version_value = int(getattr(msg, 'version', 0) or 0)
        self.current_route_version = version_value if version_value > 0 else None
        if not self._road_block_active or self.current_route_version is None:
            return

        if self._road_block_route_version is None:
            self._road_block_route_version = self.current_route_version
            return

        if self.current_route_version != self._road_block_route_version:
            self._road_block_release_candidate_route_version = self.current_route_version

    def on_road_blocked(self, msg: Bool) -> None:
        """道路封鎖イベントを受信し、停止制御フラグを更新する。"""
        now = time.monotonic()
        blocked = bool(getattr(msg, 'data', False))
        if blocked:
            self._road_block_active = True
            self._road_block_stop_until = now + self.road_block_hold_sec
            self._road_block_goal_sequence = self._goal_sequence
            self._road_block_release_candidate_sequence = None
            self._road_block_release_by_false = False
            self._road_block_route_version = self.current_route_version
            self._road_block_release_candidate_route_version = None
            self._road_block_goal_stamp = self._last_goal_stamp
            self._road_block_release_candidate_stamp = None
            self.get_logger().warn(
                f'road_blocked=True を受信。{self.road_block_hold_sec:.1f}秒間停止します。',
            )
        else:
            if self._road_block_active:
                hold_until = now + self.road_block_hold_sec
                if self._road_block_stop_until is None:
                    self._road_block_stop_until = hold_until
                else:
                    self._road_block_stop_until = max(self._road_block_stop_until, hold_until)
                self._road_block_release_candidate_sequence = self._goal_sequence
                self._road_block_release_candidate_route_version = self.current_route_version
                self._road_block_release_candidate_stamp = self._last_goal_stamp
                self._road_block_release_by_false = True
                self.get_logger().info(
                    'road_blocked=False を受信しました。停止解除条件の再評価を待ちます。',
                )
            else:
                self.get_logger().info(
                    'road_blocked=False を受信しましたが停止状態ではありません。',
                )

    def on_scan(self, msg: LaserScan) -> None:
        """/scan を処理して、前方矩形ウィンドウ内の最小前方距離を更新する。"""
        # ロボット幅の半分と検出最大距離を用意
        half_width = self.robot_width / 2.0
        max_detection_distance = self.obst_max_dist

        x_min: Optional[float] = None

        angle = msg.angle_min
        for r in msg.ranges:
            # 無効値を除外
            if r is None or r <= 0.0 or math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            # ビーム角度に基づく機体座標での投影
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # 前方矩形: 0 < x <= max_dist, |y| <= width/2
            if 0.0 < x <= max_detection_distance and abs(y) <= half_width:
                if x_min is None or x < x_min:
                    x_min = x

            angle += msg.angle_increment

        self._apply_obstacle_distance(x_min)

    def on_hint(self, msg: ObstacleAvoidanceHint) -> None:
        """ヒントメッセージから障害物距離を更新する。"""
        distance: Optional[float]
        if (
            msg.front_clearance_m is None
            or math.isinf(msg.front_clearance_m)
            or math.isnan(msg.front_clearance_m)
        ):
            distance = None
        else:
            distance = float(msg.front_clearance_m)
        self._apply_obstacle_distance(distance)

    def _apply_obstacle_distance(self, distance: Optional[float]) -> None:
        """障害物距離を正規化して内部状態へ反映する。"""
        if distance is None:
            self.obstacle_distance = None
            return

        if distance <= 0.0 or math.isnan(distance) or math.isinf(distance):
            self.obstacle_distance = None
            return

        self.obstacle_distance = float(distance)

    # -------------------- 制御ループ --------------------
    def on_timer(self) -> None:
        """周期制御ロジック。必要な入力が揃ったら cmd_vel と Marker を発行し、CSV ログを追記する。"""
        if self._should_stop_due_to_road_block():
            self._publish_stop_for_road_block()
            return

        if not (self.current_pose and self.current_velocity and self.current_goal):
            # 必要入力が揃わない場合は停止指令
            self._publish_stop_with_throttle()
            return

        cmd_vel, dbg = self.compute_time_optimal_cmd_vel(
            current_pose=self.current_pose,
            current_velocity=self.current_velocity,
            target_pose=self.current_goal,
        )

        # 指令を発行
        self.cmd_pub.publish(cmd_vel)

        # 前回のcmd_velを保存（odometryがない場合の代替用）
        self.prev_cmd_vel = cmd_vel

        # 進行方向の可視化
        self.publish_direction_marker(self.current_pose, dbg['v_desired'], dbg['w_desired'])

        # CSV ログ（可能なら）
        if self._log_fp:
            obst = '' if dbg['obstacle_distance'] is None else f"{dbg['obstacle_distance']:.6f}"
            line = (
                f"{dbg['timestamp']:.3f},"
                f"{dbg['v_current']:.6f},{dbg['w_current']:.6f},"
                f"{dbg['v_desired']:.6f},{dbg['w_desired']:.6f},"
                f"{dbg['distance_error']:.6f},{dbg['angle_diff']:.6f},"
                f"{dbg['angle_scaling']:.6f},{dbg['v_desired_scaled']:.6f},"
                f"{obst}\n"
            )
            self._log_fp.write(line)
            self._log_fp.flush()

    def _publish_stop_with_throttle(self) -> None:
        """入力未揃い時の安全停止（5秒スロットルの WARN ログ付き）。"""
        self.get_logger().warn(
            f"データ待ち（{self.amcl_pose_topic_name}, {self.odom_topic_name}, {self.goal_topic_name})",
            throttle_duration_sec=5.0,
        )
        self.cmd_pub.publish(Twist())

    def _publish_stop_for_road_block(self) -> None:
        """道路封鎖対応の停止指令と状況ログを出力する。"""
        now = time.monotonic()
        wait_reason = 'タイマ満了待ち'
        if self._road_block_stop_until is not None and now >= self._road_block_stop_until:
            pending: list[str] = []
            if not self._has_goal_release_candidate():
                pending.append('active_target更新待ち')
            if not self._has_route_release_candidate():
                pending.append('active_route更新待ち')
            wait_reason = '・'.join(pending) if pending else '解除判定処理中'
        self.get_logger().warn(
            f'road_blocked停止中（{wait_reason}）',
            throttle_duration_sec=1.0,
        )
        self.cmd_pub.publish(Twist())

    def _should_stop_due_to_road_block(self) -> bool:
        """道路封鎖イベントによる停止継続が必要か判定する。"""
        if not self._road_block_active:
            return False

        now = time.monotonic()
        if self._road_block_stop_until is not None and now < self._road_block_stop_until:
            return True

        # active_route の更新有無にかかわらず、active_target の内容更新を必須条件とする。
        if not self._has_goal_release_candidate():
            return True

        self._reset_road_block_state()
        self.get_logger().info('road_blocked解除条件を満たしたため制御を再開します。')
        return False

    def _has_goal_release_candidate(self) -> bool:
        """active_target の更新により解除条件が満たされたか判定する。"""
        if self._road_block_release_candidate_stamp is None:
            return False
        if self._road_block_release_by_false:
            return True
        if (
            self._road_block_goal_stamp is not None
            and self._road_block_release_candidate_stamp == self._road_block_goal_stamp
        ):
            return False
        goal_sequence_threshold = self._road_block_goal_sequence or 0
        release_sequence = self._road_block_release_candidate_sequence
        return release_sequence is not None and release_sequence > goal_sequence_threshold

    def _has_route_release_candidate(self) -> bool:
        """active_route の更新により解除条件が満たされたか判定する。"""
        if self._road_block_release_candidate_route_version is None:
            return False
        if self._road_block_route_version is None:
            return True
        return self._road_block_release_candidate_route_version != self._road_block_route_version

    def _reset_road_block_state(self) -> None:
        """道路封鎖制御用の内部状態を初期化する。"""
        self._road_block_active = False
        self._road_block_stop_until = None
        self._road_block_goal_sequence = None
        self._road_block_release_candidate_sequence = None
        self._road_block_release_by_false = False
        self._road_block_route_version = None
        self._road_block_release_candidate_route_version = None
        self._road_block_goal_stamp = None
        self._road_block_release_candidate_stamp = None

    def _log_goal_status(self) -> None:
        """active_target の座標と距離を 1 秒周期で INFO ログ出力する。"""
        if not self.current_goal:
            self.get_logger().info(
                f"active_target待機中: 目標未受信（{self.goal_topic_name}）",
            )
            return

        goal_pos = self.current_goal.position
        if self.current_pose:
            pose_pos = self.current_pose.position
            distance = math.hypot(goal_pos.x - pose_pos.x, goal_pos.y - pose_pos.y)

            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            target_yaw = self.quaternion_to_yaw(self.current_goal.orientation)
            bearing = math.atan2(goal_pos.y - pose_pos.y, goal_pos.x - pose_pos.x)
            yaw_error = self.normalize_angle(bearing - current_yaw)
            if distance <= self.pos_tol:
                yaw_error = self.normalize_angle(target_yaw - current_yaw)

            angle_error_deg = math.degrees(yaw_error)

            self.get_logger().info(
                f"active_target: x={goal_pos.x:.3f}, y={goal_pos.y:.3f} / 距離={distance:.3f}m"
                f" / 角度誤差={angle_error_deg:.2f}°",
            )
        else:
            self.get_logger().info(
                f"active_target: x={goal_pos.x:.3f}, y={goal_pos.y:.3f}"
                " / 現在位置未取得のため距離・角度不明",
            )

    # -------------------- ユーティリティ --------------------
    @staticmethod
    def quaternion_to_yaw(q) -> float:
        """クォータニオンからヨー角を計算する。

        Args:
            q: geometry_msgs/Quaternion

        Returns:
            float: [-pi, pi] の範囲のヨー角 [rad]。
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """角度を [-pi, pi] に正規化する。"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # -------------------- コア計算 --------------------
    def compute_time_optimal_cmd_vel(
        self,
        current_pose: Pose,
        current_velocity: Twist,
        target_pose: Pose,
    ) -> Tuple[Twist, Dict[str, float]]:
        """時間最適志向の近似ロジックで cmd_vel を算出する。

        - 角速度は PID で計算（旧実装のゲイン）
        - 線速度は角度差でスケールし、障害物距離に応じて減速/停止
        - 加速度制限は線速度の上昇側を dt*max_acc_v で制限（旧実装踏襲）

        Args:
            current_pose: 現在姿勢。
            current_velocity: 現在速度。
            target_pose: 目標姿勢。

        Returns:
            Tuple[Twist, Dict[str, float]]: 出力 Twist とデバッグ辞書。
        """
        # 現在速度を抽出（odometryがない場合は前回のcmd_velを使用）
        if current_velocity:
            v_current = float(current_velocity.linear.x)
            w_current = float(current_velocity.angular.z)
        else:
            v_current = float(self.prev_cmd_vel.linear.x)
            w_current = float(self.prev_cmd_vel.angular.z)

        # 位置・姿勢誤差を算出
        cx, cy = current_pose.position.x, current_pose.position.y
        tx, ty = target_pose.position.x, target_pose.position.y
        current_yaw = self.quaternion_to_yaw(current_pose.orientation)
        target_yaw = self.quaternion_to_yaw(target_pose.orientation)

        dx = tx - cx
        dy = ty - cy
        distance_error = math.hypot(dx, dy)

        if self._within_goal_pos_tolerance:
            if distance_error >= self.pos_tol + self.pos_tol_exit_margin:
                self._within_goal_pos_tolerance = False
        else:
            if distance_error <= self.pos_tol:
                self._within_goal_pos_tolerance = True

        within_pos_tolerance = self._within_goal_pos_tolerance

        # 目標姿勢ではなく、目標位置へのベアリングで方向誤差を求める。
        # 旧ROS1実装同様、位置到達前に目標姿勢のヨー角を使うと、
        # ゴールを通過した際に進行方向を維持したまま離脱してしまう。
        target_bearing = math.atan2(dy, dx)
        if within_pos_tolerance:
            yaw_error = self.normalize_angle(target_yaw - current_yaw)
        else:
            yaw_error = self.normalize_angle(target_bearing - current_yaw)
        angle_diff = abs(yaw_error)

        # --- 角速度（PID + アンチワインドアップ）---
        prev_error = self.prev_yaw_error
        if prev_error * yaw_error < 0.0:
            # 誤差符号が反転した場合は積分項をリセットし、飽和からの回復を早める。
            self.integral_w = 0.0

        integral_candidate = self.integral_w + yaw_error * self.dt
        integral_candidate = max(
            -self.integral_w_limit,
            min(self.integral_w_limit, integral_candidate),
        )
        derivative_w = (yaw_error - prev_error) / self.dt
        w_unsat = (
            self.kp_w * yaw_error
            + self.ki_w * integral_candidate
            + self.kd_w * derivative_w
        )

        if w_unsat > self.max_w:
            w_desired = self.max_w
            if yaw_error < 0.0:
                # 逆方向に誤差が出ている場合のみ積分を更新し、復帰を促す。
                self.integral_w = integral_candidate
        elif w_unsat < -self.max_w:
            w_desired = -self.max_w
            if yaw_error > 0.0:
                self.integral_w = integral_candidate
        else:
            w_desired = w_unsat
            self.integral_w = integral_candidate

        self.prev_yaw_error = yaw_error

        # --- 線速度（加速度上限 + 角度差スケール）---
        if distance_error > 0.0 and not within_pos_tolerance:
            # 上昇側の加速度制限（旧実装相当）
            v_ref = min(v_current + self.max_a_v * self.dt, self.max_v)
        else:
            v_ref = 0.0

        # 角度差に応じたスケール（旧実装：1 - |yaw|/pi を下限0でクリップ）
        angle_scaling = max(0.0, 1.0 - (angle_diff / math.pi))
        v_scaled = v_ref * angle_scaling

        # --- 障害物に応じた減速/停止 ---
        if self.obstacle_distance is not None:
            max_decel = self.max_a_v
            # 停止距離を計算する際は、次のステップでの速度（v_scaled）を使用
            # これにより、停止状態からの加速を許可する
            stopping_distance = (v_scaled ** 2) / (2.0 * max_decel) + self.min_obstacle_distance

            if self.obstacle_distance <= stopping_distance:
                # 完全停止（緊急停止領域）
                v_scaled = 0.0
                if abs(yaw_error) <= self.ang_tol:
                    # 目標向きに十分近い場合のみ角速度も停止
                    w_desired = 0.0
            elif self.obstacle_distance < self.safety_distance:
                # 安全距離内では線形に減速（min_obst_dist で 0、safety で v_scaled）
                denom = max(1e-6, (self.safety_distance - self.min_obstacle_distance))
                scale = (self.obstacle_distance - self.min_obstacle_distance) / denom
                scale = max(0.0, min(1.0, scale))
                v_scaled *= scale

        # 最大速度再チェック
        v_scaled = max(0.0, min(self.max_v, v_scaled))

        # ゴール到達判定（位置・角度の両方）
        if within_pos_tolerance and abs(yaw_error) <= self.ang_tol:
            v_scaled = 0.0
            w_desired = 0.0
            self.integral_w = 0.0

        # 出力メッセージ作成
        cmd = Twist()
        cmd.linear.x = float(v_scaled)
        cmd.angular.z = float(w_desired)

        dbg: Dict[str, float] = {
            'timestamp': time.time(),
            'v_current': v_current,
            'w_current': w_current,
            'v_desired': float(v_ref),
            'w_desired': float(w_desired),
            'distance_error': float(distance_error),
            'angle_diff': float(angle_diff),
            'angle_scaling': float(angle_scaling),
            'v_desired_scaled': float(v_scaled),
            'obstacle_distance': (None if self.obstacle_distance is None else float(self.obstacle_distance)),
        }
        return cmd, dbg

    # -------------------- 可視化 --------------------
    def publish_direction_marker(self, current_pose: Pose, v_desired: float, w_desired: float) -> None:
        """進行方向の矢印 Marker を /direction_marker に発行する。

        Args:
            current_pose: 現在姿勢。
            v_desired: 計算された線速度。
            w_desired: 計算された角速度。
        """
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'direction_marker'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # 矢印の始点/終点（現在位置＋向き）
        yaw = self.quaternion_to_yaw(current_pose.orientation)
        start = Point(x=current_pose.position.x, y=current_pose.position.y, z=0.0)
        length = max(0.3, min(2.0, 0.5 + 0.5 * abs(v_desired)))  # 速度に応じ長さを少し変化
        end = Point(
            x=current_pose.position.x + length * math.cos(yaw),
            y=current_pose.position.y + length * math.sin(yaw),
            z=0.0,
        )
        marker.points = [start, end]

        # スケール（矢の太さ）
        marker.scale.x = 0.05  # シャフト直径
        marker.scale.y = 0.2   # ヘッド直径
        marker.scale.z = 0.0   # ヘッド長さ

        # 色（赤）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = Duration(seconds=0.0).to_msg()
        self.marker_pub.publish(marker)

    def publish_stop_command(self) -> None:
        """終了時に停止指令を送出し、再起動時の安全性を高める。"""
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        stop_twist.linear.z = 0.0
        stop_twist.angular.x = 0.0
        stop_twist.angular.y = 0.0
        stop_twist.angular.z = 0.0
        self.get_logger().info('終了処理として cmd_vel に速度ゼロを publish します。')
        self.cmd_pub.publish(stop_twist)

    # -------------------- 終了処理 --------------------
    def destroy_node(self) -> bool:
        """ノード破棄時に停止指令を送り、ログファイルをクローズする。"""
        try:
            self.publish_stop_command()
            if hasattr(self, '_log_fp') and self._log_fp:
                self._log_fp.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    """エントリポイント。"""
    rclpy.init()
    node = RobotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
