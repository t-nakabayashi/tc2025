#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot_simulator ノード実装.

/active_target を購読し、簡易ロボット運動（等速直線運動）をシミュレートして
/amcl_pose を 100ms 周期で配信する。必要に応じて /tf も配信可能。

主な機能と仕様:
-----------------
- 最初の active_target 受信時: 目標の向きと反対(180°)側に 5m 離れた点を初期位置とする
- 等速直線運動: 既定 5 km/h (= 1.3889 m/s)、周期 100ms
- 極小解停止: target が不変かつ距離 <= 1m で、次ステップが遠ざかるなら停止
- QoS: /active_target は RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)
- すべての出力 frame_id は "map"（パラメータで変更可）
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Vector3
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time


# =========================
# ユーティリティ関数群
# =========================

def yaw_to_quat(yaw: float) -> Quaternion:
    """ヨー (Z軸回り) 角度[rad] を四元数に変換する.

    Args:
        yaw: ヨー角 [rad]

    Returns:
        Quaternion: x=y=z=0 の平面回転を表す四元数.
    """
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    """四元数からヨー角[rad]を計算する（ロール/ピッチは0前提の簡易式）."""
    # z,w のみ使用（平面回転）
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def clamp(v: float, lo: float, hi: float) -> float:
    """値を [lo, hi] に丸める."""
    return max(lo, min(hi, v))


def distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """2次元距離を返す."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


@dataclass
class Pose2D:
    """2D 平面上の位置姿勢."""
    x: float
    y: float
    yaw: float  # [rad]

    def as_tuple(self) -> Tuple[float, float]:
        """(x, y) のタプルを返す."""
        return (self.x, self.y)


# =========================
# メインノード
# =========================

class RobotSimulatorNode(Node):
    """/active_target 追従を行うロボットシミュレータ."""

    def __init__(self) -> None:
        super().__init__('robot_simulator')

        # ---- パラメータ宣言（既定値と説明） ----
        self.declare_parameter('speed_kmph', 5.0)              # 等速直線の速度[km/h]
        self.declare_parameter('timer_period_ms', 100)         # 更新周期[ms]
        self.declare_parameter('frame_id', 'map')              # 出力 frame_id
        self.declare_parameter('child_frame_id', 'base_link')  # /tf 用の child frame
        self.declare_parameter('publish_tf', False)            # /tf を配信するか
        self.declare_parameter('init_offset_m', 5.0)           # 初期位置のオフセット[m]
        self.declare_parameter('stop_radius_m', 1.0)           # 極小解停止の半径[m]
        self.declare_parameter('noise_pos_std_m', 0.0)         # 位置ノイズ σ[m]
        self.declare_parameter('noise_yaw_std_deg', 0.0)       # yaw ノイズ σ[deg]
        # 共分散は対角成分（x, y, z, roll, pitch, yaw）の分散値をパラメータで与える
        self.declare_parameter('pose_cov_diag',
                               [0.02, 0.02, 0.04, 0.0, 0.0, math.radians(2.0) ** 2])
        self.declare_parameter('log_debug', False)             # デバッグログ出力

        # ---- パラメータ取得 ----
        self.speed_kmph: float = float(self.get_parameter('speed_kmph').get_parameter_value().double_value)
        self.timer_period_ms: int = int(self.get_parameter('timer_period_ms').get_parameter_value().integer_value)
        self.frame_id: str = str(self.get_parameter('frame_id').get_parameter_value().string_value)
        self.child_frame_id: str = str(self.get_parameter('child_frame_id').get_parameter_value().string_value)
        self.publish_tf: bool = bool(self.get_parameter('publish_tf').get_parameter_value().bool_value)
        self.init_offset_m: float = float(self.get_parameter('init_offset_m').get_parameter_value().double_value)
        self.stop_radius_m: float = float(self.get_parameter('stop_radius_m').get_parameter_value().double_value)
        self.noise_pos_std_m: float = float(self.get_parameter('noise_pos_std_m').get_parameter_value().double_value)
        self.noise_yaw_std_deg: float = float(self.get_parameter('noise_yaw_std_deg').get_parameter_value().double_value)
        self.pose_cov_diag: List[float] = list(self.get_parameter('pose_cov_diag').get_parameter_value().double_array_value)
        self.log_debug: bool = bool(self.get_parameter('log_debug').get_parameter_value().bool_value)

        # ---- パラメータ検証 ----
        if self.speed_kmph <= 0.0:
            raise ValueError('speed_kmph は正の値である必要があります。')
        if self.timer_period_ms <= 0:
            raise ValueError('timer_period_ms は正の整数である必要があります。')
        if len(self.pose_cov_diag) != 6:
            raise ValueError('pose_cov_diag は長さ6（x,y,z,roll,pitch,yaw の分散）である必要があります。')

        # ---- 内部状態 ----
        self._v_mps: float = self.speed_kmph * 1000.0 / 3600.0  # 速度[m/s]
        self._dt: float = self.timer_period_ms / 1000.0         # 周期[s]
        self._pose: Optional[Pose2D] = None                     # 現在のロボット位置姿勢
        self._have_target: bool = False                         # active_target を受信済みか
        self._target_pose: Optional[Pose2D] = None              # 目標の位置姿勢（最新）
        self._target_stamp: Optional[Time] = None               # 目標のヘッダ時刻（最新）
        self._target_changed_since_last_step: bool = False      # 前回ステップ以降に target が更新されたか

        # ---- QoS 設定 ----
        # /active_target は最新1件を確実に受け取りたいので、TL + RELIABLE + KEEP_LAST(1)
        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- 通信設定 ----
        self._sub_target = self.create_subscription(
            PoseStamped,
            '/active_target',
            self._on_active_target,
            target_qos,
        )

        self._pub_pose = self.create_publisher(PoseStamped, '/amcl_pose', 10)

        if self.publish_tf:
            self._pub_tf = self.create_publisher(TFMessage, '/tf', 10)
        else:
            self._pub_tf = None

        # ---- タイマ設定 ----
        self._timer = self.create_timer(self._dt, self._on_timer)

        # ---- 起動ログ ----
        self.get_logger().info(
            f'robot_simulator 起動: v={self._v_mps:.3f} m/s, dt={self._dt:.3f} s, '
            f'frame_id={self.frame_id}, publish_tf={self.publish_tf}'
        )

    # =========================
    # コールバック・更新処理
    # =========================

    def _on_active_target(self, msg: PoseStamped) -> None:
        """/active_target 受信コールバック.

        初回受信時は、ターゲットの向きの反対へ init_offset_m だけ離れた点を初期位置とする。
        以降は最新のターゲットを内部状態に保持する。
        """
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            # frame が一致しない場合は警告のみ（座標変換はしない想定）
            self.get_logger().warn(
                f'/active_target frame_id 不一致: got="{msg.header.frame_id}" expected="{self.frame_id}"'
            )

        # 目標の姿勢（位置 + ヨー）を取り出す
        t_x = float(msg.pose.position.x)
        t_y = float(msg.pose.position.y)
        t_yaw = quat_to_yaw(msg.pose.orientation)
        print("Target pose: x={:.2f}, y={:.2f}, yaw={:.2f}".format(t_x, t_y, t_yaw))
        self._target_pose = Pose2D(t_x, t_y, t_yaw)
        self._target_stamp = msg.header.stamp
        self._target_changed_since_last_step = True

        if not self._have_target:
            # 初回のみ初期位置を設定（ターゲットの背後 init_offset_m）
            vx = math.cos(t_yaw)
            vy = math.sin(t_yaw)
            p0_x = t_x - self.init_offset_m * vx
            p0_y = t_y - self.init_offset_m * vy
            # 初期の向きは「ターゲット方向を向く」
            init_yaw = math.atan2(t_y - p0_y, t_x - p0_x)
            self._pose = Pose2D(p0_x, p0_y, init_yaw)
            self._have_target = True
            if self.log_debug:
                self.get_logger().info(
                    f'初期化: P0=({p0_x:.3f},{p0_y:.3f}), yaw={math.degrees(init_yaw):.1f}°'
                )

    def _on_timer(self) -> None:
        """運動更新タイマ（既定 100ms）."""
        if not self._have_target or self._pose is None or self._target_pose is None:
            # まだターゲットが無ければ何もしない
            return

        # 現在の位置・ターゲット
        px, py = self._pose.x, self._pose.y
        tx, ty = self._target_pose.x, self._target_pose.y

        # ターゲットへの距離と方向
        dx = tx - px
        dy = ty - py
        d = math.hypot(dx, dy)

        if d > 1e-9:
            ux = dx / d
            uy = dy / d
        else:
            ux = 0.0
            uy = 0.0

        # 次の一歩（等速直線運動）
        step = self._v_mps * self._dt
        cand_x = px + step * ux
        cand_y = py + step * uy

        # 極小解停止: target が変化していない & 1m以内 & 次で遠ざかるなら停止
        # 「target 不変」は、直前ステップ以降に更新が無いことを意味する
        target_changed = self._target_changed_since_last_step
        self._target_changed_since_last_step = False  # 消費

        dist_now = d
        dist_next = math.hypot(tx - cand_x, ty - cand_y)
        print("Dist:{:.2f}m".format(dist_now))

        will_stop = (not target_changed) and (dist_now <= self.stop_radius_m) and (dist_next > dist_now + 1e-12)

        if will_stop:
            new_x, new_y = px, py  # 停止
        else:
            new_x, new_y = cand_x, cand_y

        # 向きは常にターゲット方向を向く（停止時も向き更新）
        if ux != 0.0 or uy != 0.0:
            new_yaw = math.atan2(uy, ux)
        else:
            new_yaw = self._pose.yaw

        # ノイズ（見かけ上の出力だけに付与）
        out_x = new_x + (random.gauss(0.0, self.noise_pos_std_m) if self.noise_pos_std_m > 0.0 else 0.0)
        out_y = new_y + (random.gauss(0.0, self.noise_pos_std_m) if self.noise_pos_std_m > 0.0 else 0.0)
        out_yaw = new_yaw + (math.radians(random.gauss(0.0, self.noise_yaw_std_deg))
                             if self.noise_yaw_std_deg > 0.0 else 0.0)

        # 内部状態を更新（内部はノイズなしの真値で保持）
        self._pose.x = new_x
        self._pose.y = new_y
        self._pose.yaw = new_yaw

        # /amcl_pose を配信
        self._publish_amcl_pose(out_x, out_y, out_yaw)

        # /tf が有効なら配信
        if self._pub_tf is not None:
            self._publish_tf(out_x, out_y, out_yaw)

        if self.log_debug:
            self.get_logger().debug(
                f'update: d={dist_now:.3f} -> ({new_x:.3f},{new_y:.3f}) '
                f'yaw={math.degrees(new_yaw):.1f}° stop={will_stop}'
            )

    # =========================
    # 出力（Publish）
    # =========================

    def _publish_amcl_pose(self, x: float, y: float, yaw: float) -> None:
        """/amcl_pose を配信する."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quat(yaw)

        self._pub_pose.publish(msg)

    def _publish_tf(self, x: float, y: float, yaw: float) -> None:
        """/tf を配信する（有効時のみ）."""
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.frame_id
        ts.child_frame_id = self.child_frame_id
        ts.transform.translation.x = float(x)
        ts.transform.translation.y = float(y)
        ts.transform.translation.z = 0.0
        ts.transform.rotation = yaw_to_quat(yaw)

        tf_msg = TFMessage(transforms=[ts])
        assert self._pub_tf is not None
        self._pub_tf.publish(tf_msg)


# =========================
# エントリポイント
# =========================

def main() -> None:
    """エントリポイント."""
    rclpy.init()
    node = RobotSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

