#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
route_planner ノード（GetRoute 完了 + UpdateRoute 完了：仮想エッジ/履歴/経度キー修正対応）

主な機能と仕様:
-----------------
本ノードは、複数の固定/可変ブロックから構成されるルートを生成・配布（GetRoute）し、
走行中の経路封鎖イベントに応じて可変ブロックを再探索した新ルートを配布（UpdateRoute）する。

"""

from __future__ import annotations

import csv
import math
import os
import sys
import traceback
from pathlib import Path
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple

import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from sensor_msgs.msg import Image

from route_msgs.msg import Route, Waypoint
from route_msgs.srv import GetRoute, UpdateRoute

# 可変ルート探索（外部モジュール）
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

from graph_solver import solve_variable_route  # 実プロジェクトの配置に合わせて import 経路を調整すること

def _copy_pose(src: Pose) -> Pose:
    p = Pose()
    p.position.x = src.position.x
    p.position.y = src.position.y
    p.position.z = src.position.z
    p.orientation.x = src.orientation.x
    p.orientation.y = src.orientation.y
    p.orientation.z = src.orientation.z
    p.orientation.w = src.orientation.w
    return p


def _load_png_as_image(path: str) -> Image:
    """PNG画像をバイト読み込みして sensor_msgs/Image に変換"""
    import cv2
    img = Image()
    img.header = Header()
    img.header.frame_id = "map"
    cv_img = cv2.imread(path, cv2.IMREAD_COLOR)
    if cv_img is None:
        raise FileNotFoundError(f"Cannot read PNG file: {path}")
    img.height, img.width, _ = cv_img.shape
    img.encoding = "bgr8"
    img.step = cv_img.shape[1] * 3
    img.data = cv_img.tobytes()
    return img


# ===== データクラス ==================================================================

@dataclass
class SegmentCacheEntry:
    """CSVから読み込んだ waypoint 配列のキャッシュ。

    Attributes:
        segment_id: CSVファイル（相対/絶対パス）を識別するID。
        waypoints: CSVから復元済みのWaypoint配列。ここでは端点ラベルの刻印等は行わない。
    """
    segment_id: str
    waypoints: List[Waypoint]


@dataclass
class WaypointOrigin:
    """waypoint の由来情報（UpdateRoute の prev/next 判定・仮想エッジ生成に利用）。

    Attributes:
        block_name: 所属ブロック名。
        block_index: 所属ブロックのインデックス（YAML読み込み時に付与）。
        segment_id: 固定CSV or 可変エッジCSV（仮想は None）。
        edge_u: 可変エッジの source（端点U）。固定ブロックでは None。
        edge_v: 可変エッジの target（端点V）。固定ブロックでは None。
        index_in_edge: エッジ内のローカルインデックス（可変エッジ連結時の“採用した向き”でのindex）。
        u_first: True なら「このWaypointOriginを生成した時のエッジ向きが U→V」であったことを示す。
                 False なら V→U 向き（= CSVを reverse して使用）。
    """
    block_name: str
    block_index: int
    segment_id: Optional[str]
    edge_u: Optional[str]
    edge_v: Optional[str]
    index_in_edge: Optional[int]
    u_first: Optional[bool]


# ===== ユーティリティ関数 =============================================================

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """yaw[rad] をクォータニオンに変換する（roll,pitch=0固定）。"""
    q = Quaternion()
    half = yaw * 0.5
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def load_route_image_from_solver(result: Dict[str, Any]) -> Optional[Image]:
    """solve_variable_route の戻り値に含まれる画像ファイルパスから Image を生成する。"""
    img_path = result.get("route_image_path")
    if not img_path or not os.path.exists(img_path):
        print(f"[WARN] route_image_path not found or missing: {img_path}")
        return None
    return _load_png_as_image(img_path)


def make_text_png_image(text: str = "No variable route in this plan") -> Image:
    """最小1x1の有効Imageを返す。"""
    img = Image()
    img.header = Header()
    img.header.frame_id = "map"
    img.height = 1
    img.width = 1
    img.encoding = "bgr8"
    img.step = 3
    img.data = b"\x00\x00\x00"
    return img


def almost_same_xy(ax: float, ay: float, bx: float, by: float, eps: float = 1e-6) -> bool:
    """2点のXY距離が閾値以下か判定。"""
    return math.hypot(ax - bx, ay - by) <= eps


def concat_with_dedup(base: List[Waypoint], ext: List[Waypoint], eps: float = 1e-6) -> List[Waypoint]:
    """2つの waypoint 配列を連結。境界が同一点なら ext 先頭を除外（重複排除）して結合する。"""
    if not base:
        return list(ext)
    if not ext:
        return base
    bx = base[-1].pose.position.x
    by = base[-1].pose.position.y
    ex = ext[0].pose.position.x
    ey = ext[0].pose.position.y
    if almost_same_xy(bx, by, ex, ey, eps):
        # ext[0]にラベルがありbase[-1]に無ければ移す（端点ラベルを保護）
        if ext[0].label and not base[-1].label:
            base[-1].label = ext[0].label
        return base + ext[1:]
    return base + ext


def stamp_edge_end_labels(wps: List[Waypoint], src_label: str, dst_label: str) -> None:
    """エッジ両端の waypoint にノードラベルを刻印（境界識別・スライス用）。"""
    if not wps:
        return
    wps[0].label = src_label
    wps[-1].label = dst_label


def resolve_path(base_dir: Optional[str], path_str: str) -> str:
    """相対パスを base_dir 起点で解決。base_dir 未指定なら path_str をそのまま返す。"""
    if not base_dir:
        return path_str
    if os.path.isabs(path_str):
        return path_str
    return os.path.normpath(os.path.join(base_dir, path_str))


def parse_waypoint_csv(csv_path: str) -> List[Waypoint]:
    """waypoint CSV を読み込み、Waypoint 配列へ変換する。

    CSV定義（新仕様）:
      label,latitude,longitude,x,y,z,q1,q2,q3,q4,right_is_open,left_is_open,line_is_stop,signal_is_stop,isnot_skipnum

    注意:
      - 後方互換のため label が無い場合は id もしくは num を文字列化して label とする。
      - longitude の綴りは正しいもののみ対応（longuitude は非対応）。
    """
    waypoints: List[Waypoint] = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            wp = Waypoint()

            # ラベル: label 無しなら id もしくは num を文字列化して使用
            label_val = row.get("label")
            if label_val is None or label_val == "":
                label_val = row.get("id") or row.get("num") or ""
            wp.label = str(label_val)

            # index は後で再採番するが、ここではCSV内の行番号を仮設定
            wp.index = i

            # 位置
            try:
                wp.pose.position.x = float(row.get("x", 0.0))
                wp.pose.position.y = float(row.get("y", 0.0))
                wp.pose.position.z = float(row.get("z", 0.0))
            except ValueError:
                raise ValueError(f"Invalid XYZ in {csv_path} at row {i+1}")

            # 姿勢（CSV記録値。後段で必要に応じて再計算する）
            try:
                wp.pose.orientation.x = float(row.get("q1", 0.0))
                wp.pose.orientation.y = float(row.get("q2", 0.0))
                wp.pose.orientation.z = float(row.get("q3", 0.0))
                wp.pose.orientation.w = float(row.get("q4", 1.0))
            except ValueError:
                raise ValueError(f"Invalid quaternion in {csv_path} at row {i+1}")

            # 緯度経度（存在しない場合もある）
            # 各種属性（旧カラム名との互換を確保）
            def _bool_from_int(s: Any) -> bool:
                try:
                    return bool(int(str(s)))
                except Exception:
                    return False

            def _float_from_any(value: Any) -> float:
                try:
                    return float(value)
                except Exception:
                    return 0.0

            wp.right_open = _float_from_any(
                row.get("right_open", row.get("right_is_open", 0.0))
            )
            wp.left_open = _float_from_any(
                row.get("left_open", row.get("left_is_open", 0.0))
            )
            wp.line_stop = _bool_from_int(row.get("line_stop", row.get("line_is_stop", 0)))
            wp.signal_stop = _bool_from_int(
                row.get("signal_stop", row.get("signal_is_stop", 0))
            )
            wp.not_skip = _bool_from_int(row.get("not_skip", row.get("isnot_skipnum", 0)))

            waypoints.append(wp)
    return waypoints


def indexing(wps: List[Waypoint]) -> None:
    """Waypoint.index を 0..N-1 に再採番する。"""
    for i, wp in enumerate(wps):
        wp.index = i


def slice_by_labels(
    wps: List[Waypoint],
    start_label: str,
    goal_label: str,
) -> Tuple[List[Waypoint], int]:
    """start_label を前方から、goal_label を後方から検索し、その範囲でスライスする。
    start_label または goal_label が空文字/None の場合は、
    それぞれ先頭または末尾ノードを採用する。

    Returns:
        (sliced_list, start_offset) を返す。
        start_offset は元配列に対する切り出し開始位置。
    """
    start_idx: Optional[int] = None
    goal_idx: Optional[int] = None

    # --- start_label の補完または検索 ---
    if not start_label:  # 空文字や None の場合は先頭ノードから
        start_idx = 0
        print(f"[WARN][slice_by_labels] start_label is empty; using first waypoint '{wps[0].label}'")
    else:
        for i, wp in enumerate(wps):
            if wp.label == start_label:
                start_idx = i
                break

    # --- goal_label の補完または検索 ---
    if not goal_label:  # 空文字や None の場合は末尾ノードまで
        goal_idx = len(wps) - 1
        print(f"[WARN][slice_by_labels] goal_label is empty; using last waypoint '{wps[-1].label}'")
    else:
        for j in range(len(wps) - 1, -1, -1):
            if wps[j].label == goal_label:
                goal_idx = j
                break

    # --- バリデーション ---
    if start_idx is None:
        raise ValueError(f"Start label '{start_label}' not found.")
    if goal_idx is None:
        raise ValueError(f"Goal label '{goal_label}' not found.")
    if start_idx > goal_idx:
        raise ValueError("Invalid slice order: start > goal.")

    # --- スライス処理 ---
    sliced = wps[start_idx: goal_idx + 1]
    if len(sliced) <= 1:
        # start == goal で1点のみはエラー扱い（要件）
        raise ValueError("Single-point route not allowed.")

    return sliced, start_idx


def calc_total_distance(wps: List[Waypoint]) -> float:
    """総距離[m]を算出。"""
    dist = 0.0
    for i in range(1, len(wps)):
        x0, y0 = wps[i - 1].pose.position.x, wps[i - 1].pose.position.y
        x1, y1 = wps[i].pose.position.x, wps[i].pose.position.y
        dist += math.hypot(x1 - x0, y1 - y0)
    return dist


def adjust_orientations(
    wps: List[Waypoint],
    recalc_ranges: List[Tuple[int, int]],
    block_boundaries: List[int],
) -> None:
    """姿勢を再計算する範囲を限定して実施する。

    Args:
        wps: ルート全体のwaypoints（スライス後）。
        recalc_ranges: 完全再計算すべき連続区間のインデックス範囲（start_idx, end_idx）。逆走区間や仮想区間を入れる。
        block_boundaries: ブロック末尾インデックス（端点姿勢のみ補正）。
    """
    # 完全再計算区間（仮想/逆走を含む）
    for start, end in recalc_ranges:
        if start < 0 or end >= len(wps) or start >= end:
            continue
        for i in range(start, end):
            dx = wps[i+1].pose.position.x - wps[i].pose.position.x
            dy = wps[i+1].pose.position.y - wps[i].pose.position.y
            yaw = math.atan2(dy, dx)
            wps[i].pose.orientation = yaw_to_quaternion(yaw)
        dx = wps[end].pose.position.x - wps[end-1].pose.position.x
        dy = wps[end].pose.position.y - wps[end-1].pose.position.y
        yaw = math.atan2(dy, dx)
        wps[end].pose.orientation = yaw_to_quaternion(yaw)

    # ブロック末尾waypoint（端点のみ）
    for idx in block_boundaries:
        if 0 < idx < len(wps):
            dx = wps[idx].pose.position.x - wps[idx-1].pose.position.x
            dy = wps[idx].pose.position.y - wps[idx-1].pose.position.y
            yaw = math.atan2(dy, dx)
            wps[idx].pose.orientation = yaw_to_quaternion(yaw)

    # ルート全体の末尾（端点のみ）
    if len(wps) > 1:
        dx = wps[-1].pose.position.x - wps[-2].pose.position.x
        dy = wps[-1].pose.position.y - wps[-2].pose.position.y
        yaw = math.atan2(dy, dx)
        wps[-1].pose.orientation = yaw_to_quaternion(yaw)


def pack_route_msg(
    wps: List[Waypoint],
    version: int,
    total_distance: float,
    route_image: Optional[Image],
) -> Route:
    """Route.msg を組み立てる。必要に応じて route_image はテキストPNGプレースホルダを入れる。"""
    msg = Route()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.waypoints = wps
    msg.version = int(version)
    msg.total_distance = float(total_distance)
    msg.route_image = route_image if route_image is not None else make_text_png_image()
    return msg


# ===== RoutePlanner ノード ============================================================

class RoutePlannerNode(Node):
    """ルート配布サーバ（GetRoute 完了 + UpdateRoute 完了：仮想エッジ/履歴対応）。"""

    def __init__(self) -> None:
        super().__init__("route_planner")

        # --- 起動時パラメータ ---
        self.declare_parameter("config_yaml_path", "")
        self.declare_parameter("csv_base_dir", "")

        self.config_yaml_path: str = self.get_parameter("config_yaml_path").get_parameter_value().string_value
        self.csv_base_dir: str = self.get_parameter("csv_base_dir").get_parameter_value().string_value

        if not self.config_yaml_path:
            self.get_logger().error("config_yaml_path is required.")
        else:
            self.get_logger().info(f"Using config_yaml_path: {self.config_yaml_path}")

        # --- メンバ（状態） ---
        self.blocks: List[Dict[str, Any]] = []                  # YAMLのブロック原義（固定/可変）
        self.segments: Dict[str, SegmentCacheEntry] = {}        # segment_id -> キャッシュ済みwaypoints
        self.current_route: Optional[Route] = None
        self.current_route_origins: List[WaypointOrigin] = []   # current_route.waypoints と同長
        self.route_version: int = 0                             # Route.msgのversion(int32)に対応
        self.current_block_name: Optional[str] = None           # 累積封鎖の対象ブロック名
        self.closed_edges: Set[frozenset] = set()               # {frozenset({u,v}), }
        self.last_request_checkpoints: Set[str] = set()         # GetRoute時に追加されたチェックポイント
        self.visited_checkpoints_hist: Dict[str, Set[str]] = {} # ブロック名 -> 訪問済みチェックポイント（永続）

        # --- ロード処理 ---
        self._load_blocks_from_yaml()
        self._load_csv_segments()

        # --- サービス登録（逐次処理: MutuallyExclusive） ---
        cb_group = MutuallyExclusiveCallbackGroup()
        self._srv_get = self.create_service(
            GetRoute, "/get_route", self.handle_get_route, callback_group=cb_group
        )
        self._srv_update = self.create_service(
            UpdateRoute, "/update_route", self.handle_update_route, callback_group=cb_group
        )

        self.get_logger().info("route_planner is ready.")

    # ===== YAML/CSV ロード ============================================================

    def _load_blocks_from_yaml(self) -> None:
        """YAMLファイルを読み込み、self.blocks を初期化する。"""
        if not self.config_yaml_path:
            self.get_logger().warn("No config_yaml_path specified.")
            return
        try:
            with open(self.config_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            self.blocks = []
            return

        # blocks の取得（route_planner/ros__parameters/blocks か、root直下blocks の両対応）
        blocks = None
        if isinstance(data, dict):
            rp = data.get("route_planner", {})
            if isinstance(rp, dict):
                params = rp.get("ros__parameters", {})
                if isinstance(params, dict):
                    blocks = params.get("blocks")
            if blocks is None:
                blocks = data.get("blocks")
        if not isinstance(blocks, list):
            self.get_logger().warn("No valid 'blocks' list found in YAML.")
            self.blocks = []
            return

        # csv_base_dir が未指定なら、YAMLのあるディレクトリを基準にする
        if not self.csv_base_dir:
            self.csv_base_dir = os.path.dirname(os.path.abspath(self.config_yaml_path))

        # ブロックを正規化しつつ格納
        norm_blocks: List[Dict[str, Any]] = []
        for i, b in enumerate(blocks):
            if not isinstance(b, dict):
                self.get_logger().error(f"Block[{i}] is not a dict.")
                continue
            btype = b.get("type")
            name = b.get("name", f"block_{i}")
            if btype == "fixed":
                seg = b.get("segment_id")
                if not seg:
                    self.get_logger().error(f"Fixed block '{name}' missing 'segment_id'.")
                    continue
                norm_blocks.append({
                    "type": "fixed",
                    "name": name,
                    "segment_id": seg,
                    "index": len(norm_blocks),
                })
            elif btype == "variable":
                nodes_file = b.get("nodes_file")
                edges_file = b.get("edges_file")
                start = b.get("start")
                goal = b.get("goal")
                checkpoints = b.get("checkpoints", [])
                # 必須項目チェック
                missing = [k for k, v in [
                    ("nodes_file", nodes_file),
                    ("edges_file", edges_file),
                    ("start", start),
                    ("goal", goal),
                ] if not v]
                if missing:
                    self.get_logger().error(f"Variable block '{name}' missing fields: {missing}")
                    continue
                # checkpoints は >=1 必須
                if not isinstance(checkpoints, list) or len(checkpoints) < 1:
                    self.get_logger().error(f"Variable block '{name}' requires >=1 checkpoints.")
                    continue
                norm_blocks.append({
                    "type": "variable",
                    "name": name,
                    "nodes_file": nodes_file,
                    "edges_file": edges_file,
                    "start": start,
                    "goal": goal,
                    "checkpoints": checkpoints,
                    "nodes": None,   # List[Dict{id,lat,lon}]
                    "edges": None,   # List[Dict{source,target,waypoint_list,reversible}]
                    "index": len(norm_blocks),
                })
            else:
                self.get_logger().error(f"Unknown block type: {btype}")
                continue

        self.blocks = norm_blocks

    def _load_csv_segments(self) -> None:
        """固定ブロックと可変ブロックの CSV をロードし、キャッシュや原義を整える。"""
        if not self.blocks:
            return

        for b in self.blocks:
            btype = b["type"]
            bname = b["name"]

            if btype == "fixed":
                # 固定CSVをキャッシュ
                seg_path = resolve_path(self.csv_base_dir, b["segment_id"])
                try:
                    wps = parse_waypoint_csv(seg_path)
                    self.segments[b["segment_id"]] = SegmentCacheEntry(b["segment_id"], wps)
                except Exception as e:
                    self.get_logger().error(f"[fixed:{bname}] failed to load segment '{seg_path}': {e}")
                    continue

            elif btype == "variable":
                # nodes.csv をロード（id,lat,lon）
                nodes_file = resolve_path(self.csv_base_dir, b["nodes_file"])
                nodes: List[Dict[str, Any]] = []
                try:
                    with open(nodes_file, newline="", encoding="utf-8") as f:
                        reader = csv.DictReader(f)
                        for row in reader:
                            nid = row.get("id")
                            if nid is None or nid == "":
                                raise ValueError("nodes.csv: 'id' is required.")
                            try:
                                lat = float(row.get("lat"))
                                lon = float(row.get("lon"))
                            except Exception:
                                raise ValueError("nodes.csv: 'lat'/'lon' must be float.")
                            nodes.append({"id": str(nid), "lat": lat, "lon": lon})
                    b["nodes"] = nodes
                except Exception as e:
                    self.get_logger().error(f"[variable:{bname}] failed to load nodes '{nodes_file}': {e}")
                    b["nodes"] = []
                    continue

                # edges.csv をロード（source,target,waypoint_list,reversible）
                edges_file = resolve_path(self.csv_base_dir, b["edges_file"])
                edges: List[Dict[str, Any]] = []
                try:
                    with open(edges_file, newline="", encoding="utf-8") as f:
                        reader = csv.DictReader(f)
                        for i, row in enumerate(reader):
                            source = row.get("source")
                            target = row.get("target")
                            seg_rel = row.get("waypoint_list")
                            rev_raw = row.get("reversible", "1")

                            if not source or not target or not seg_rel:
                                raise ValueError(f"edges.csv row {i+1}: source/target/waypoint_list are required.")
                            try:
                                reversible = int(str(rev_raw))
                            except Exception:
                                raise ValueError(f"edges.csv row {i+1}: reversible must be 0 or 1.")
                            if reversible not in (0, 1):
                                raise ValueError(f"edges.csv row {i+1}: reversible must be 0 or 1.")

                            edge = {
                                "source": str(source),
                                "target": str(target),
                                "waypoint_list": seg_rel,  # 相対のまま保持（キャッシュは別）
                                "reversible": reversible,
                            }
                            edges.append(edge)

                            # waypoint CSV をキャッシュ
                            seg_path = resolve_path(self.csv_base_dir, seg_rel)
                            try:
                                if seg_rel not in self.segments:
                                    wps = parse_waypoint_csv(seg_path)
                                    self.segments[seg_rel] = SegmentCacheEntry(seg_rel, wps)
                            except Exception as e:
                                raise RuntimeError(f"failed to load segment '{seg_path}': {e}")

                    b["edges"] = edges
                except Exception as e:
                    self.get_logger().error(f"[variable:{bname}] failed to load edges '{edges_file}': {e}")
                    b["edges"] = []
                    continue

            else:
                continue

    # ===== GetRoute / UpdateRoute =====================================================

    def handle_get_route(self, request: GetRoute.Request, response: GetRoute.Response) -> GetRoute.Response:
        """初期ルートの生成と配布。Update 用の由来メタも同時に構築して保存する。"""
        try:
            # --- サービスリクエストのフィールド名を設計書仕様に統一 ---
            start_label = getattr(request, "start_label", None)
            goal_label = getattr(request, "goal_label", None)
            checkpoint_labels = getattr(request, "checkpoint_labels", [])
            last_route_image: Optional[Image] = None
            self.get_logger().info(f"Get route: start_label='{start_label}', goal_label='{goal_label}', checkpoints='{checkpoint_labels}'")
            if not self.blocks:
                raise RuntimeError("No blocks configuration loaded.")

            # ルート構築の作業配列
            route_wps: List[Waypoint] = []
            origins: List[WaypointOrigin] = []
            reversed_ranges: List[Tuple[int, int]] = []   # 姿勢補正の完全再計算対象（逆走区間）
            block_tail_indices: List[int] = []            # ブロック末尾（端点補正用）
            has_variable: bool = False

            # YAML定義のブロック順に連結していく
            for block in self.blocks:
                btype = block["type"]
                bname = block["name"]
                bindex = block["index"]

                if btype == "fixed":
                    seg_id = block["segment_id"]
                    entry = self.segments.get(seg_id)
                    if entry is None:
                        raise RuntimeError(f"[fixed:{bname}] segment not loaded: {seg_id}")

                    # 連結（重複境界は除去）
                    before_len = len(route_wps)
                    route_wps = concat_with_dedup(route_wps, entry.waypoints)
                    # origin を付与（固定: edge_u/v なし、segment_id は固定CSV）
                    added = len(route_wps) - before_len
                    for _ in range(added):
                        origins.append(WaypointOrigin(
                            block_name=bname,
                            block_index=bindex,
                            segment_id=seg_id,
                            edge_u=None,
                            edge_v=None,
                            index_in_edge=None,
                            u_first=None,
                        ))
                    block_tail_indices.append(len(route_wps) - 1)

                elif btype == "variable":
                    has_variable = True
                    # YAMLチェックポイントとリクエスト追加チェックポイントを順序保持で集合和（重複は除去）
                    merged_cps = list(dict.fromkeys((block.get("checkpoints", []) or []) + list(request.checkpoint_labels)))

                    # 可変ルート探索を実行
                    nodes = block.get("nodes") or []
                    edges = block.get("edges") or []
                    start = block.get("start")
                    goal = block.get("goal")
                    if not nodes or not edges:
                        raise RuntimeError(f"[variable:{bname}] nodes/edges not loaded.")
                    if not start or not goal:
                        raise RuntimeError(f"[variable:{bname}] start/goal not set.")

                    solve_result = solve_variable_route(nodes=nodes, edges=edges, start=start, goal=goal, checkpoints=merged_cps)
                    edge_seq: List[Dict[str, Any]] = solve_result.get("edge_sequence", [])
                    if not edge_seq:
                        raise RuntimeError(f"[variable:{bname}] solver returned empty edge_sequence.")

                    # route_image 読み込みを試行
                    img = load_route_image_from_solver(solve_result)
                    if img is not None:
                        last_route_image = img  # 最新の可変ブロック画像を保持

                    # edge_sequence を辿ってセグメントを連結
                    for e in edge_seq:
                        seg_id = e["segment_id"]
                        direction = e["direction"]  # "forward" | "reverse"
                        src = e["source"]
                        dst = e["target"]
                        entry = self.segments.get(seg_id)
                        if entry is None:
                            raise RuntimeError(f"[variable:{bname}] segment not loaded: {seg_id}")

                        # U→V 向きに合わせるか判定（direction == "forward" を U→V と定義）
                        u_first = True if direction == "forward" else False
                        seg_wps = entry.waypoints if u_first else list(reversed(entry.waypoints))
                        stamp_edge_end_labels(seg_wps, src_label=src, dst_label=dst)

                        # 連結しつつ、origin を並行して構築
                        start_idx = len(route_wps)
                        route_wps = concat_with_dedup(route_wps, seg_wps)
                        end_idx = len(route_wps) - 1

                        # origin 付与（可変: edge_u/v と index_in_edge を設定。u_first も記録。）
                        local_len = end_idx - start_idx + 1
                        for i_local in range(local_len):
                            origins.append(WaypointOrigin(
                                block_name=bname,
                                block_index=bindex,
                                segment_id=seg_id,
                                edge_u=src,
                                edge_v=dst,
                                index_in_edge=i_local,
                                u_first=u_first,
                            ))

                        if not u_first:
                            # 逆走（V→U）で採用された区間は完全再計算リストに入れる
                            reversed_ranges.append((start_idx, end_idx))

                    block_tail_indices.append(len(route_wps) - 1)

                else:
                    self.get_logger().warn(f"Unknown block type: {btype}")

            if not route_wps:
                raise RuntimeError("No waypoints generated from blocks configuration.")

            # start/goal でスライス（スライス開始オフセットを取得）
            sliced, start_offset = slice_by_labels(route_wps, request.start_label, request.goal_label)
            # origins も同じ範囲に揃える
            origins = origins[start_offset:start_offset + len(sliced)]
            # インデックスを0..N-1で再採番
            indexing(sliced)
            # 総距離を算出
            total_distance = calc_total_distance(sliced)
            # スライス後のインデックス系に合わせて補正
            def _shift_inside(r: Tuple[int, int], offset: int, n: int) -> Optional[Tuple[int, int]]:
                s, e = r[0] - offset, r[1] - offset
                if e < 0 or s >= n:
                    return None
                s = max(s, 0)
                e = min(e, n - 1)
                if s >= e:
                    return None
                return (s, e)

            recalc_ranges: List[Tuple[int, int]] = []
            for r in reversed_ranges:
                rr = _shift_inside(r, start_offset, len(sliced))
                if rr is not None:
                    recalc_ranges.append(rr)

            shifted_block_tails: List[int] = []
            for idx in block_tail_indices:
                j = idx - start_offset
                if 0 <= j < len(sliced):
                    shifted_block_tails.append(j)

            # 姿勢補正（逆走区間 + ブロック末尾 + ルート末尾）
            adjust_orientations(sliced, recalc_ranges, shifted_block_tails)

            # 画像を選択（可変あり & 読み込み成功時はsolver画像）
            if has_variable and last_route_image is not None:
                route_image = last_route_image
            elif has_variable:
                route_image = make_text_png_image("variable part image (placeholder)")
            else:
                route_image = make_text_png_image("No variable route in this plan")

            # バージョン更新・応答
            self.route_version = 1
            route = pack_route_msg(sliced, self.route_version, total_distance, route_image)

            # 状態を保持（UpdateRoute が参照）
            self.current_route = route
            self.current_route_origins = origins
            self.last_request_checkpoints = set(request.checkpoint_labels)
            self.visited_checkpoints_hist.clear()  # 初期ルート生成時に訪問履歴をリセット

            response.success = True
            response.message = ""
            response.route = route
            return response

        except Exception as e:
            self.get_logger().error(f"[GetRoute] {e}\n{traceback.format_exc()}")
            response.success = False
            response.message = str(e)
            response.route = Route()
            return response

    def handle_update_route(self, request: UpdateRoute.Request, response: UpdateRoute.Response) -> UpdateRoute.Response:
        """経路封鎖リルートの実行。仮想エッジ（current→prev→U）を先頭に挿入し、その後に新可変ルートと後続ブロックを連結する。"""
        try:
            # 0) 前提検証
            if self.current_route is None or not self.current_route.waypoints:
                response.success = False
                response.message = "No current route in server."
                response.route = Route()
                return response
            if request.route_version != self.current_route.version:
                self.get_logger().error("[UpdateRoute] Route version mismatch.")
                response.success = False
                response.message = "Route version mismatch."
                response.route = self.current_route or Route()
                return response

            wps = self.current_route.waypoints
            origins = self.current_route_origins
            n = len(wps)

            # 1) prev/next の正当性検証（現ルート上で隣接）
            if not (0 <= request.prev_index < n - 1):
                raise RuntimeError("Invalid prev_index.")
            if request.next_index != request.prev_index + 1:
                raise RuntimeError("prev/next must be adjacent (next_index = prev_index + 1).")
            if wps[request.prev_index].label != request.prev_wp_label:
                raise RuntimeError("prev_wp_label mismatch current route.")
            if wps[request.next_index].label != request.next_wp_label:
                raise RuntimeError("next_wp_label mismatch current route.")

            prev_origin = origins[request.prev_index]
            next_origin = origins[request.next_index]

            # 2) 対象ブロックを特定（同一ブロックである必要あり）
            if prev_origin.block_name != next_origin.block_name:
                raise RuntimeError("prev/next belong to different blocks.")
            block_name = prev_origin.block_name
            block_idx = prev_origin.block_index

            # 対象ブロックを取得
            block = None
            for b in self.blocks:
                if b["name"] == block_name and b["index"] == block_idx:
                    block = b
                    break
            if block is None:
                raise RuntimeError(f"Block not found: {block_name}")
            if block["type"] == "fixed":
                # 固定ブロックはリルート不可（封鎖検知時はエラー返却。ただしノードは継続）
                self.get_logger().warn("Closure in fixed block (not reroutable).")
                response.success = False
                response.message = "Closure in fixed block (not reroutable)."
                response.route = self.current_route or Route()
                return response

            # 3) 現在走行中エッジの (U,V) と、prev のエッジ内位置を特定
            u_node = prev_origin.edge_u
            v_node = prev_origin.edge_v
            seg_id_running = prev_origin.segment_id
            u_first_flag = prev_origin.u_first
            local_idx_prev = prev_origin.index_in_edge
            if not (u_node and v_node and seg_id_running is not None and u_first_flag is not None and local_idx_prev is not None):
                # 可変ブロックなのに必要メタが欠けるのは異常
                raise RuntimeError("Failed to identify running edge metadata for closure.")

            # 4) 封鎖累積の管理（同ブロック内は追加／別ブロックならリセット）
            if self.current_block_name != block_name:
                self.closed_edges = set()
                self.current_block_name = block_name
            # 片方向通行可能は考えないため {U,V} を丸ごと封鎖
            self.closed_edges.add(frozenset({u_node, v_node}))

            # 5) グラフ = 原義 - 累積封鎖
            nodes, edges = self._build_graph_with_closures(block_name)

            # 6) 未通過チェックポイント集合（永続履歴を考慮）
            yaml_cps = set(block.get("checkpoints", []))
            req_cps = set(self.last_request_checkpoints)  # GetRoute で追加された分
            # このブロックの nodes に存在するものだけを有効チェックポイントとする
            node_ids = {nd["id"] for nd in (block.get("nodes") or [])}
            required = {c for c in (yaml_cps | req_cps) if c in node_ids}

            # 永続履歴（これまでの封鎖を跨いでも消えない）
            hist = self.visited_checkpoints_hist.setdefault(block_name, set())
            # 今回の current_route で prev までに通過済みを追加
            visited_now: Set[str] = set()
            for i in range(0, request.prev_index + 1):
                lab = wps[i].label
                if lab in required:
                    visited_now.add(lab)
            hist |= visited_now
            remaining_cps = list(required - hist)

            # 7) 仮想ノード/エッジは solver に __virtual__ を流さず、
            #    代わりに「current→prev→U」の waypoint 群を先頭に自前で連結する。
            #    solver の start は U、goal は block.goal。
            goal = block.get("goal")
            if not goal:
                raise RuntimeError("Block goal not set.")

            # 7-1) 先頭: current（今回の現在位置）
            new_wps: List[Waypoint] = []
            new_origins: List[WaypointOrigin] = []
            new_wps.append(self._make_waypoint_from_pose(request.current_pose, label="current"))
            new_origins.append(WaypointOrigin(
                block_name=block_name,
                block_index=block_idx,
                segment_id=None,
                edge_u=None,
                edge_v=None,
                index_in_edge=None,
                u_first=None,
            ))

            # 7-2) 仮想エッジ: current→prev→U の waypoint 群を生成して連結
            virtual_wps = self._make_virtual_edge_waypoints(
                seg_id=seg_id_running,
                u_label=u_node,
                local_idx_prev=local_idx_prev,
                u_first_on_route=u_first_flag,
                prev_wp=wps[request.prev_index],
                current_pose=request.current_pose,
            )
            # 端点ラベル（Uラベル）を刻印（virtual_wps の末尾は U になる）
            if virtual_wps:
                stamp_edge_end_labels(virtual_wps, src_label="current", dst_label=u_node)

            # 追加（重複境界は concat 内で解消される）
            start_idx_virtual = len(new_wps)
            new_wps = concat_with_dedup(new_wps, virtual_wps)
            end_idx_virtual = len(new_wps) - 1
            # origin は "仮想エッジ" として記録（segment_id="__virtual__"相当）
            for i_local in range(end_idx_virtual - start_idx_virtual + 1):
                new_origins.append(WaypointOrigin(
                    block_name=block_name,
                    block_index=block_idx,
                    segment_id="__virtual__",
                    edge_u=u_node,
                    edge_v=v_node,
                    index_in_edge=i_local,
                    u_first=True,  # virtual は current→U の向き（U側へ向かう）としてUファースト扱いで良い
                ))

            # 仮想区間は「姿勢完全再計算区間」に追加
            recalc_ranges: List[Tuple[int, int]] = []
            if end_idx_virtual > start_idx_virtual:
                recalc_ranges.append((start_idx_virtual, end_idx_virtual))

            block_tail_indices: List[int] = []  # 末尾補正用

            # 8) solver 実行（start=U, goal=block.goal, checkpoints=remaining_cps）
            result = solve_variable_route(nodes=nodes, edges=edges, start=u_node, goal=goal, checkpoints=remaining_cps)
            edge_seq: List[Dict[str, Any]] = result.get("edge_sequence", [])
            if not edge_seq:
                raise RuntimeError("No route found after applying closures.")
            # 画像取得
            img_solver = load_route_image_from_solver(result)

            # 9) 可変結果を連結（__virtual__ は solver から基本返さない前提。返ってきても無視）
            for e in edge_seq:
                if e.get("segment_id") == "__virtual__":
                    # 念のためスキップ（通常は発生しない想定）
                    continue
                seg_id2 = e["segment_id"]
                direction2 = e["direction"]  # "forward" | "reverse"
                src2 = e["source"]
                dst2 = e["target"]
                entry2 = self.segments.get(seg_id2)
                if entry2 is None:
                    raise RuntimeError(f"Segment not loaded: {seg_id2}")

                u_first2 = True if direction2 == "forward" else False
                seg_wps2 = entry2.waypoints if u_first2 else list(reversed(entry2.waypoints))
                stamp_edge_end_labels(seg_wps2, src_label=src2, dst_label=dst2)

                start_idx2 = len(new_wps)
                new_wps = concat_with_dedup(new_wps, seg_wps2)
                end_idx2 = len(new_wps) - 1

                # origin 付与
                for i_local in range(end_idx2 - start_idx2 + 1):
                    new_origins.append(WaypointOrigin(
                        block_name=block_name,
                        block_index=block_idx,
                        segment_id=seg_id2,
                        edge_u=src2,
                        edge_v=dst2,
                        index_in_edge=i_local,
                        u_first=u_first2,
                    ))

                # 逆走区間は姿勢完全再計算に追加
                if not u_first2 and end_idx2 > start_idx2:
                    recalc_ranges.append((start_idx2, end_idx2))

            # ブロック末尾を記録（可変ブロックの末尾）
            block_tail_indices.append(len(new_wps) - 1)

            # 10) 後続の未走行ブロックを定義どおり連結（固定はCSV、可変は再探索）
            for b in self.blocks:
                if b["index"] <= block_idx:
                    continue
                bname2 = b["name"]
                bidx2 = b["index"]
                if b["type"] == "fixed":
                    seg_id3 = b["segment_id"]
                    entry3 = self.segments.get(seg_id3)
                    if entry3 is None:
                        raise RuntimeError(f"[fixed:{bname2}] segment not loaded: {seg_id3}")
                    before = len(new_wps)
                    new_wps = concat_with_dedup(new_wps, entry3.waypoints)
                    for _ in range(len(new_wps) - before):
                        new_origins.append(WaypointOrigin(
                            block_name=bname2,
                            block_index=bidx2,
                            segment_id=seg_id3,
                            edge_u=None,
                            edge_v=None,
                            index_in_edge=None,
                            u_first=None,
                        ))
                    block_tail_indices.append(len(new_wps) - 1)
                elif b["type"] == "variable":
                    nodes2 = b.get("nodes") or []
                    edges2 = b.get("edges") or []
                    start2 = b.get("start")
                    goal2 = b.get("goal")
                    if not nodes2 or not edges2 or not start2 or not goal2:
                        raise RuntimeError(f"[variable:{bname2}] definition incomplete.")

                    merged_cps2 = list(dict.fromkeys((b.get("checkpoints", []) or []) + list(self.last_request_checkpoints)))
                    node_ids2 = {nd["id"] for nd in nodes2}
                    cps2 = [c for c in merged_cps2 if c in node_ids2]

                    res2 = solve_variable_route(nodes=nodes2, edges=edges2, start=start2, goal=goal2, checkpoints=cps2)
                    img_extra = load_route_image_from_solver(res2)
                    if img_extra is not None and img_solver is None:
                        img_solver = img_extra
                    es2: List[Dict[str, Any]] = res2.get("edge_sequence", [])
                    if not es2:
                        raise RuntimeError(f"[variable:{bname2}] solver returned empty edge_sequence.")
                    for e2 in es2:
                        seg_id4 = e2["segment_id"]
                        direction4 = e2["direction"]
                        src4 = e2["source"]
                        dst4 = e2["target"]
                        entry4 = self.segments.get(seg_id4)
                        if entry4 is None:
                            raise RuntimeError(f"[variable:{bname2}] segment not loaded: {seg_id4}")
                        u_first4 = True if direction4 == "forward" else False
                        seg_wps4 = entry4.waypoints if u_first4 else list(reversed(entry4.waypoints))
                        stamp_edge_end_labels(seg_wps4, src_label=src4, dst_label=dst4)

                        start_idx4 = len(new_wps)
                        new_wps = concat_with_dedup(new_wps, seg_wps4)
                        end_idx4 = len(new_wps) - 1

                        for i_local in range(end_idx4 - start_idx4 + 1):
                            new_origins.append(WaypointOrigin(
                                block_name=bname2,
                                block_index=bidx2,
                                segment_id=seg_id4,
                                edge_u=src4,
                                edge_v=dst4,
                                index_in_edge=i_local,
                                u_first=u_first4,
                            ))

                        if not u_first4 and end_idx4 > start_idx4:
                            recalc_ranges.append((start_idx4, end_idx4))

                    block_tail_indices.append(len(new_wps) - 1)

            # 11) finalize（姿勢補正・採番・距離・画像・version++）
            adjust_orientations(new_wps, recalc_ranges, block_tail_indices)
            indexing(new_wps)
            total_distance = calc_total_distance(new_wps)
            route_image = img_solver if img_solver is not None else make_text_png_image("variable part image (placeholder)")
            self.route_version += 1
            new_route = pack_route_msg(new_wps, self.route_version, total_distance, route_image)

            # 状態更新
            self.current_route = new_route
            self.current_route_origins = new_origins
            # visited 履歴は維持（本ブロックのキーに対して既に更新済み）

            response.success = True
            response.message = ""
            response.route = new_route
            return response

        except Exception as e:
            self.get_logger().error(f"[UpdateRoute] {e}\n{traceback.format_exc()}")
            response.success = False
            response.message = str(e)
            response.route = Route()
            return response

    # ===== 内部補助 =====================================================

    def _build_graph_with_closures(self, block_name: str) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        """可変ブロックの原義から累積封鎖を反映した nodes/edges を構築する。"""
        # 対象ブロックを取得
        block = None
        for b in self.blocks:
            if b["name"] == block_name:
                block = b
                break
        if block is None or block["type"] != "variable":
            raise RuntimeError("Block not found or not variable.")

        nodes = list(block.get("nodes") or [])
        raw_edges = list(block.get("edges") or [])
        # 累積封鎖の適用：{u,v} が self.closed_edges にあれば、その行は除外
        filt_edges: List[Dict[str, Any]] = []
        for e in raw_edges:
            u = str(e["source"])
            v = str(e["target"])
            if frozenset({u, v}) in self.closed_edges:
                continue
            filt_edges.append(dict(e))  # シャローコピー

        return nodes, filt_edges

    def _make_waypoint_from_pose(self, pose_stamped: PoseStamped, label: str) -> Waypoint:
        """PoseStamped から Waypoint を作る簡易ヘルパ。index は後段で採番し直す。"""
        wp = Waypoint()
        wp.label = label
        wp.index = 0
        wp.pose = _copy_pose(pose_stamped.pose)
        return wp

    def _make_virtual_edge_waypoints(
        self,
        seg_id: str,
        u_label: str,
        local_idx_prev: int,
        u_first_on_route: bool,
        prev_wp: Waypoint,
        current_pose: PoseStamped,
    ) -> List[Waypoint]:
        """仮想エッジ current→prev→U の waypoint 群を生成する。

        ロジック:
          - 走行中エッジ（seg_id）の CSV を取得し、"Uが先頭" となる向き（u_first）で配列を準備する。
            * GetRoute時に記録した u_first_on_route を使って、prev のローカルindexを u_first 基準のindexに変換する。
          - current（Pose）を先頭に、prev（既存Waypoint）を挟み、prev→U 方向に向かう配列を切り出す。
            * prev が既に U（= index 0）なら、[current, U] の最短列になる（処理は同一）。
          - 生成した配列は、後段で stamp_edge_end_labels("current", U) で端点ラベルを刻む。

        Args:
            seg_id: 走行中エッジの segment_id（CSVを特定）。
            u_label: 手前側ノードのラベル（U）。
            local_idx_prev: prev のエッジ内ローカルindex（GetRoute時の向き基準）。
            u_first_on_route: GetRoute時にそのエッジを U→V 向きで使ったかどうかのフラグ。
            prev_wp: 現ルート上の prev Waypoint（座標/姿勢を持つ）。
            current_pose: 現在位置の PoseStamped。

        Returns:
            仮想エッジ（current→prev→U）に相当する Waypoint 配列。
        """
        entry = self.segments.get(seg_id)
        if entry is None:
            raise RuntimeError(f"Virtual edge source segment not loaded: {seg_id}")

        base = entry.waypoints  # CSV由来の素の配列（端点ラベルは未刻印）

        # "Uが先頭" となる向きの配列を作る（u_first=True の配列）
        # GetRoute時の向きが U→V (u_first_on_route=True) なら、そのまま。
        # 逆なら反転して U→V 向きに合わせる。
        if u_first_on_route:
            seg_u_first = list(base)
            # prev のindexはそのまま
            prev_idx_u_first = int(local_idx_prev)
        else:
            seg_u_first = list(reversed(base))
            # 反転したため prev の index は変換（len-1 - idx）
            prev_idx_u_first = (len(seg_u_first) - 1) - int(local_idx_prev)

        if not (0 <= prev_idx_u_first < len(seg_u_first)):
            raise RuntimeError("Invalid prev index in virtual edge generation.")

        # Uは seg_u_first[0] に対応する（末尾は V）
        # prev→U 方向に向かう配列は seg_u_first[0:prev_idx_u_first+1] を逆順にせず、
        # current→prev→  →U となるように、current, prev, seg_u_first[prev_idx_u_first-1  0] を連結する。
        virtual: List[Waypoint] = []
        # current を先頭に追加（Waypoint化）
        current_wp = self._make_waypoint_from_pose(current_pose, label="current")
        virtual.append(current_wp)

        # prev を続けて追加（既存prevをコピーして先頭重複回避。位置はそのまま）
        prev_copy = Waypoint()
        prev_copy.label = prev_wp.label  # prev は中間なのでlabelは通常空/任意だが、そのまま踏襲
        prev_copy.index = 0
        prev_copy.pose = _copy_pose(prev_wp.pose)
        if hasattr(prev_copy, "right_open"):
            prev_copy.right_open = float(
                getattr(prev_wp, "right_open", getattr(prev_wp, "right_is_open", 0.0))
            )
        if hasattr(prev_copy, "left_open"):
            prev_copy.left_open = float(
                getattr(prev_wp, "left_open", getattr(prev_wp, "left_is_open", 0.0))
            )
        if hasattr(prev_copy, "line_stop"):
            prev_copy.line_stop = bool(
                getattr(prev_wp, "line_stop", getattr(prev_wp, "line_is_stop", False))
            )
        if hasattr(prev_copy, "signal_stop"):
            prev_copy.signal_stop = bool(
                getattr(prev_wp, "signal_stop", getattr(prev_wp, "signal_is_stop", False))
            )
        if hasattr(prev_copy, "not_skip"):
            prev_copy.not_skip = bool(
                getattr(prev_wp, "not_skip", getattr(prev_wp, "isnot_skipnum", False))
            )
        virtual.append(prev_copy)

        # prev_idx_u_first から 1 ずつデクリメントして U=0 まで辿る
        # prev が U と同一（prev_idx_u_first==0）のときは追加なし（[current, prev(=U)]のみ）
        for idx in range(prev_idx_u_first - 1, -1, -1):
            src = seg_u_first[idx]
            # コピーして追加（オリジナルを汚さない）
            wp = Waypoint()
            wp.label = src.label
            wp.index = 0
            wp.pose = _copy_pose(src.pose)
            if hasattr(wp, "right_open"):
                wp.right_open = float(
                    getattr(src, "right_open", getattr(src, "right_is_open", 0.0))
                )
            if hasattr(wp, "left_open"):
                wp.left_open = float(
                    getattr(src, "left_open", getattr(src, "left_is_open", 0.0))
                )
            if hasattr(wp, "line_stop"):
                wp.line_stop = bool(
                    getattr(src, "line_stop", getattr(src, "line_is_stop", False))
                )
            if hasattr(wp, "signal_stop"):
                wp.signal_stop = bool(
                    getattr(src, "signal_stop", getattr(src, "signal_is_stop", False))
                )
            if hasattr(wp, "not_skip"):
                wp.not_skip = bool(
                    getattr(src, "not_skip", getattr(src, "isnot_skipnum", False))
                )
            virtual.append(wp)

        return virtual


# ===== エントリポイント ===============================================================

def main(argv: Optional[List[str]] = None) -> None:
    """ノードのエントリポイント。"""
    rclpy.init(args=argv)
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)