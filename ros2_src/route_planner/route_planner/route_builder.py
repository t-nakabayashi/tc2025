#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ルート定義YAMLからwaypoint列を生成するユーティリティ関数群."""

from __future__ import annotations

import copy
import csv
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

import yaml

if __package__ in (None, ""):
    # 単体実行時はパッケージ相対インポートが利用できないため、実行ファイル直下をパスへ追加する。
    PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
    if PACKAGE_DIR not in sys.path:
        sys.path.insert(0, PACKAGE_DIR)
    from graph_solver import render_route_on_map, solve_variable_route
else:
    from .graph_solver import render_route_on_map, solve_variable_route


def parse_weight_factor(value: Any) -> float:
    """edges.csv の重み係数を正の実数として解釈する。"""

    if value is None or str(value).strip() == "":
        return 1.0

    try:
        factor = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("edges.csv: weight_factor は正の数値で指定してください。") from exc

    if factor <= 0:
        raise ValueError("edges.csv: weight_factor は正の数値で指定してください。")

    return factor


@dataclass
class PositionRecord:
    """位置座標を保持するシンプルなデータクラス."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class OrientationRecord:
    """姿勢（クォータニオン）を保持するシンプルなデータクラス."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class PoseRecord:
    """位置と姿勢をまとめたデータクラス."""

    position: PositionRecord = field(default_factory=PositionRecord)
    orientation: OrientationRecord = field(default_factory=OrientationRecord)


@dataclass
class WaypointRecord:
    """ROSメッセージを使わずにwaypoint情報を保持するデータクラス."""

    label: str = ""
    index: int = 0
    pose: PoseRecord = field(default_factory=PoseRecord)
    right_open: float = 0.0
    left_open: float = 0.0
    line_stop: bool = False
    signal_stop: bool = False
    not_skip: bool = False
    segment_is_fixed: bool = False
    latitude: Optional[float] = None
    longitude: Optional[float] = None

    def clone(self) -> "WaypointRecord":
        """ディープコピーを生成する."""

        return copy.deepcopy(self)


@dataclass
class WaypointOrigin:
    """waypointの由来を記録するデータクラス."""

    block_name: str
    block_index: int
    segment_id: Optional[str]
    edge_u: Optional[str]
    edge_v: Optional[str]
    index_in_edge: Optional[int]
    u_first: Optional[bool]


@dataclass
class GraphEdgeRecord:
    """可変ブロックのエッジを表現するシンプルなデータ構造."""

    source: str
    target: str
    segment_id: str
    reversible: int = 0
    weight_factor: float = 1.0

    def key(self) -> frozenset:
        """エッジ対（無向）を一意に表すキーを返す。"""

        return frozenset({self.source, self.target})

    def to_dict(self) -> Dict[str, Any]:
        """RouteBuilder内部で扱う辞書形式へ変換する。"""

        return {
            "source": self.source,
            "target": self.target,
            "segment_id": self.segment_id,
            "waypoint_list": self.segment_id,
            "reversible": int(self.reversible),
            "weight_factor": self.weight_factor,
        }


class VariableGraphBuilder:
    """可変ブロックのノード・エッジ集合を管理するビルダ."""

    def __init__(
        self,
        nodes: List[Dict[str, Any]],
        edges: Iterable[Dict[str, Any]],
    ) -> None:
        self._nodes: List[Dict[str, Any]] = [dict(node) for node in nodes]
        self._edges: List[GraphEdgeRecord] = []
        self._original_edges: List[GraphEdgeRecord] = []
        self.replace_edges(edges)

    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_edge(edge: Dict[str, Any]) -> GraphEdgeRecord:
        """edges.csv由来の辞書を GraphEdgeRecord に正規化する。"""

        src = edge.get("source")
        dst = edge.get("target")
        seg = edge.get("segment_id") or edge.get("waypoint_list")
        if src is None or dst is None or seg is None:
            raise ValueError("Edge definition requires source/target/segment_id")
        reversible_raw = edge.get("reversible", 0)
        try:
            reversible = int(reversible_raw)
        except (TypeError, ValueError):
            reversible = 0
        weight_factor = parse_weight_factor(edge.get("weight_factor", 1.0))
        return GraphEdgeRecord(
            str(src),
            str(dst),
            str(seg),
            1 if reversible else 0,
            weight_factor,
        )

    # ------------------------------------------------------------------
    def replace_edges(self, edges: Iterable[Dict[str, Any]]) -> None:
        """内部エッジ集合を与えられた内容で置き換える。"""

        normalized = [self._normalize_edge(edge) for edge in edges]
        self._edges = list(normalized)
        self._original_edges = list(normalized)

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """最初に読み込んだエッジ集合へ戻す。"""

        self._edges = list(self._original_edges)

    # ------------------------------------------------------------------
    def remove_edge_pair(self, node_u: str, node_v: str) -> bool:
        """指定したノード対に該当するエッジ（双方向）を削除する。"""

        key = frozenset({str(node_u), str(node_v)})
        filtered: List[GraphEdgeRecord] = []
        removed = False
        for edge in self._edges:
            if edge.key() == key:
                removed = True
                continue
            filtered.append(edge)
        self._edges = filtered
        return removed

    # ------------------------------------------------------------------
    def add_edge(self, edge: Dict[str, Any]) -> GraphEdgeRecord:
        """新たなエッジを追加する（既存と同一なら無視）。"""

        record = self._normalize_edge(edge)
        for current in self._edges:
            if (
                current.source == record.source
                and current.target == record.target
                and current.segment_id == record.segment_id
                and current.reversible == record.reversible
                and math.isclose(current.weight_factor, record.weight_factor)
            ):
                return current
        self._edges.append(record)
        return record

    # ------------------------------------------------------------------
    def get_nodes(self) -> List[Dict[str, Any]]:
        """ノード集合をコピーで返す。"""

        return [dict(node) for node in self._nodes]

    # ------------------------------------------------------------------
    def get_edges(self) -> List[Dict[str, Any]]:
        """現在のエッジ集合を辞書形式で返す。"""

        return [edge.to_dict() for edge in self._edges]


@dataclass
class SegmentRecord:
    """CSV一枚分のwaypoint配列をキャッシュするデータクラス."""

    segment_id: str
    waypoints: List[WaypointRecord]


@dataclass
class VariableSolverInfo:
    """可変ブロックの経路探索結果を保持するための情報."""

    nodes: Dict[str, Tuple[float, float]]
    solver_result: Dict[str, Any]


@dataclass
class RouteBuildResult:
    """build_route()の結果をまとめるコンテナ."""

    waypoints: List[WaypointRecord]
    origins: List[WaypointOrigin]
    total_distance: float
    has_variable_block: bool
    solver_info: Optional[VariableSolverInfo]


def _log_with_logger(logger: Optional[Any], level: str, message: str) -> None:
    """任意のロガーへログを送出する補助関数."""

    if logger is None:
        return
    log_func = getattr(logger, level, None)
    if not callable(log_func):
        # logging.Logger 互換の warning() しか持たない場合に備える。
        if level == "warn":
            log_func = getattr(logger, "warning", None)
    if callable(log_func):
        log_func(message)


def render_variable_route_overlay(
    nodes: Dict[str, Tuple[float, float]],
    solver_result: Dict[str, Any],
    map_image_path: str,
    map_worldfile_path: str,
    *,
    output_path: Optional[str] = None,
    logger: Optional[Any] = None,
) -> Optional[Any]:
    """solve_variable_route結果を地図画像へ描画しRGB配列を返す."""

    if not map_image_path or not map_worldfile_path:
        return None

    node_sequence = solver_result.get("node_sequence", [])
    visit_order = solver_result.get("visit_order", [])
    if not node_sequence or not visit_order:
        return None

    try:
        return render_route_on_map(
            nodes,
            node_sequence,
            visit_order,
            map_image_path,
            map_worldfile_path,
            output_path=output_path,
        )
    except Exception as exc:  # pylint: disable=broad-except
        _log_with_logger(logger, "warn", f"地図描画の生成に失敗しました: {exc}")
        return None


def render_variable_route_overlay_from_info(
    solver_info: Optional[VariableSolverInfo],
    map_image_path: str,
    map_worldfile_path: str,
    *,
    output_path: Optional[str] = None,
    logger: Optional[Any] = None,
) -> Optional[Any]:
    """VariableSolverInfoを入力として地図描画を生成するラッパー."""

    if solver_info is None:
        return None
    return render_variable_route_overlay(
        solver_info.nodes,
        solver_info.solver_result,
        map_image_path,
        map_worldfile_path,
        output_path=output_path,
        logger=logger,
    )


def resolve_path(base_dir: Optional[str], path_str: str) -> str:
    """base_dirを起点に相対パスを解決する."""

    if not base_dir:
        return path_str
    if os.path.isabs(path_str):
        return path_str
    return os.path.normpath(os.path.join(base_dir, path_str))


def normalize_nodes(
    nodes_data: Any,
) -> Dict[str, Tuple[float, float]]:
    """YAML/CSVから読み込んだnodes情報を辞書へ正規化する."""

    if isinstance(nodes_data, dict):
        normalized: Dict[str, Tuple[float, float]] = {}
        for nid, coords in nodes_data.items():
            if not isinstance(coords, (list, tuple)) or len(coords) != 2:
                raise ValueError(f"ノード座標の形式が不正です: {nid}")
            lat, lon = coords
            normalized[str(nid)] = (float(lat), float(lon))
        return normalized

    normalized = {}
    for item in nodes_data:
        nid = str(item.get("id"))
        if not nid:
            raise ValueError("ノードIDが存在しません。")
        try:
            lat = float(item.get("lat"))
            lon = float(item.get("lon"))
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"ノード {nid} の緯度経度が数値に変換できません: {exc}"
            )
        normalized[nid] = (lat, lon)
    return normalized


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """yaw角[rad]をクォータニオンに変換する."""

    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def parse_waypoint_csv(csv_path: str) -> List[WaypointRecord]:
    """waypoint CSVをWayPointRecordの配列へ変換する."""

    waypoints: List[WaypointRecord] = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            wp = WaypointRecord()
            label_val = row.get("label")
            if label_val is None or label_val == "":
                label_val = row.get("id") or row.get("num") or ""
            wp.label = str(label_val)
            wp.index = i
            try:
                wp.pose.position.x = float(row.get("x", 0.0))
                wp.pose.position.y = float(row.get("y", 0.0))
                wp.pose.position.z = float(row.get("z", 0.0))
            except ValueError as exc:
                raise ValueError(f"Invalid XYZ in {csv_path} at row {i + 1}: {exc}")
            try:
                wp.pose.orientation.x = float(row.get("q1", 0.0))
                wp.pose.orientation.y = float(row.get("q2", 0.0))
                wp.pose.orientation.z = float(row.get("q3", 0.0))
                wp.pose.orientation.w = float(row.get("q4", 1.0))
            except ValueError as exc:
                raise ValueError(
                    f"Invalid quaternion in {csv_path} at row {i + 1}: {exc}"
                )

            def _bool_from_int(value: Any) -> bool:
                try:
                    return bool(int(str(value)))
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
            wp.line_stop = _bool_from_int(
                row.get("line_stop", row.get("line_is_stop", 0))
            )
            wp.signal_stop = _bool_from_int(
                row.get("signal_stop", row.get("signal_is_stop", 0))
            )
            wp.not_skip = _bool_from_int(
                row.get("not_skip", row.get("isnot_skipnum", 0))
            )

            lat_raw = row.get("lat")
            if lat_raw in (None, ""):
                lat_raw = row.get("latitude")
            try:
                if lat_raw not in (None, ""):
                    wp.latitude = float(lat_raw)
            except (TypeError, ValueError):
                wp.latitude = None

            lon_raw = row.get("lon")
            if lon_raw in (None, ""):
                lon_raw = row.get("longitude")
            try:
                if lon_raw not in (None, ""):
                    wp.longitude = float(lon_raw)
            except (TypeError, ValueError):
                wp.longitude = None

            waypoints.append(wp)
    return waypoints


def _distance_between_records(a: WaypointRecord, b: WaypointRecord) -> float:
    """2つのwaypoint間の平面距離を返却する."""

    return math.hypot(
        a.pose.position.x - b.pose.position.x,
        a.pose.position.y - b.pose.position.y,
    )


def _calc_link_cost(
    prev_wp: Optional[WaypointRecord],
    candidate_wp: WaypointRecord,
    next_wp: Optional[WaypointRecord],
) -> float:
    """共有端点の候補を利用した際の接続コストを見積もる."""

    cost = 0.0
    if prev_wp is not None:
        cost += _distance_between_records(prev_wp, candidate_wp)
    if next_wp is not None:
        cost += _distance_between_records(candidate_wp, next_wp)
    return cost


def _merge_endpoint_records(
    base_wp: WaypointRecord,
    new_wp: WaypointRecord,
    prefer_new: bool,
) -> None:
    """重複した端点の属性を統合する."""

    if not base_wp.label:
        base_wp.label = new_wp.label

    if prefer_new:
        base_wp.pose = copy.deepcopy(new_wp.pose)
    else:
        if base_wp.pose.position.x == 0.0 and base_wp.pose.position.y == 0.0:
            base_wp.pose.position = copy.deepcopy(new_wp.pose.position)
        if base_wp.pose.orientation.w == 0.0:
            base_wp.pose.orientation = copy.deepcopy(new_wp.pose.orientation)

    base_wp.right_open = max(base_wp.right_open, new_wp.right_open)
    base_wp.left_open = max(base_wp.left_open, new_wp.left_open)
    base_wp.line_stop = base_wp.line_stop or new_wp.line_stop
    base_wp.signal_stop = base_wp.signal_stop or new_wp.signal_stop
    base_wp.not_skip = base_wp.not_skip or new_wp.not_skip
    base_wp.segment_is_fixed = base_wp.segment_is_fixed or new_wp.segment_is_fixed
    if new_wp.latitude is not None:
        base_wp.latitude = new_wp.latitude
    if new_wp.longitude is not None:
        base_wp.longitude = new_wp.longitude


def concat_records_with_dedup(
    base: List[WaypointRecord],
    ext: List[WaypointRecord],
    eps: float = 1e-6,
) -> List[WaypointRecord]:
    """waypoint配列を重複排除付きで連結する."""

    result = list(base)
    ext_clones = [wp.clone() for wp in ext]
    if not result:
        return ext_clones
    if not ext_clones:
        return result

    last_wp = result[-1]
    first_wp = ext_clones[0]
    duplicate = False
    prefer_new = False

    if _distance_between_records(last_wp, first_wp) <= eps:
        duplicate = True
    elif last_wp.label and first_wp.label and last_wp.label == first_wp.label:
        duplicate = True
        prev_wp = result[-2] if len(result) >= 2 else None
        next_wp = ext_clones[1] if len(ext_clones) >= 2 else None
        cost_keep = _calc_link_cost(prev_wp, last_wp, next_wp)
        cost_new = _calc_link_cost(prev_wp, first_wp, next_wp)
        prefer_new = cost_new < cost_keep

    if duplicate:
        _merge_endpoint_records(last_wp, first_wp, prefer_new)
        result.extend(ext_clones[1:])
    else:
        result.extend(ext_clones)
    return result


def _latlon_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """緯度経度のユークリッド距離を概算する."""

    return math.hypot(lat1 - lat2, lon1 - lon2)


def infer_first_matches_source(
    waypoints: List[WaypointRecord],
    src_label: str,
    dst_label: str,
    nodes: Dict[str, Tuple[float, float]],
) -> Optional[bool]:
    """セグメント先頭がsrc_labelに対応しているかを推定する."""

    if not waypoints:
        return None

    first = waypoints[0]
    last = waypoints[-1]
    if first.label == src_label or last.label == dst_label:
        return True
    if first.label == dst_label or last.label == src_label:
        return False

    src_pos = nodes.get(src_label)
    dst_pos = nodes.get(dst_label)
    if not src_pos or not dst_pos:
        return None

    if first.latitude is None or first.longitude is None:
        return None
    if last.latitude is None or last.longitude is None:
        return None

    first_to_src = _latlon_distance(first.latitude, first.longitude, src_pos[0], src_pos[1])
    first_to_dst = _latlon_distance(first.latitude, first.longitude, dst_pos[0], dst_pos[1])
    last_to_src = _latlon_distance(last.latitude, last.longitude, src_pos[0], src_pos[1])
    last_to_dst = _latlon_distance(last.latitude, last.longitude, dst_pos[0], dst_pos[1])

    tol = 1e-6
    if first_to_src + tol < first_to_dst and last_to_dst + tol < last_to_src:
        return True
    if first_to_dst + tol < first_to_src and last_to_src + tol < last_to_dst:
        return False
    return None


def stamp_edge_end_labels_records(
    waypoints: List[WaypointRecord],
    src_label: str,
    dst_label: str,
) -> None:
    """可変エッジの端点ラベルを設定する."""

    if not waypoints:
        return
    waypoints[0].label = src_label
    waypoints[-1].label = dst_label


def indexing_records(waypoints: List[WaypointRecord]) -> None:
    """waypointのindexを0..N-1に再設定する."""

    for idx, wp in enumerate(waypoints):
        wp.index = idx


def slice_by_labels_records(
    waypoints: List[WaypointRecord],
    start_label: str,
    goal_label: str,
    *,
    start_index_hint: Optional[int] = None,
    goal_index_hint: Optional[int] = None,
) -> Tuple[List[WaypointRecord], int]:
    """start_labelとgoal_labelでwaypoint列をスライスする."""

    start_idx: Optional[int] = None
    goal_idx: Optional[int] = None

    if not waypoints:
        raise ValueError("Waypoint list is empty.")

    def _validate_hint(index_hint: Optional[int], label: str) -> Optional[int]:
        if index_hint is None:
            return None
        if index_hint < 0 or index_hint >= len(waypoints):
            return None
        if label and waypoints[index_hint].label != label:
            return None
        return index_hint

    validated_start_hint = _validate_hint(start_index_hint, start_label)
    validated_goal_hint = _validate_hint(goal_index_hint, goal_label)

    if not start_label:
        start_idx = 0
    elif validated_start_hint is not None:
        start_idx = validated_start_hint
    else:
        for i, wp in enumerate(waypoints):
            if wp.label == start_label:
                start_idx = i
                break

    if not goal_label:
        goal_idx = len(waypoints) - 1
    elif validated_goal_hint is not None:
        goal_idx = validated_goal_hint
    else:
        for j in range(len(waypoints) - 1, -1, -1):
            if waypoints[j].label == goal_label:
                goal_idx = j
                break

    if start_idx is None:
        raise ValueError(f"Start label '{start_label}' not found.")
    if goal_idx is None:
        raise ValueError(f"Goal label '{goal_label}' not found.")
    if start_idx > goal_idx:
        raise ValueError("Invalid slice order: start > goal.")

    sliced = [wp.clone() for wp in waypoints[start_idx : goal_idx + 1]]
    if len(sliced) <= 1:
        raise ValueError("Single-point route not allowed.")
    return sliced, start_idx


def calc_total_distance_records(waypoints: List[WaypointRecord]) -> float:
    """waypoint列の総距離[m]を算出する."""

    distance = 0.0
    for i in range(1, len(waypoints)):
        x0 = waypoints[i - 1].pose.position.x
        y0 = waypoints[i - 1].pose.position.y
        x1 = waypoints[i].pose.position.x
        y1 = waypoints[i].pose.position.y
        distance += math.hypot(x1 - x0, y1 - y0)
    return distance


def adjust_orientations_records(
    waypoints: List[WaypointRecord],
    recalc_ranges: List[Tuple[int, int]],
    block_boundaries: List[int],
) -> None:
    """姿勢の再計算を行う."""

    for start, end in recalc_ranges:
        if start < 0 or end >= len(waypoints) or start >= end:
            continue
        for i in range(start, end):
            dx = waypoints[i + 1].pose.position.x - waypoints[i].pose.position.x
            dy = waypoints[i + 1].pose.position.y - waypoints[i].pose.position.y
            _, _, qz, qw = yaw_to_quaternion(math.atan2(dy, dx))
            waypoints[i].pose.orientation.x = 0.0
            waypoints[i].pose.orientation.y = 0.0
            waypoints[i].pose.orientation.z = qz
            waypoints[i].pose.orientation.w = qw
        dx = waypoints[end].pose.position.x - waypoints[end - 1].pose.position.x
        dy = waypoints[end].pose.position.y - waypoints[end - 1].pose.position.y
        _, _, qz, qw = yaw_to_quaternion(math.atan2(dy, dx))
        waypoints[end].pose.orientation.x = 0.0
        waypoints[end].pose.orientation.y = 0.0
        waypoints[end].pose.orientation.z = qz
        waypoints[end].pose.orientation.w = qw

    for idx in block_boundaries:
        if 0 < idx < len(waypoints):
            dx = waypoints[idx].pose.position.x - waypoints[idx - 1].pose.position.x
            dy = waypoints[idx].pose.position.y - waypoints[idx - 1].pose.position.y
            _, _, qz, qw = yaw_to_quaternion(math.atan2(dy, dx))
            waypoints[idx].pose.orientation.x = 0.0
            waypoints[idx].pose.orientation.y = 0.0
            waypoints[idx].pose.orientation.z = qz
            waypoints[idx].pose.orientation.w = qw

    if len(waypoints) > 1:
        dx = waypoints[-1].pose.position.x - waypoints[-2].pose.position.x
        dy = waypoints[-1].pose.position.y - waypoints[-2].pose.position.y
        _, _, qz, qw = yaw_to_quaternion(math.atan2(dy, dx))
        waypoints[-1].pose.orientation.x = 0.0
        waypoints[-1].pose.orientation.y = 0.0
        waypoints[-1].pose.orientation.z = qz
        waypoints[-1].pose.orientation.w = qw


def apply_segment_fixed_flags_records(
    waypoints: List[WaypointRecord],
    origins: List[WaypointOrigin],
    blocks: List[Dict[str, Any]],
) -> None:
    """segment_is_fixedフラグを設定する."""

    block_type_map: Dict[Tuple[str, int], str] = {}
    for block in blocks:
        name = block.get("name")
        index = block.get("index")
        btype = block.get("type")
        if name is None or index is None:
            continue
        block_type_map[(str(name), int(index))] = str(btype or "")

    for wp in waypoints:
        wp.segment_is_fixed = False

    count = min(len(waypoints), len(origins))
    for i in range(count):
        origin = origins[i]
        block_name = getattr(origin, "block_name", None)
        block_index = getattr(origin, "block_index", None)
        if block_name is None or block_index is None:
            continue
        block_key = (str(block_name), int(block_index))
        block_type = block_type_map.get(block_key, "")
        is_fixed = block_type == "fixed"
        if getattr(origin, "segment_id", None) == "__virtual__":
            is_fixed = False
        waypoints[i].segment_is_fixed = bool(is_fixed)


class RouteBuilder:
    """route_config.yamlを読み込みwaypoint列を生成するビルダー."""

    def __init__(
        self,
        config_yaml_path: str,
        csv_base_dir: Optional[str] = None,
        logger: Optional[Any] = None,
    ) -> None:
        self.config_yaml_path = config_yaml_path
        self.csv_base_dir = csv_base_dir or ""
        self.logger = logger
        self.blocks: List[Dict[str, Any]] = []
        self.segments: Dict[str, SegmentRecord] = {}
        self.graph_builders: Dict[Tuple[str, int], VariableGraphBuilder] = {}

    # ------------------------------------------------------------------
    def _log(self, level: str, message: str) -> None:
        if self.logger is None:
            return
        log_func = getattr(self.logger, level, None)
        if callable(log_func):
            log_func(message)

    # ------------------------------------------------------------------
    def load(self) -> None:
        """YAMLと関連CSVを読み込み内部状態を初期化する."""

        self._load_blocks_from_yaml()
        self._load_csv_segments()

    # ------------------------------------------------------------------
    def _find_block(self, name: str, index: int) -> Optional[Dict[str, Any]]:
        for block in self.blocks:
            if block.get("name") == name and block.get("index") == index:
                return block
        return None

    # ------------------------------------------------------------------
    def get_graph_builder(self, name: str, index: int) -> Optional[VariableGraphBuilder]:
        """可変ブロックに対応するグラフビルダを取得する。"""

        return self.graph_builders.get((name, index))

    # ------------------------------------------------------------------
    def remove_graph_edge(self, name: str, index: int, node_u: str, node_v: str) -> bool:
        """指定ブロックからエッジを削除し、ブロック定義へ反映する。"""

        builder = self.get_graph_builder(name, index)
        if builder is None:
            return False
        removed = builder.remove_edge_pair(node_u, node_v)
        block = self._find_block(name, index)
        if block is not None:
            block["edges"] = builder.get_edges()
        return removed

    # ------------------------------------------------------------------
    def add_graph_edge(self, name: str, index: int, edge: Dict[str, Any]) -> None:
        """指定ブロックへエッジを追加し、ブロック定義へ反映する。"""

        builder = self.get_graph_builder(name, index)
        if builder is None:
            raise RuntimeError(f"Graph builder not found for block: {name}[{index}]")
        builder.add_edge(edge)
        block = self._find_block(name, index)
        if block is not None:
            block["edges"] = builder.get_edges()

    # ------------------------------------------------------------------
    def _load_blocks_from_yaml(self) -> None:
        if not self.config_yaml_path:
            self._log("warn", "config_yaml_path が指定されていません。")
            self.blocks = []
            return
        try:
            with open(self.config_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception as exc:
            self._log("error", f"YAMLの読み込みに失敗しました: {exc}")
            self.blocks = []
            return

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
            self._log("warn", "YAMLに有効なblocks配列がありません。")
            self.blocks = []
            return

        if not self.csv_base_dir:
            self.csv_base_dir = os.path.dirname(os.path.abspath(self.config_yaml_path))

        normalized: List[Dict[str, Any]] = []
        for raw_index, block in enumerate(blocks):
            if not isinstance(block, dict):
                self._log("error", f"blocks[{raw_index}] が辞書ではありません。")
                continue
            btype = block.get("type")
            name = block.get("name", f"block_{raw_index}")
            if btype == "fixed":
                seg = block.get("segment_id")
                if not seg:
                    self._log(
                        "error",
                        f"固定ブロック '{name}' に segment_id が設定されていません。",
                    )
                    continue
                normalized.append(
                    {
                        "type": "fixed",
                        "name": name,
                        "segment_id": seg,
                        "index": len(normalized),
                    }
                )
            elif btype == "variable":
                nodes_file = block.get("nodes_file")
                edges_file = block.get("edges_file")
                start = block.get("start")
                goal = block.get("goal")
                checkpoints = block.get("checkpoints", [])
                missing = [
                    key
                    for key, val in [
                        ("nodes_file", nodes_file),
                        ("edges_file", edges_file),
                        ("start", start),
                        ("goal", goal),
                    ]
                    if not val
                ]
                if missing:
                    self._log(
                        "error",
                        f"可変ブロック '{name}' に必須項目 {missing} が不足しています。",
                    )
                    continue
                if not isinstance(checkpoints, list) or len(checkpoints) < 1:
                    self._log(
                        "error",
                        f"可変ブロック '{name}' のcheckpointsは1件以上必要です。",
                    )
                    continue
                normalized.append(
                    {
                        "type": "variable",
                        "name": name,
                        "nodes_file": nodes_file,
                        "edges_file": edges_file,
                        "start": start,
                        "goal": goal,
                        "checkpoints": checkpoints,
                        "nodes": None,
                        "edges": None,
                        "index": len(normalized),
                    }
                )
            else:
                self._log("error", f"未知のブロックタイプです: {btype}")
        self.blocks = normalized

    # ------------------------------------------------------------------
    def _load_csv_segments(self) -> None:
        self.segments = {}
        self.graph_builders = {}
        if not self.blocks:
            return

        for block in self.blocks:
            btype = block["type"]
            name = block["name"]
            if btype == "fixed":
                seg_path = resolve_path(self.csv_base_dir, block["segment_id"])
                try:
                    waypoints = parse_waypoint_csv(seg_path)
                    self.segments[block["segment_id"]] = SegmentRecord(
                        block["segment_id"], waypoints
                    )
                except Exception as exc:
                    self._log(
                        "error",
                        f"固定ブロック '{name}' のCSV読み込みに失敗しました: {exc}",
                    )
            elif btype == "variable":
                nodes_file = resolve_path(self.csv_base_dir, block["nodes_file"])
                edges_file = resolve_path(self.csv_base_dir, block["edges_file"])
                nodes: List[Dict[str, Any]] = []
                try:
                    with open(nodes_file, newline="", encoding="utf-8") as f:
                        reader = csv.DictReader(f)
                        for row in reader:
                            nid = row.get("id")
                            if nid is None or nid == "":
                                raise ValueError("nodes.csv: 'id' が欠落しています。")
                            lat = float(row.get("lat"))
                            lon = float(row.get("lon"))
                            nodes.append({"id": str(nid), "lat": lat, "lon": lon})
                    block["nodes"] = nodes
                except Exception as exc:
                    self._log(
                        "error",
                        f"可変ブロック '{name}' のnodes読み込みに失敗しました: {exc}",
                    )
                    block["nodes"] = []
                    continue

                edges: List[Dict[str, Any]] = []
                try:
                    with open(edges_file, newline="", encoding="utf-8") as f:
                        reader = csv.DictReader(f)
                        for row in reader:
                            source = row.get("source")
                            target = row.get("target")
                            seg_rel = row.get("waypoint_list")
                            reversible_raw = row.get("reversible", "1")
                            weight_factor = parse_weight_factor(
                                row.get("weight_factor", 1.0)
                            )
                            if not source or not target or not seg_rel:
                                raise ValueError(
                                    "edges.csv: source/target/waypoint_list は必須です。"
                                )
                            reversible = int(str(reversible_raw))
                            if reversible not in (0, 1):
                                raise ValueError(
                                    "edges.csv: reversible は 0 または 1 で指定してください。"
                                )
                            edge = {
                                "source": str(source),
                                "target": str(target),
                                "waypoint_list": seg_rel,
                                "reversible": reversible,
                                "weight_factor": weight_factor,
                            }
                            edges.append(edge)
                            seg_path = resolve_path(self.csv_base_dir, seg_rel)
                            if seg_rel not in self.segments:
                                try:
                                    waypoints = parse_waypoint_csv(seg_path)
                                    self.segments[seg_rel] = SegmentRecord(
                                        seg_rel, waypoints
                                    )
                                except Exception as exc:
                                    raise RuntimeError(
                                        f"segment '{seg_path}' の読み込みに失敗しました: {exc}"
                                    )
                    block["edges"] = edges
                    key = (block["name"], block["index"])
                    self.graph_builders[key] = VariableGraphBuilder(nodes, edges)
                except Exception as exc:
                    self._log(
                        "error",
                        f"可変ブロック '{name}' のedges読み込みに失敗しました: {exc}",
                    )
                    block["edges"] = []
                    key = (block["name"], block["index"])
                    self.graph_builders[key] = VariableGraphBuilder(nodes, [])

    # ------------------------------------------------------------------
    def _prepare_edges_for_solver(
        self, edges: List[Dict[str, Any]]
    ) -> Tuple[List[Dict[str, Any]], Dict[str, str]]:
        solver_edges: List[Dict[str, Any]] = []
        abs_to_rel: Dict[str, str] = {}
        for edge in edges:
            seg_key = edge.get("segment_id") or edge.get("waypoint_list")
            if not seg_key:
                raise ValueError("エッジ定義にsegment_id/waypoint_listがありません。")
            seg_rel = str(seg_key)
            seg_abs = resolve_path(self.csv_base_dir, seg_rel)
            abs_to_rel[seg_abs] = seg_rel
            solver_edges.append(
                {
                    "source": edge.get("source"),
                    "target": edge.get("target"),
                    "segment_id": seg_abs,
                    "reversible": edge.get("reversible", 0),
                    "weight_factor": edge.get("weight_factor", 1.0),
                }
            )
        return solver_edges, abs_to_rel

    # ------------------------------------------------------------------
    def run_variable_solver(
        self,
        nodes: Dict[str, Tuple[float, float]],
        edges: List[Dict[str, Any]],
        start: str,
        goal: str,
        checkpoints: List[str],
    ) -> Dict[str, Any]:
        solver_edges, abs_to_rel = self._prepare_edges_for_solver(edges)
        result = solve_variable_route(
            nodes=nodes,
            edges=solver_edges,
            start=start,
            goal=goal,
            checkpoints=checkpoints,
        )
        edge_sequence = result.get("edge_sequence", [])
        for entry in edge_sequence:
            seg_abs = entry.get("segment_id")
            if seg_abs is None:
                continue
            seg_rel = abs_to_rel.get(str(seg_abs))
            if seg_rel:
                entry["segment_id"] = seg_rel
        return result

    # ------------------------------------------------------------------
    def build_route(
        self,
        start_label: str,
        goal_label: str,
        checkpoint_labels: List[str],
        *,
        start_label_origin: Optional[Tuple[str, int]] = None,
    ) -> RouteBuildResult:
        if not self.blocks:
            raise RuntimeError("ブロック定義が読み込まれていません。")

        route_wps: List[WaypointRecord] = []
        origins: List[WaypointOrigin] = []
        reversed_ranges: List[Tuple[int, int]] = []
        block_tail_indices: List[int] = []
        has_variable = False
        last_solver_info: Optional[VariableSolverInfo] = None

        for block in self.blocks:
            btype = block["type"]
            bname = block["name"]
            bindex = block["index"]
            if btype == "fixed":
                seg_id = block["segment_id"]
                entry = self.segments.get(seg_id)
                if entry is None:
                    raise RuntimeError(
                        f"[fixed:{bname}] 対応するsegmentが読み込まれていません: {seg_id}"
                    )
                before = len(route_wps)
                route_wps = concat_records_with_dedup(route_wps, entry.waypoints)
                added = len(route_wps) - before
                for _ in range(added):
                    origins.append(
                        WaypointOrigin(
                            block_name=bname,
                            block_index=bindex,
                            segment_id=seg_id,
                            edge_u=None,
                            edge_v=None,
                            index_in_edge=None,
                            u_first=None,
                        )
                    )
                block_tail_indices.append(len(route_wps) - 1)
            elif btype == "variable":
                has_variable = True
                merged_cps = list(
                    dict.fromkeys(
                        (block.get("checkpoints", []) or []) + list(checkpoint_labels)
                    )
                )
                graph_builder = self.graph_builders.get((bname, bindex))
                if graph_builder is not None:
                    nodes_raw = graph_builder.get_nodes()
                    edges = graph_builder.get_edges()
                    block["edges"] = edges
                    block["nodes"] = nodes_raw
                else:
                    nodes_raw = block.get("nodes") or []
                    edges = block.get("edges") or []
                start = block.get("start")
                goal = block.get("goal")
                if not nodes_raw or not edges:
                    raise RuntimeError(f"[variable:{bname}] nodes/edges が空です。")
                if not start or not goal:
                    raise RuntimeError(f"[variable:{bname}] start/goal が未設定です。")

                nodes_dict = normalize_nodes(nodes_raw)
                solve_result = self.run_variable_solver(
                    nodes=nodes_dict,
                    edges=edges,
                    start=start,
                    goal=goal,
                    checkpoints=merged_cps,
                )
                edge_seq: List[Dict[str, Any]] = solve_result.get("edge_sequence", [])
                if not edge_seq:
                    raise RuntimeError(
                        f"[variable:{bname}] solverが空のedge_sequenceを返しました。"
                    )
                last_solver_info = VariableSolverInfo(nodes_dict, solve_result)

                edge_lookup: Dict[Tuple[str, str, str], Dict[str, Any]] = {}
                for edge_def in edges:
                    seg_key = edge_def.get("segment_id") or edge_def.get("waypoint_list")
                    if not seg_key:
                        continue
                    edge_lookup[(edge_def["source"], edge_def["target"], str(seg_key))] = edge_def

                for entry in edge_seq:
                    seg_id = entry["segment_id"]
                    direction = entry["direction"]
                    src = entry["source"]
                    dst = entry["target"]
                    cache = self.segments.get(seg_id)
                    if cache is None:
                        raise RuntimeError(
                            f"[variable:{bname}] segmentが未ロードです: {seg_id}"
                        )
                    edge_meta = edge_lookup.get((src, dst, seg_id))
                    if edge_meta is None:
                        edge_meta = edge_lookup.get((dst, src, seg_id))
                    row_src = src
                    row_dst = dst
                    if edge_meta is not None:
                        row_src = edge_meta.get("source", src)
                        row_dst = edge_meta.get("target", dst)

                    alignment = infer_first_matches_source(
                        cache.waypoints, src_label=src, dst_label=dst, nodes=nodes_dict
                    )
                    # reversible=1 でCSVの記述向きがsource/targetと逆転している場合に備え、
                    # セグメント先頭がどちらのノードに近いかを確認して向きを決定する。
                    if alignment is None:
                        if edge_meta is not None:
                            if src == row_src and dst == row_dst:
                                u_first = True
                            elif src == row_dst and dst == row_src:
                                u_first = False
                            else:
                                u_first = True if direction == "forward" else False
                        else:
                            u_first = True if direction == "forward" else False
                    else:
                        u_first = alignment
                    seg_wps = [wp.clone() for wp in cache.waypoints]
                    if not u_first:
                        seg_wps.reverse()
                    stamp_edge_end_labels_records(seg_wps, src_label=src, dst_label=dst)

                    start_idx = len(route_wps)
                    route_wps = concat_records_with_dedup(route_wps, seg_wps)
                    end_idx = len(route_wps) - 1
                    local_len = end_idx - start_idx + 1
                    for i_local in range(local_len):
                        origins.append(
                            WaypointOrigin(
                                block_name=bname,
                                block_index=bindex,
                                segment_id=seg_id,
                                edge_u=src,
                                edge_v=dst,
                                index_in_edge=i_local,
                                u_first=u_first,
                            )
                        )
                    if not u_first and end_idx > start_idx:
                        reversed_ranges.append((start_idx, end_idx))
                block_tail_indices.append(len(route_wps) - 1)
            else:
                self._log("warn", f"未知のブロックタイプを無視します: {btype}")

        if not route_wps:
            raise RuntimeError("blocks定義からwaypointを生成できませんでした。")

        self._log("info", f"start_label: {start_label}, goal_label: {goal_label}, route: {route_wps}")
        start_index_hint: Optional[int] = None
        if start_label and start_label_origin is not None:
            hint_name, hint_index = start_label_origin
            candidates: List[int] = []
            for idx, origin in enumerate(origins):
                if origin.block_name != hint_name or origin.block_index != hint_index:
                    continue
                if route_wps[idx].label == start_label:
                    candidates.append(idx)
                prev_idx = idx - 1
                if prev_idx >= 0 and route_wps[prev_idx].label == start_label:
                    candidates.append(prev_idx)
            if candidates:
                start_index_hint = min(candidates)
            else:
                self._log(
                    "warn",
                    (
                        "start_label_origin hint was provided but no matching waypoint was found. "
                        "Falling back to the first occurrence of the label."
                    ),
                )

        sliced, start_offset = slice_by_labels_records(
            route_wps,
            start_label,
            goal_label,
            start_index_hint=start_index_hint,
        )
        origins = origins[start_offset : start_offset + len(sliced)]
        indexing_records(sliced)
        total_distance = calc_total_distance_records(sliced)

        def _shift_inside(
            rng: Tuple[int, int], offset: int, length: int
        ) -> Optional[Tuple[int, int]]:
            s = rng[0] - offset
            e = rng[1] - offset
            if e < 0 or s >= length:
                return None
            s = max(s, 0)
            e = min(e, length - 1)
            if s >= e:
                return None
            return s, e

        recalc_ranges: List[Tuple[int, int]] = []
        for rng in reversed_ranges:
            shifted = _shift_inside(rng, start_offset, len(sliced))
            if shifted is not None:
                recalc_ranges.append(shifted)

        shifted_block_tails: List[int] = []
        for idx in block_tail_indices:
            shifted_idx = idx - start_offset
            if 0 <= shifted_idx < len(sliced):
                shifted_block_tails.append(shifted_idx)

        adjust_orientations_records(sliced, recalc_ranges, shifted_block_tails)
        apply_segment_fixed_flags_records(sliced, origins, self.blocks)

        return RouteBuildResult(
            waypoints=sliced,
            origins=origins,
            total_distance=total_distance,
            has_variable_block=has_variable,
            solver_info=last_solver_info,
        )


def write_waypoints_to_csv(path: str, waypoints: List[WaypointRecord]) -> None:
    """WaypointRecordの配列をCSVファイルへ書き出す."""

    header = [
        "label",
        "index",
        "x",
        "y",
        "z",
        "q1",
        "q2",
        "q3",
        "q4",
        "right_open",
        "left_open",
        "line_stop",
        "signal_stop",
        "not_skip",
        "segment_is_fixed",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        for wp in waypoints:
            writer.writerow(
                {
                    "label": wp.label,
                    "index": wp.index,
                    "x": wp.pose.position.x,
                    "y": wp.pose.position.y,
                    "z": wp.pose.position.z,
                    "q1": wp.pose.orientation.x,
                    "q2": wp.pose.orientation.y,
                    "q3": wp.pose.orientation.z,
                    "q4": wp.pose.orientation.w,
                    "right_open": wp.right_open,
                    "left_open": wp.left_open,
                    "line_stop": int(bool(wp.line_stop)),
                    "signal_stop": int(bool(wp.signal_stop)),
                    "not_skip": int(bool(wp.not_skip)),
                    "segment_is_fixed": int(bool(wp.segment_is_fixed)),
                }
            )


def build_waypoints_from_config(
    config_yaml_path: str,
    start_label: str,
    goal_label: str,
    checkpoints: List[str],
    csv_base_dir: Optional[str] = None,
) -> RouteBuildResult:
    """外部スクリプト用の簡易インターフェース."""

    builder = RouteBuilder(config_yaml_path, csv_base_dir=csv_base_dir)
    builder.load()
    return builder.build_route(start_label, goal_label, checkpoints)


def main() -> None:
    """コマンドラインエントリーポイント."""

    import argparse

    parser = argparse.ArgumentParser(
        description="route_config.yamlからwaypoint CSVを生成します。"
    )
    parser.add_argument("config", help="route_config.yamlのパス")
    parser.add_argument("start", help="開始ラベル")
    parser.add_argument("goal", help="終了ラベル")
    parser.add_argument(
        "--checkpoint",
        action="append",
        default=[],
        help="追加のチェックポイントラベル（複数指定可）",
    )
    parser.add_argument(
        "--output",
        default="generated_waypoints.csv",
        help="出力するCSVファイルパス",
    )
    parser.add_argument(
        "--csv-base-dir",
        default=None,
        help="CSV探索の基準ディレクトリ（省略時はYAMLの配置場所）",
    )
    parser.add_argument(
        "--map-image",
        default=None,
        help="可変ブロックを地図上へ描画する際の背景画像パス",
    )
    parser.add_argument(
        "--map-worldfile",
        default=None,
        help="背景画像に対応するワールドファイル（tfw等）のパス",
    )
    parser.add_argument(
        "--map-output",
        default=None,
        help="生成した地図画像の保存先（省略時はCSVと同じ接頭辞）",
    )
    args = parser.parse_args()

    builder = RouteBuilder(args.config, csv_base_dir=args.csv_base_dir)
    builder.load()
    result = builder.build_route(
        start_label=args.start,
        goal_label=args.goal,
        checkpoint_labels=list(args.checkpoint or []),
    )
    write_waypoints_to_csv(args.output, result.waypoints)

    map_args_provided = any(
        value is not None for value in [args.map_image, args.map_worldfile, args.map_output]
    )
    if map_args_provided:
        if not args.map_image or not args.map_worldfile:
            print(
                "地図画像を生成する場合は --map-image と --map-worldfile を両方指定してください。",
                file=sys.stderr,
            )
        elif not result.has_variable_block or result.solver_info is None:
            print("可変ブロックが存在しないため地図画像は生成されませんでした。")
        else:
            output_path = args.map_output
            if not output_path:
                base, _ = os.path.splitext(os.path.abspath(args.output))
                output_path = base + "_map.png"
            rgb_array = render_variable_route_overlay_from_info(
                result.solver_info,
                args.map_image,
                args.map_worldfile,
                output_path=output_path,
            )
            if rgb_array is None:
                print("地図画像の生成に失敗しました。", file=sys.stderr)
            else:
                print(f"地図画像を保存しました: {output_path}")


if __name__ == "__main__":
    main()
