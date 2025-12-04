#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""waypoint を segment（edge）に切り出し、エッジリストを生成するスクリプト.

実行例:
    python segment_waypoints.py --waypoints waypoint.csv --nodes nodes.csv

機能概要:
- waypointリスト（CSV）とノードリスト（CSV）を入力に取り、以下を行う:
  1) waypoint の先頭/末尾の node が負なら切り出して fixed/ 配下に保存（保存時は node 列を削除）。
  2) 残りの waypoint のうち node が非負（数値/文字列を含む）の行を、最も近いノードの id に置換。
  3) セグメントを variable/segments/edge_端点A_端点B.csv として保存（端点行を含め、保存時は node 列を削除）。
     - セグメント内の label は、端点は node 値（ノードID）に、中間は 先頭端点ID + "_n"。
    4) variable/segments/ 内の全 edge_*.csv を走査し、
     variable/edges.csv を生成（列: source, target, waypoint_list, reversible,
     weight_factor=1）。
     - waypoint_list は edge CSV の相対パス。
     - 逆方向 edge_端点B_端点A.csv が存在する場合 reversible=0、無い場合 1。

補足:
- 本実行前に variable/segments の既存 edge_*.csv をクリーンし、古い数値名ファイルを除去する。
- waypoint CSV: 'latitude','longitude','node','label' 必須
- nodes CSV: 'id','lat','lon' 必須
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple, Union

import pandas as pd


# =========================
# ユーティリティ
# =========================
def ensure_dir(path: Path) -> None:
    """指定ディレクトリが無ければ作成する."""
    path.mkdir(parents=True, exist_ok=True)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """2点の緯度経度（度）から地表距離（メートル）を返す."""
    r_earth = 6371000.0  # 地球半径[m]
    rad = math.radians
    dlat = rad(lat2 - lat1)
    dlon = rad(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(rad(lat1)) * math.cos(rad(lat2)) * math.sin(dlon / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r_earth * c


Number = Union[int, float]


def _as_number(s: str) -> Optional[Number]:
    """文字列を数値に変換（int優先→float）。失敗時 None."""
    try:
        if "." not in s and "e" not in s.lower():
            return int(s)
        return float(s)
    except Exception:
        return None


def is_negative_numeric(value: object) -> bool:
    """値が『負（int/float/数値文字列）』か判定する."""
    if isinstance(value, (int, float)):
        return value < 0
    if isinstance(value, str):
        num = _as_number(value.strip())
        return (num is not None) and (num < 0)
    return False


def is_non_negative_marker(value: object) -> bool:
    """端点判定: 『非負（int/float/数値文字列）』または『非数値の文字列（ID想定）』を True.

    - 数値（int/float）: 0 以上 → True、負 → False
    - 数値文字列: 0 以上 → True、負 → False
    - 非数値文字列（例: 'C1'）: True（IDとみなす）
    - None/空: False
    - その他型: False
    """
    if value is None:
        return False
    if isinstance(value, (int, float)):
        return value >= 0
    if isinstance(value, str):
        s = value.strip()
        if s == "":
            return False
        num = _as_number(s)
        if num is not None:
            return num >= 0
        # 数値でなければ ID 文字列とみなす
        return True
    return False


# =========================
# 前処理
# =========================
def _find_nearest_node(
    lat: float,
    lon: float,
    node_ids: Sequence[str],
    node_lats: Sequence[float],
    node_lons: Sequence[float],
) -> Tuple[Optional[str], float]:
    """nodes.csv から最も近いノードと距離[m]を返す."""

    best_id: Optional[str] = None
    best_dist: float = float("inf")
    for nid, nlat, nlon in zip(node_ids, node_lats, node_lons):
        dist = haversine_m(lat, lon, float(nlat), float(nlon))
        if dist < best_dist:
            best_dist = dist
            best_id = nid
    return best_id, best_dist


def sanitize_waypoint_nodes(
    wp_df: pd.DataFrame,
    nodes_df: pd.DataFrame,
    representative_threshold_m: float = 10.0,
) -> pd.DataFrame:
    """node列の値を代表点判定ルールに従い補正する."""

    df = wp_df.copy()
    node_ids: List[str] = nodes_df["id"].astype(str).tolist()
    node_lats: List[float] = nodes_df["lat"].astype(float).tolist()
    node_lons: List[float] = nodes_df["lon"].astype(float).tolist()

    # Step A: ノード候補と最短距離を取得し、判定閾値以上離れていればノードではないとみなす。
    nearest_entries: List[Tuple[int, str, float]] = []
    for idx, row in df.iterrows():
        if not is_non_negative_marker(row["node"]):
            continue
        lat = float(row["latitude"])
        lon = float(row["longitude"])
        best_id, best_dist = _find_nearest_node(lat, lon, node_ids, node_lats, node_lons)
        if best_id is None or best_dist >= representative_threshold_m:
            df.at[idx, "node"] = -1
            continue
        nearest_entries.append((idx, best_id, best_dist))

    # Step B: 同じノードに紐づく行が連続している場合は最も距離が近い行のみを残す。
    nearest_entries.sort(key=lambda entry: entry[0])
    i = 0
    while i < len(nearest_entries):
        idx_i, current_id, _ = nearest_entries[i]
        if not is_non_negative_marker(df.at[idx_i, "node"]):
            i += 1
            continue

        j = i + 1
        block: List[Tuple[int, str, float]] = [(idx_i, current_id, nearest_entries[i][2])]
        while j < len(nearest_entries) and nearest_entries[j][1] == current_id:
            idx_j, _, dist_j = nearest_entries[j]
            if is_non_negative_marker(df.at[idx_j, "node"]):
                block.append((idx_j, current_id, dist_j))
            j += 1

        if len(block) >= 2:
            best_entry = min(block, key=lambda entry: entry[2])
            for idx_entry, _, _ in block:
                if idx_entry != best_entry[0]:
                    df.at[idx_entry, "node"] = -1

        i = j

    return df


# =========================
# コア処理
# =========================
def cut_head_tail_and_save(
    wp_df: pd.DataFrame, out_dir_fixed: Path
) -> Tuple[pd.DataFrame, Optional[pd.DataFrame], Optional[pd.DataFrame]]:
    """waypoint の先頭/末尾の負 node 部分を切り出して保存（保存時は node 列を削除）."""
    ensure_dir(out_dir_fixed)
    df = wp_df.copy()

    # --- 先頭側 ---
    fixed_first_df: Optional[pd.DataFrame] = None
    if len(df) > 0 and is_negative_numeric(df.iloc[0]["node"]):
        first_nonneg_idx = None
        for i, val in enumerate(df["node"].tolist()):
            if not is_negative_numeric(val):
                first_nonneg_idx = i
                break
        if first_nonneg_idx is not None and first_nonneg_idx > 0:
            fixed_first_df = df.iloc[: first_nonneg_idx].copy()
            if "node" in fixed_first_df.columns:
                fixed_first_df = fixed_first_df.drop(columns=["node"])
            fixed_first_path = out_dir_fixed / "fixed_first.csv"
            fixed_first_df.to_csv(fixed_first_path, index=False, encoding="utf-8")
            df = df.iloc[first_nonneg_idx:].reset_index(drop=True)

    # --- 末尾側 ---
    fixed_second_df: Optional[pd.DataFrame] = None
    if len(df) > 0 and is_negative_numeric(df.iloc[-1]["node"]):
        last_nonneg_idx = None
        for i_rev, val in enumerate(reversed(df["node"].tolist())):
            if not is_negative_numeric(val):
                last_nonneg_idx = len(df) - 1 - i_rev
                break
        if last_nonneg_idx is not None and last_nonneg_idx < len(df) - 1:
            fixed_second_df = df.iloc[last_nonneg_idx + 1 :].copy()
            if "node" in fixed_second_df.columns:
                fixed_second_df = fixed_second_df.drop(columns=["node"])
            fixed_second_path = out_dir_fixed / "fixed_second.csv"
            fixed_second_df.to_csv(fixed_second_path, index=False, encoding="utf-8")
            df = df.iloc[: last_nonneg_idx + 1].reset_index(drop=True)

    return df, fixed_first_df, fixed_second_df


def replace_nonneg_nodes_with_nearest_id(
    wp_df: pd.DataFrame, nodes_df: pd.DataFrame
) -> pd.DataFrame:
    """node が非負（数値/文字列含む）の行を最近傍ノードIDに置換."""
    df = wp_df.copy()

    # ノード一覧を準備
    node_ids: List[str] = nodes_df["id"].astype(str).tolist()
    node_lats: List[float] = nodes_df["lat"].astype(float).tolist()
    node_lons: List[float] = nodes_df["lon"].astype(float).tolist()

    # 置換実行
    for idx, row in df.iterrows():
        val = row["node"]
        if is_non_negative_marker(val):
            lat = float(row["latitude"])
            lon = float(row["longitude"])
            best_id: Optional[str] = None
            best_dist: float = float("inf")
            for nid, nlat, nlon in zip(node_ids, node_lats, node_lons):
                d = haversine_m(lat, lon, float(nlat), float(nlon))
                if d < best_dist:
                    best_dist = d
                    best_id = nid
            if best_id is not None:
                # ノード列を必ず「文字列ID」で統一
                df.at[idx, "node"] = str(best_id)

    return df


def iter_endpoint_indices(node_series: Sequence[object]) -> Iterable[int]:
    """端点（node が非負=文字列含む）のインデックスを返す."""
    for i, v in enumerate(node_series):
        if is_non_negative_marker(v):
            yield i


def relabel_segment_labels_inplace(seg_df: pd.DataFrame) -> None:
    """セグメント内の label を仕様に従い書き換える."""
    if seg_df.empty:
        return
    start_node_str = str(seg_df.iloc[0]["node"])
    end_node_str = str(seg_df.iloc[-1]["node"])
    # 端点のラベルをノードIDにする
    seg_df.at[seg_df.index[0], "label"] = start_node_str
    if len(seg_df) > 1:
        seg_df.at[seg_df.index[-1], "label"] = end_node_str
    # 中間点のラベルは start-end の組み合わせに連番を付ける
    base_label = f"{start_node_str}-{end_node_str}"
    counter = 1
    for ridx in seg_df.index[1:-1]:
        seg_df.at[ridx, "label"] = f"{base_label}_{counter}"
        counter += 1


def clean_segments_dir(out_dir_segments: Path) -> None:
    """セグメント出力前に既存 edge_*.csv を削除（古い数値名などを一掃）."""
    if not out_dir_segments.exists():
        return
    for p in out_dir_segments.glob("edge_*.csv"):
        try:
            p.unlink()
        except Exception:
            pass


def slice_and_save_segments(wp_df: pd.DataFrame, out_dir_segments: Path) -> List[Path]:
    """端点をもとにセグメントを切り出し variable/segments に保存."""
    ensure_dir(out_dir_segments)
    saved_paths: List[Path] = []

    endpoints: List[int] = list(iter_endpoint_indices(wp_df["node"].tolist()))
    if len(endpoints) < 2:
        return saved_paths

    for i in range(len(endpoints) - 1):
        s = endpoints[i]
        t = endpoints[i + 1]
        seg_df = wp_df.iloc[s : t + 1].copy()  # 端点を含めて抽出

        # ラベル書き換え
        start_node_value = str(seg_df.iloc[0]["node"])
        relabel_segment_labels_inplace(seg_df)

        # 保存用に node 列を削除
        if "node" in seg_df.columns:
            seg_df_to_save = seg_df.drop(columns=["node"])
        else:
            seg_df_to_save = seg_df

        # ファイル名
        end_node_value = str(seg_df.iloc[-1]["node"])
        filename = f"edge_{start_node_value}_{end_node_value}.csv"
        out_path = out_dir_segments / filename

        seg_df_to_save.to_csv(out_path, index=False, encoding="utf-8")
        saved_paths.append(out_path)

    return saved_paths


def _parse_edge_filename(edge_csv_path: Path) -> Optional[Tuple[str, str]]:
    """edgeファイル名から (source, target) を取り出す安全なパーサ.

    形式: edge_端点A_端点B.csv
    - 端点名に '_' が含まれても、'edge_' 以降を末尾の '_' で二分割する。
    """
    stem = edge_csv_path.stem  # 例: "edge_C1_C2"
    prefix = "edge_"
    if not stem.startswith(prefix):
        return None
    rest = stem[len(prefix) :]  # "C1_C2"（C1やC2に'_'が含まれていてもよい）
    split_pos = rest.rfind("_")
    if split_pos <= 0 or split_pos >= len(rest) - 1:
        return None
    src = rest[:split_pos]
    tgt = rest[split_pos + 1 :]
    return src, tgt


def build_edge_list(out_dir_segments: Path, out_path_edges: Path) -> None:
    """segmentsディレクトリ内の全 edge_*.csv から edges.csv を生成.

    edges.csv 列: source, target, waypoint_list, reversible, weight_factor
    - waypoint_list には各 edge CSV の相対パス（例: variable/segments/edge_C1_C2.csv）
    - reversible は逆方向ファイルが「存在すれば 0、無ければ 1」
    - weight_factor は係数 1 固定で出力し、ルート作成時のデフォルト値を表す
    """
    ensure_dir(out_dir_segments)
    edge_files = list(out_dir_segments.glob("edge_*.csv"))

    # source,target -> パス の辞書
    edge_map: dict[Tuple[str, str], Path] = {}
    for f in edge_files:
        parsed = _parse_edge_filename(f)
        if parsed is None:
            continue
        src, tgt = parsed
        edge_map[(src, tgt)] = f

    # reversible 判定＆レコード作成
    records: List[Tuple[str, str, str, int, int]] = []
    for (src, tgt), path in edge_map.items():
        reverse_exists = (tgt, src) in edge_map
        reversible = 0 if reverse_exists else 1
        rel_path = str(Path("variable/segments") / path.name)
        records.append((src, tgt, rel_path, reversible, 1))

    # 出力
    ensure_dir(out_path_edges.parent)
    df_edges = pd.DataFrame(
        records,
        columns=["source", "target", "waypoint_list", "reversible", "weight_factor"],
    )
    df_edges = df_edges.sort_values(by=["source", "target"]).reset_index(drop=True)
    df_edges.to_csv(out_path_edges, index=False, encoding="utf-8")


# =========================
# メイン
# =========================
def load_csvs(waypoints_path: Path, nodes_path: Path) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """CSV を読み込む."""
    wp_df = pd.read_csv(waypoints_path)
    nodes_df = pd.read_csv(nodes_path)
    for col in ("latitude", "longitude", "node", "label"):
        if col not in wp_df.columns:
            raise ValueError(f"waypoint CSV に必須列 '{col}' がありません。")
    for col in ("id", "lat", "lon"):
        if col not in nodes_df.columns:
            raise ValueError(f"nodes CSV に必須列 '{col}' がありません。")
    return wp_df, nodes_df


def run(waypoints_path: Path, nodes_path: Path) -> None:
    """全処理を実行."""
    wp_df, nodes_df = load_csvs(waypoints_path, nodes_path)

    out_dir_fixed = Path("fixed")
    out_dir_segments = Path("variable/segments")
    out_path_edges = Path("variable/edges.csv")

    # 0) node列の事前精査（代表点条件を満たさない行を除外）
    wp_df = sanitize_waypoint_nodes(wp_df, nodes_df)

    # セグメント出力前に古い edge_*.csv を削除（数値名の取り込みを防ぐ）
    clean_segments_dir(out_dir_segments)

    # 1) 先頭/末尾の切り出し
    wp_core_df, _, _ = cut_head_tail_and_save(wp_df, out_dir_fixed)

    # 2) 最近傍ノード置換（floatや数値文字列も正しく対象化）
    wp_core_df = replace_nonneg_nodes_with_nearest_id(wp_core_df, nodes_df)

    # 3) セグメント出力（必ずノードID文字列がファイル名に反映される）
    _ = slice_and_save_segments(wp_core_df, out_dir_segments)

    # 4) エッジリスト出力（source,target はノードID、waypoint_list は相対パス）
    build_edge_list(out_dir_segments, out_path_edges)

    print("Done: variable/segments/ および variable/edges.csv を出力しました。")


def parse_args() -> argparse.Namespace:
    """引数パーサ."""
    p = argparse.ArgumentParser(
        description="waypoint を node 端点で分割し、edgeファイルと edge リストを生成します。"
    )
    p.add_argument("--waypoints", type=Path, required=True, help="waypoint CSV のパス")
    p.add_argument("--nodes", type=Path, required=True, help="nodes CSV のパス")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(args.waypoints, args.nodes)

