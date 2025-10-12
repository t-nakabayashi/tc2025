"""graph_solver.py

可変ルート探索を行うためのライブラリおよびCLIツール

主な機能と仕様:
-----------------
- 無向グラフとして扱い、start→checkpoints群→goal をすべて通過する最短経路（順序最適化/TSPパス）を探索する。
- reversible=1 の場合はひとつのCSV（セグメント）を forward/reverse 双方向で利用できるとみなし、内部的にも両方向の辺を用意する。
- reversible=0 の場合は、U→V と V→U それぞれの行が edges.csv に存在する必要がある（対向行が無ければエラー）。
- route_planner など外部からライブラリ関数として呼び出せる（外部I/Fは固定）。
- CLI から単体で動作確認できるように main() も提供する。
- JSON 出力時の tuple キー問題（dist_matrix, path_table）を文字列化で解決。
- 可視化は背景地図つき（地図画像はMAP_IMAGE_PATHの定数で指定）。

【入力仕様（CSV）】
- nodes.csv: id,lat,lon
- edges.csv: source,target,segment_id,reversible
  - segment_id は Waypoint CSV（セグメント）のファイルパス
  - reversible は 0/1, true/false, yes/no（大小無視）
- Waypoint CSV（セグメント）:
  - いずれかの形式: (x,y) または (lat,lon) / (lon,lat)

【出力】
- solve_variable_route() は edge_sequence / node_sequence / visit_order / stats を返す（外部I/F）。
- CLI は PNG（経路・端点完全グラフ）と JSON（経路詳細）を outdir に保存する。
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple, Union

import matplotlib.pyplot as plt
import japanize_matplotlib
import networkx as nx

# ---------------------------------------------------------------------------
# 地図画像と表示範囲の定義（定数）
# ---------------------------------------------------------------------------
MAP_IMAGE_PATH = "./map/gsi20250928000333444.png"
MAP_WIDTH_PX = 600
MAP_HEIGHT_PX = 600
MAP_LAT_MIN = 36.079371
MAP_LAT_MAX = 36.082107
MAP_LON_MIN = 140.079144
MAP_LON_MAX = 140.082352


# ============================================================
# 低レベルユーティリティ
# ============================================================

def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """WGS-84 を仮定したハヴァーサイン距離（m）。"""
    R = 6371_000.0  # 地球半径[m]
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def _euclid_len(pts: List[Tuple[float, float]]) -> float:
    """与えられた点列の総ユークリッド距離を計算する。"""
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        total += math.hypot(dx, dy)
    return total


def _load_waypoint_csv_length(path: str) -> Tuple[float, List[Tuple[float, float]]]:
    """セグメント（waypoint）CSV を読み込み、総距離と点列を返す。

    CSV は以下のいずれかの列名を持つこと:
      - x,y                      → 直交座標（メートル等）。ユークリッド長で合計。
      - lon,lat もしくは lat,lon → 地理座標（度）。ハヴァーサイン距離で合計。

    Args:
        path: waypoint CSV ファイルパス。

    Returns:
        (総距離[m], [(x_like,y_like),...]) のタプル。点列は可視化等の補助用。
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"Waypoint CSV not found: {path}")

    rows: List[Dict[str, str]] = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        return 0.0, []

    has_x = "x" in rows[0] and "y" in rows[0]
    has_lonlat = ("lon" in rows[0] and "lat" in rows[0]) or ("lat" in rows[0] and "lon" in rows[0])

    if has_x:
        pts = [(float(r.get("x", 0.0)), float(r.get("y", 0.0))) for r in rows]
        return _euclid_len(pts), pts

    if has_lonlat:
        # 並びはどちらでも対応（latitude/longitude 別名対応もここで吸収）
        lats = [float(r.get("lat", r.get("latitude", 0.0)) or 0.0) for r in rows]
        lons = [float(r.get("lon", r.get("longitude", 0.0)) or 0.0) for r in rows]
        pts = list(zip(lons, lats))  # 可視化では (x:lon, y:lat) で扱うことが多い
        total_m = 0.0
        for i in range(1, len(lats)):
            total_m += _haversine_m(lats[i - 1], lons[i - 1], lats[i], lons[i])
        return total_m, pts

    raise ValueError(f"Waypoint CSV must have either (x,y) or (lat,lon)/(lon,lat): {path}")


# ============================================================
# 入力ローダ
# ============================================================

def load_nodes_csv(path: str) -> Dict[str, Tuple[float, float]]:
    """nodes.csv を読み込む。id,lat,lon 形式。"""
    nodes: Dict[str, Tuple[float, float]] = {}
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            nid = row["id"]
            lat = float(row.get("lat", 0.0) or 0.0)
            lon = float(row.get("lon", 0.0) or 0.0)
            nodes[nid] = (lat, lon)
    return nodes


def _parse_bool(value: Any) -> bool:
    """0/1, true/false, yes/no などを bool に変換。"""
    v = str(value).strip().lower()
    return v in ("1", "true", "t", "yes", "y")


def load_edges_csv(path: str) -> List[Dict[str, Any]]:
    """edges.csv を読み込む。source,target,segment_id,reversible 形式（waypoint_list でも可）。"""
    edges: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            src = row["source"].strip()
            tgt = row["target"].strip()
            seg = row.get("segment_id") or row.get("waypoint_list")
            if not seg:
                raise ValueError("edges.csv: 'segment_id' or 'waypoint_list' is required.")
            rev = _parse_bool(row.get("reversible", "0"))
            edges.append({"source": src, "target": tgt, "segment_id": seg, "reversible": rev})
    return edges


# ============================================================
# Dijkstra / 経路復元
# ============================================================

def dijkstra_single_source(G: nx.Graph, source: str) -> Tuple[Dict[str, float], Dict[str, Optional[str]]]:
    """1始点 Dijkstra で全ノードへの最短距離と前駆ノードを計算。"""
    dist: Dict[str, float] = {node: float("inf") for node in G.nodes}
    prev: Dict[str, Optional[str]] = {node: None for node in G.nodes}
    dist[source] = 0.0
    heap: List[Tuple[float, str]] = [(0.0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        for v in G.neighbors(u):
            w = G[u][v]["weight"]
            alt = d + w
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(heap, (alt, v))
    return dist, prev


def reconstruct_path(prev: Dict[str, Optional[str]], source: str, target: str) -> List[str]:
    """Dijkstra結果から source→target の経路（ノード列）を復元。"""
    path: List[str] = []
    u = target
    if prev.get(u) is None and u != source:
        return []  # 到達不能
    while u is not None:
        path.append(u)
        if u == source:
            break
        u = prev[u]
    path.reverse()
    return path


def compute_terminals_shortest_paths(
    G: nx.Graph, terminals: List[str]
) -> Tuple[Dict[Tuple[int, int], float], Dict[Tuple[int, int], List[str]]]:
    """端点集合間の最短距離と経路を計算。"""
    k = len(terminals)
    dist_matrix: Dict[Tuple[int, int], float] = {}
    path_table: Dict[Tuple[int, int], List[str]] = {}

    for i, t in enumerate(terminals):
        dist, prev = dijkstra_single_source(G, t)
        for j, u in enumerate(terminals):
            if i == j:
                dist_matrix[(i, j)] = 0.0
                path_table[(i, j)] = [t]
            else:
                dist_matrix[(i, j)] = dist[u]
                path_table[(i, j)] = reconstruct_path(prev, t, u)
    return dist_matrix, path_table


# ============================================================
# Held-Karp (TSPパス)
# ============================================================

def held_karp_path(
    dist_matrix: Dict[Tuple[int, int], float],
    start_idx: int,
    goal_idx: int,
    k: int,
) -> Tuple[float, List[int]]:
    """Held-Karp 法で start→（中継点）→goal の最短順序を求める。"""
    dp: Dict[Tuple[int, int], Tuple[float, Optional[int]]] = {}
    dp[(1 << start_idx, start_idx)] = (0.0, None)

    for mask in range(1 << k):
        for u in range(k):
            if not (mask & (1 << u)):
                continue
            if (mask, u) not in dp:
                continue
            cost_u, _ = dp[(mask, u)]
            for v in range(k):
                if mask & (1 << v):
                    continue
                # goal は最後に入るように制約
                if v == goal_idx and mask != (1 << k) - 1 - (1 << goal_idx):
                    continue
                new_mask = mask | (1 << v)
                alt = cost_u + dist_matrix.get((u, v), float("inf"))
                if (new_mask, v) not in dp or alt < dp[(new_mask, v)][0]:
                    dp[(new_mask, v)] = (alt, u)

    full_mask = (1 << k) - 1
    if (full_mask, goal_idx) not in dp:
        return float("inf"), []

    best_cost, _ = dp[(full_mask, goal_idx)]

    # 復元
    order: List[int] = []
    mask = full_mask
    v = goal_idx
    while True:
        order.append(v)
        cost_v, u = dp[(mask, v)]
        if u is None:
            break
        mask ^= 1 << v
        v = u
    order.reverse()
    return best_cost, order


# ============================================================
# 可変ルート探索（ライブラリI/F：外部互換を維持）
# ============================================================

def solve_variable_route(
    nodes: Union[Dict[str, Tuple[float, float]], List[Dict[str, Any]]],
    edges: List[Dict[str, Any]],
    start: str,
    goal: str,
    checkpoints: List[str],
) -> Dict[str, Any]:
    """可変ルート探索。外部I/Fは固定し、戻り値は edge_sequence/node_sequence/visit_order/stats に加えて
    route_image_path を返す。

    - JSONへ直列化可能な stats のみ含める（tupleキーは "i-j" 文字列に変換）。
    - Graph オブジェクトや tupleキー版の dist_matrix/path_table は戻り値に含めない。
    - plot_route_image() を呼び出し、同一プロセス内では上書き、別プロセスなら別ファイルとして /tmp に画像を保存する。
    """
    if len(checkpoints) < 1:
        raise ValueError("checkpoints must contain at least one node")

    # nodes 正規化
    if isinstance(nodes, list):
        try:
            nodes = {str(n["id"]): (float(n["lat"]), float(n["lon"])) for n in nodes}
        except Exception as e:
            raise TypeError(f"nodes list must contain id/lat/lon: {e}")
    elif not isinstance(nodes, dict):
        raise TypeError("nodes must be dict or list-of-dict")

    # 存在チェック
    for nid in [start, goal, *checkpoints]:
        if nid not in nodes:
            raise KeyError(f"Node not found in nodes: {nid}")

    # edges 正規化
    normalized_edges: List[Dict[str, Any]] = []
    for e in edges:
        try:
            src = str(e["source"])
            tgt = str(e["target"])
        except Exception as ex:
            raise ValueError(f"edge must have source/target: {ex}")
        seg = e.get("segment_id") or e.get("waypoint_list")
        if not seg:
            raise ValueError("edge must have segment_id or waypoint_list")
        normalized_edges.append({
            "source": src,
            "target": tgt,
            "segment_id": str(seg),
            "reversible": _parse_bool(e.get("reversible", False)),
        })
    edges = normalized_edges

    # (1) 辺定義の構築と長さキャッシュ
    dir_edge: Dict[Tuple[str, str], Dict[str, Any]] = {}
    seg_len_cache: Dict[str, Tuple[float, List[Tuple[float, float]]]] = {}

    def seg_len(seg_path: str) -> float:
        if seg_path not in seg_len_cache:
            seg_len_cache[seg_path] = _load_waypoint_csv_length(seg_path)
        return seg_len_cache[seg_path][0]

    for e in edges:
        u, v, seg, rev = e["source"], e["target"], e["segment_id"], e["reversible"]
        length_m = seg_len(seg)
        dir_edge[(u, v)] = {"segment_id": seg, "reversible": bool(rev), "length": length_m}
        if rev:
            dir_edge[(v, u)] = {"segment_id": seg, "reversible": True, "length": length_m}

    # reversible=0 の対向不足チェック
    for (u, v), meta in list(dir_edge.items()):
        if not meta["reversible"] and (v, u) not in dir_edge:
            raise ValueError(f"Non-reversible edge {u}->{v} requires counterpart {v}->{u}")

    # (2) 無向グラフ構築（重みは両方向が存在すれば平均、片方向のみならその値）
    G = nx.Graph()
    for nid in nodes.keys():
        G.add_node(nid, pos=(nodes[nid][1], nodes[nid][0]))  # pos = (lon, lat)

    undirected_weight: Dict[Tuple[str, str], float] = {}
    for (u, v), meta in dir_edge.items():
        a, b = sorted((u, v))
        w = meta["length"]
        if (a, b) in undirected_weight:
            undirected_weight[(a, b)] = 0.5 * (undirected_weight[(a, b)] + w)
        else:
            undirected_weight[(a, b)] = w
    for (a, b), w in undirected_weight.items():
        G.add_edge(a, b, weight=w)

    # (3) 端点集合
    terminals: List[str] = [start] + list(checkpoints) + [goal]
    k = len(terminals)

    # (4) 端点間最短経路
    dist_matrix, path_table = compute_terminals_shortest_paths(G, terminals)

    # 到達不能チェック
    unreachable: List[Tuple[str, str]] = []
    for i in range(k):
        for j in range(k):
            if dist_matrix[(i, j)] == float("inf"):
                unreachable.append((terminals[i], terminals[j]))
    if unreachable:
        pairs = ", ".join([f"{a}->{b}" for a, b in unreachable])
        raise RuntimeError(f"Some terminal pairs are unreachable in the graph: {pairs}")

    # (5) Held-Karp による順序最適化
    start_idx, goal_idx = 0, k - 1
    best_cost, order = held_karp_path(dist_matrix, start_idx, goal_idx, k)
    if not order or math.isinf(best_cost):
        raise RuntimeError("Failed to find a feasible route order (Held-Karp returned INF or empty order)")

    # (6) 経路復元（ノード列・訪問順）
    node_sequence: List[str] = []
    visit_order: List[str] = []
    for i in range(len(order) - 1):
        a_idx, b_idx = order[i], order[i + 1]
        a, b = terminals[a_idx], terminals[b_idx]
        path_nodes = path_table[(a_idx, b_idx)]
        if not path_nodes:
            raise RuntimeError(f"No path between terminals: {a} -> {b}")
        if i == 0:
            node_sequence.extend(path_nodes)
        else:
            node_sequence.extend(path_nodes[1:])
        visit_order.append(a)
    visit_order.append(terminals[order[-1]])

    # (7) edge_sequence 生成（実際に辿る向きでセグメント・向きを付与）
    edge_sequence: List[Dict[str, Any]] = []
    for i in range(1, len(node_sequence)):
        u, v = node_sequence[i - 1], node_sequence[i]
        if (u, v) in dir_edge:
            meta = dir_edge[(u, v)]
            edge_sequence.append({
                "source": u,
                "target": v,
                "segment_id": meta["segment_id"],
                "direction": "forward",
                "weight": meta["length"],
            })
        elif (v, u) in dir_edge:
            meta = dir_edge[(v, u)]
            if not meta["reversible"]:
                raise ValueError(f"Edge {u}->{v} is not allowed (non-reversible segment only in opposite direction)")
            edge_sequence.append({
                "source": u,
                "target": v,
                "segment_id": meta["segment_id"],
                "direction": "reverse",
                "weight": meta["length"],
            })
        else:
            raise ValueError(f"No edge row for {u}<->{v} in edges.csv (neither direction present)")

    # JSON 直列化可能な統計（tupleキーは文字列化）
    stats = {
        "terminals": terminals,
        "order_indices": order,
        "best_cost": best_cost,
        "num_nodes": G.number_of_nodes(),
        "num_edges": G.number_of_edges(),
        "dist_matrix": {f"{i}-{j}": d for (i, j), d in dist_matrix.items()},
        "path_table": {f"{i}-{j}": path_table[(i, j)] for (i, j) in path_table},
    }

    # (8) ルート画像を /tmp に出力し、そのパスを戻り値に含める
    image_path = plot_route_image(nodes, node_sequence, visit_order)  # out_path=Noneで /tmp/route_solver_<pid>.png に保存

    # 外部I/F（route_planner等）向けの戻り値
    return {
        "edge_sequence": edge_sequence,
        "node_sequence": node_sequence,
        "visit_order": visit_order,
        "stats": stats,
        "route_image_path": image_path,  # 追加：画像ファイルパス
    }


# ============================================================
# 可視化（地図背景付き）
# ============================================================

def plot_route_image(
    nodes: Dict[str, Tuple[float, float]],
    node_sequence: List[str],
    visit_order: List[str],
    out_path: Optional[str] = None,
) -> str:
    """地図背景付きで経路と訪問順表を描画する。

    Args:
        nodes: {id: (lat, lon)} ノード集合
        node_sequence: 経路ノード列
        visit_order: 端点訪問順
        out_path: 保存先パス。None の場合は /tmp/route_solver_<pid>.png に保存

    Returns:
        実際に保存したファイルのパス
    """
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    import os

    # 背景地図
    img = plt.imread(MAP_IMAGE_PATH)
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(ncols=2, nrows=1, width_ratios=[2.5, 1.0])
    ax_map = fig.add_subplot(gs[0, 0])
    ax_table = fig.add_subplot(gs[0, 1])
    ax_map.imshow(img, extent=[MAP_LON_MIN, MAP_LON_MAX, MAP_LAT_MIN, MAP_LAT_MAX], origin="upper")
    ax_map.set_title("Variable Route")

    # ノード座標（lon,lat）辞書
    pos = {nid: (lon, lat) for nid, (lat, lon) in nodes.items()}

    # 全ノードを灰色で描画、IDラベルを表示
    G = nx.Graph()
    for nid in nodes.keys():
        G.add_node(nid)
    nx.draw_networkx_nodes(G, pos, node_size=50, node_color="lightgray", ax=ax_map)
    nx.draw_networkx_labels(G, pos, labels={nid: nid for nid in G.nodes()}, font_size=12, ax=ax_map)

    # 経路を青矢印で描画
    for u, v in zip(node_sequence[:-1], node_sequence[1:]):
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        ax_map.annotate(
            "",
            xy=(x2, y2), xycoords="data",
            xytext=(x1, y1), textcoords="data",
            arrowprops=dict(arrowstyle="->", color="blue", lw=2),
        )

    # 重要ノードを強調（start, goal, via）
    if visit_order:
        color_map = {"start": "green", "goal": "red", "via": "orange"}
        start_id = visit_order[0]
        goal_id = visit_order[-1]
        via_ids = visit_order[1:-1]
        if start_id in pos:
            nx.draw_networkx_nodes(G, pos, nodelist=[start_id],
                                   node_size=260, node_color=color_map["start"],
                                   ax=ax_map, linewidths=1.2, edgecolors="black")
        if goal_id in pos:
            nx.draw_networkx_nodes(G, pos, nodelist=[goal_id],
                                   node_size=260, node_color=color_map["goal"],
                                   ax=ax_map, linewidths=1.2, edgecolors="black")
        if via_ids:
            nx.draw_networkx_nodes(G, pos, nodelist=via_ids,
                                   node_size=260, node_color=color_map["via"],
                                   ax=ax_map, linewidths=1.2, edgecolors="black")

    # 訪問順・距離の表を作成
    table_data: List[List[Any]] = []
    headers = ["訪問順", "ノードID", "区間距離(m)", "累積距離(m)"]
    cumulative = 0.0
    for i, nid in enumerate(node_sequence):
        seg_dist = 0.0
        if i > 0:
            lat1, lon1 = nodes[node_sequence[i - 1]]
            lat2, lon2 = nodes[nid]
            seg_dist = _haversine_m(lat1, lon1, lat2, lon2)
        cumulative += seg_dist
        table_data.append([i + 1, nid, f"{seg_dist:.1f}", f"{cumulative:.1f}"])

    ax_table.axis("off")
    table = ax_table.table(cellText=table_data, colLabels=headers, loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.2)

    # 凡例
    legend_handles = [
        Patch(facecolor="green", edgecolor="black", label="出発地"),
        Patch(facecolor="orange", edgecolor="black", label="経由地"),
        Patch(facecolor="red", edgecolor="black", label="到着地"),
        Patch(facecolor="lightgray", edgecolor="black", label="その他"),
        Line2D([0], [0], color="blue", lw=2, label="経路"),
    ]
    ax_map.legend(handles=legend_handles, loc="upper left", framealpha=0.95)

    # 軸設定
    ax_map.set_xlim(MAP_LON_MIN, MAP_LON_MAX)
    ax_map.set_ylim(MAP_LAT_MIN, MAP_LAT_MAX)
    ax_map.set_xlabel("Longitude")
    ax_map.set_ylabel("Latitude")

    plt.tight_layout()

    # ファイルパスの決定
    if out_path is None:
        pid = os.getpid()
        out_path = f"/tmp/route_solver_{pid}.png"

    # 画像保存
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_terminal_complete_graph(
    terminals: List[str],
    dist_matrix: Dict[Tuple[int, int], float],
    nodes: Dict[str, Tuple[float, float]],
    out_path: Optional[str] = None,
    terminal_order: Optional[List[int]] = None,
) -> None:
    """地図背景付きで端点完全グラフを描画し、最適経路のエッジを強調する。

    Args:
        terminals: 重要ノード（start + checkpoints + goal）
        dist_matrix: タプルキー版の端点間距離（(i,j)→距離）
        nodes: 全ノードの座標 {id: (lat,lon)}
        out_path: 画像保存パス
        terminal_order: Held-Karp が返した端点順序（例: [0,2,3,1]）
    """
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D

    # 有向グラフ（端点完全グラフ）
    H = nx.DiGraph()
    pos = {nid: (lon, lat) for nid, (lat, lon) in nodes.items()}
    for nid in terminals:
        H.add_node(nid)
    k = len(terminals)
    for i, u in enumerate(terminals):
        for j, v in enumerate(terminals):
            if i == j:
                continue
            d = dist_matrix.get((i, j), math.inf)
            if d == math.inf:
                continue
            H.add_edge(u, v, weight=d)

    # 背景地図
    img = plt.imread(MAP_IMAGE_PATH)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img, extent=[MAP_LON_MIN, MAP_LON_MAX, MAP_LAT_MIN, MAP_LAT_MAX], origin="upper")

    # 全エッジを薄く描画
    nx.draw_networkx_edges(H, pos, alpha=0.2, edge_color="gray", ax=ax)

    # ノードを灰色で描画＋ラベル
    nx.draw_networkx_nodes(H, pos, node_size=150, node_color="lightgray", ax=ax)
    nx.draw_networkx_labels(H, pos, font_size=8, ax=ax)

    # 最適経路エッジを強調表示
    if terminal_order and len(terminal_order) >= 2:
        path_edges = []
        for a_idx, b_idx in zip(terminal_order[:-1], terminal_order[1:]):
            u = terminals[a_idx]
            v = terminals[b_idx]
            if H.has_edge(u, v):
                path_edges.append((u, v))
        nx.draw_networkx_edges(
            H, pos, edgelist=path_edges, width=2.5, edge_color="magenta", ax=ax, arrows=True
        )

    # 重要ノードを強調（start/goal/via）
    if terminals:
        start_id = terminals[0]
        goal_id = terminals[-1]
        via_ids = terminals[1:-1]
        if start_id in pos:
            nx.draw_networkx_nodes(H, pos, nodelist=[start_id],
                                   node_size=300, node_color="green",
                                   ax=ax, linewidths=1.2, edgecolors="black")
        if goal_id in pos:
            nx.draw_networkx_nodes(H, pos, nodelist=[goal_id],
                                   node_size=300, node_color="red",
                                   ax=ax, linewidths=1.2, edgecolors="black")
        if via_ids:
            nx.draw_networkx_nodes(H, pos, nodelist=via_ids,
                                   node_size=300, node_color="orange",
                                   ax=ax, linewidths=1.2, edgecolors="black")

    # 凡例
    legend_handles = [
        Patch(facecolor="green", edgecolor="black", label="出発地"),
        Patch(facecolor="orange", edgecolor="black", label="経由地"),
        Patch(facecolor="red", edgecolor="black", label="到着地"),
        Line2D([0], [0], color="magenta", lw=2, label="最適経路"),
    ]
    ax.legend(handles=legend_handles, loc="upper left", framealpha=0.95)

    # 軸と範囲
    ax.set_xlim(MAP_LON_MIN, MAP_LON_MAX)
    ax.set_ylim(MAP_LAT_MIN, MAP_LAT_MAX)
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("Terminal Complete Graph")

    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=150)
    plt.close(fig)


# ============================================================
# CLI
# ============================================================

def _rebuild_graph_for_terminals(
    nodes: Dict[str, Tuple[float, float]],
    edges: List[Dict[str, Any]],
) -> nx.Graph:
    """solve_variable_route と同じルールで無向グラフを再構築（CLIの描画用）。"""
    # edges を solve と同様に正規化
    normalized_edges: List[Dict[str, Any]] = []
    for e in edges:
        seg = e.get("segment_id") or e.get("waypoint_list")
        normalized_edges.append({
            "source": str(e["source"]),
            "target": str(e["target"]),
            "segment_id": str(seg),
            "reversible": _parse_bool(e.get("reversible", False)),
        })

    # まず両方向の「有向エッジ」集合を作り、のちに無向化（両方向があれば平均）
    dir_edge: Dict[Tuple[str, str], Dict[str, Any]] = {}
    for e in normalized_edges:
        u, v, seg, rev = e["source"], e["target"], e["segment_id"], e["reversible"]
        length_m = _load_waypoint_csv_length(seg)[0]
        dir_edge[(u, v)] = {"segment_id": seg, "reversible": bool(rev), "length": length_m}
        if rev:
            dir_edge[(v, u)] = {"segment_id": seg, "reversible": True, "length": length_m}

    # 無向化（solve と同等の重みルール）
    G = nx.Graph()
    for nid in nodes.keys():
        G.add_node(nid, pos=(nodes[nid][1], nodes[nid][0]))
    undirected_weight: Dict[Tuple[str, str], float] = {}
    for (u, v), meta in dir_edge.items():
        a, b = sorted((u, v))
        w = meta["length"]
        if (a, b) in undirected_weight:
            undirected_weight[(a, b)] = 0.5 * (undirected_weight[(a, b)] + w)
        else:
            undirected_weight[(a, b)] = w
    for (a, b), w in undirected_weight.items():
        G.add_edge(a, b, weight=w)

    return G


def main() -> None:
    parser = argparse.ArgumentParser(description="Variable-route solver (TSP path over undirected graph)")
    parser.add_argument("--nodes", required=True, help="Path to nodes.csv (columns: id,lat,lon)")
    parser.add_argument("--edges", required=True, help="Path to edges.csv (columns: source,target,segment_id|waypoint_list,reversible)")
    parser.add_argument("--start", required=True, help="Start node id")
    parser.add_argument("--goal", required=True, help="Goal node id")
    parser.add_argument("--checkpoints", required=True, help="Comma-separated checkpoint node ids (>=1)")
    parser.add_argument("--outdir", default="out", help="Output directory")
    parser.add_argument("--json", dest="json_path", default=None, help="Optional output JSON path (default: out/route.json)")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    nodes = load_nodes_csv(args.nodes)
    edges = load_edges_csv(args.edges)
    cps = [s for s in args.checkpoints.split(",") if s]
    if len(cps) < 1:
        raise SystemExit("checkpoints must contain at least one node")

    # ライブラリI/F（外部互換）を使用
    result = solve_variable_route(nodes, edges, args.start, args.goal, cps)

    # 画像出力 1: 経路（地図背景）
    img_path = os.path.join(args.outdir, "variable_route.png")
    plot_route_image(nodes, result["node_sequence"], result["visit_order"], img_path)

    # 画像出力 2: 端点完全グラフ
    terminals = result["stats"]["terminals"]
    G = _rebuild_graph_for_terminals(nodes, edges)  # solve と同じ重みルールで再構築
    dist_matrix_tuple, _ = compute_terminals_shortest_paths(G, terminals)

    complete_img_path = os.path.join(args.outdir, "terminal_complete.png")
    plot_terminal_complete_graph(
        terminals,
        dist_matrix_tuple,
        nodes,
        complete_img_path,
        terminal_order=result["stats"]["order_indices"],  # ★ Held-Karp の順序を渡す
    )

    # JSON 出力（tupleキーは含まないのでそのまま直列化できる）
    json_path = args.json_path or os.path.join(args.outdir, "route.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    # 標準出力（簡易統計）
    print("== STATS ==")
    print(f"best_cost: {result['stats']['best_cost']:.3f}")
    print(f"num_nodes: {result['stats']['num_nodes']}")
    print(f"num_edges: {result['stats']['num_edges']}")
    print("terminals:", ", ".join(result["stats"]["terminals"]))
    print("visit_order:", " -> ".join(result["visit_order"]))
    print(f"JSON saved to: {json_path}")
    print(f"Images saved to: {img_path}, {complete_img_path}")


if __name__ == "__main__":
    main()

