#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Waypoint 地図連携エディタ（Tkinter + matplotlib）

本スクリプトは、詳細設計 v1.2 に基づく完全実装です。
- 地図（PNG+PGW）上に CSV の waypoint を重畳表示
- 2モード：ナビゲーション（俯瞰）／フォーカス（単点編集）
- フォーカス：短辺 ±25 m 自動ズーム、位置 0.25 m シフト、向き ±5° 回転、フラグ編集、削除、保存（上書き可）
- 座標変換は必ず付属の `latlon_to_pixel_mapper.py` の API を呼び出して行う（本スクリプトは新規変換を実装しない）
- x,y が ENU と一致しない可能性があるため、(lat,lon)→ローカルメートル座標 (E,N) と CSV の (x,y) の相似変換（s,R,t）を自動推定し、一貫して更新する
- Google Python Style Guide 準拠、型ヒント付き、日本語コメント

使い方:
    python waypoint_editor.py --csv /path/to/fixed_first.csv --map /path/to/map.png --pgw /path/to/map.pgw

必要パッケージ:
    pip install pandas numpy matplotlib
    （Tkinter は OS の標準で提供されることが多い）
"""
from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import math
import sys
from collections import OrderedDict
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np
import pandas as pd

# Tkinter / matplotlib 埋め込み
import tkinter as tk
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib import image as mpimg

# --- 重要 ---
# 座標変換は既存 mapper モジュール（latlon_to_pixel_mapper.py）を利用する。
# 実行場所に依らず確実に import できるよう経路を調整する。


def _import_mapper_module() -> Any:
    """latlon_to_pixel_mapper モジュールをロードして返す。"""

    import importlib

    candidates = [
        "route_planner.route_planner.latlon_to_pixel_mapper",
        str((Path(__file__).resolve().parents[1] / "route_planner")),
        "/mnt/data",
    ]

    # 1. パッケージ経由
    try:
        return importlib.import_module(candidates[0])
    except Exception:
        pass

    # 2. スクリプト相対 / 追加候補を sys.path へ入れて順に import
    for p in candidates[1:]:
        if p not in sys.path:
            sys.path.insert(0, p)
        try:
            return importlib.import_module("latlon_to_pixel_mapper")
        except Exception:
            continue

    print(
        "ERROR: latlon_to_pixel_mapper.py を import できません。route_planner/route_planner/ 配下を確認してください。",
        file=sys.stderr,
    )
    raise ImportError("latlon_to_pixel_mapper import failed")


mapper_mod = _import_mapper_module()


# =============================================================================
# 低レベル補助（数学・姿勢・描画スケール）
# =============================================================================

def clamp(v: float, vmin: float, vmax: float) -> float:
    """値を [vmin, vmax] にクランプする。"""
    return max(vmin, min(v, vmax))


def rad2deg(rad: float) -> float:
    """ラジアン→度。"""
    return rad * 180.0 / math.pi


def deg2rad(deg: float) -> float:
    """度→ラジアン。"""
    return deg * math.pi / 180.0


def normalize_angle_rad(rad: float) -> float:
    """角度（ラジアン）を (-pi, pi] に正規化。"""
    a = (rad + math.pi) % (2.0 * math.pi)
    if a <= 0:
        a += 2.0 * math.pi
    return a - math.pi


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """四元数（x,y,z,w）→ yaw[rad]（Z回頭、反時計回り正、0は+X）。"""
    # ROS 標準の ZYX (yaw-pitch-roll) に基づく yaw 抽出
    # 参考: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return normalize_angle_rad(yaw)


def yaw_to_quat(yaw_rad: float) -> Tuple[float, float, float, float]:
    """yaw[rad] → 四元数（x,y,z,w）。Z回頭のみ。"""
    half = 0.5 * yaw_rad
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    # 正規化（数値誤差対策）
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def ensure_int_if_integer_stringish(v: Any) -> Any:
    """'12' や 12.0 のような値を可能なら int に直す（ラベル表示用途）。"""
    try:
        if isinstance(v, (int, np.integer)):
            return int(v)
        if isinstance(v, float) and float(v).is_integer():
            return int(v)
        if isinstance(v, str) and v.strip() != "":
            fv = float(v)
            if fv.is_integer():
                return int(fv)
    except Exception:
        pass
    return v


# =============================================================================
# MapAdapter: 既存 mapper の API 差異を吸収するラッパ
# =============================================================================

class MapAdapter:
    """ユーザ提供の mapper モジュール（latlon_to_pixel_mapper.py）の API を叩き分ける薄いラッパ。

    重要：
      - 本クラスは新規の座標変換を実装しない。
      - 可能な限り既存API（関数名の揺れに対応）を呼び出す。
      - 必要な API が無い場合は起動時に明確なエラーを出す。
    """

    def __init__(self, module: Any) -> None:
        self._m = module
        # 代表的な関数候補を列挙（実プロジェクトでの関数名の揺れを吸収）
        self._latlon_to_pixel = self._resolve(
            [
                "latlon_to_pixel",
                "latlon_to_pixel_xy",
                "latlon2pixel",
                "latlon_to_imgxy",
            ]
        )
        self._pixel_to_latlon = self._resolve(
            [
                "pixel_to_latlon",
                "pixelxy_to_latlon",
                "pixel2latlon",
                "imgxy_to_latlon",
            ]
        )
        # ローカルメートル座標（E,N）との相互変換（あれば使用）
        self._latlon_to_local = self._optional(
            [
                "latlon_to_local_meters",
                "latlon_to_local",
                "latlon_to_en",
                "latlon_to_xy_local",
            ]
        )
        self._local_to_latlon = self._optional(
            [
                "local_meters_to_latlon",
                "local_to_latlon",
                "en_to_latlon",
                "xy_local_to_latlon",
            ]
        )

        # map/pgw の設定 API（あれば初期化）
        if hasattr(self._m, "load_map"):
            # 推奨: mapper_mod.load_map(map_png_path, pgw_path)
            self._load_map = getattr(self._m, "load_map")
        else:
            self._load_map = None  # pragma: no cover

        self._map_path: Optional[str] = None
        self._pgw_path: Optional[str] = None

    def _resolve(self, names: Sequence[str]) -> Callable:
        """必須 API を名前候補から解決。存在しなければ例外。"""
        for n in names:
            if hasattr(self._m, n):
                return getattr(self._m, n)
        raise RuntimeError(
            f"mapper API 不足: 必須の {names} のいずれかが見つかりません。latlon_to_pixel_mapper.py を確認してください。"
        )

    def _optional(self, names: Sequence[str]) -> Optional[Callable]:
        """任意 API を名前候補から解決。見つからなければ None。"""
        for n in names:
            if hasattr(self._m, n):
                return getattr(self._m, n)
        return None

    def load_map(self, map_path: Union[str, Path], pgw_path: Union[str, Path]) -> None:
        """マップ・ワールドファイルのロード（mapper 側に API がある場合のみ呼ぶ）。"""
        self._map_path = str(map_path)
        self._pgw_path = str(pgw_path)
        if self._load_map is not None:
            self._load_map(self._map_path, self._pgw_path)
        else:
            # load_map が無い場合でも、新APIであれば latlon_to_pixel のキャッシュ用途に空呼出しを行う
            try:
                self._latlon_to_pixel(0.0, 0.0)  # type: ignore[arg-type]
            except TypeError:
                # 旧APIのみの場合は、ダミーでキャッシュを作る
                if self._map_path is not None and self._pgw_path is not None:
                    try:
                        self._latlon_to_pixel(self._map_path, self._pgw_path, [(0.0, 0.0)])  # type: ignore[arg-type]
                    except Exception:
                        pass

    def latlon_to_pixel(self, lat: float, lon: float) -> Tuple[Optional[float], Optional[float]]:
        """(lat,lon) → (px,py)。画像外は None を返す。"""
        try:
            px, py = self._latlon_to_pixel(float(lat), float(lon))  # type: ignore[assignment]
        except TypeError:
            if self._map_path is None or self._pgw_path is None:
                raise RuntimeError("load_map() を先に実行してください。")
            res = self._latlon_to_pixel(self._map_path, self._pgw_path, [(float(lat), float(lon))])  # type: ignore[arg-type]
            if not res:
                return None, None
            px, py = res[0]
        if px is None or py is None:
            return None, None
        return float(px), float(py)

    def pixel_to_latlon(self, px: float, py: float) -> Tuple[float, float]:
        """(px,py) → (lat,lon)。"""
        try:
            return tuple(self._pixel_to_latlon(float(px), float(py)))  # type: ignore[return-value]
        except TypeError:
            if self._map_path is None or self._pgw_path is None:
                raise RuntimeError("load_map() を先に実行してください。")
            return tuple(
                self._pixel_to_latlon(float(px), float(py), image_path=self._map_path, worldfile_path=self._pgw_path)  # type: ignore[arg-type, return-value]
            )

    def latlon_to_local_m(self, lat: float, lon: float) -> Tuple[float, float]:
        """(lat,lon) → (E,N) [m]。APIが無ければ例外。"""
        if self._latlon_to_local is None:
            raise RuntimeError("mapper に latlon_to_local_meters 相当のAPIが見つかりません。")
        return tuple(self._latlon_to_local(float(lat), float(lon)))  # type: ignore[return-value]

    def local_m_to_latlon(self, e: float, n: float) -> Tuple[float, float]:
        """(E,N) [m] → (lat,lon)。APIが無ければ例外。"""
        if self._local_to_latlon is None:
            raise RuntimeError("mapper に local_meters_to_latlon 相当のAPIが見つかりません。")
        return tuple(self._local_to_latlon(float(e), float(n)))  # type: ignore[return-value]


# =============================================================================
# XYAligner: (E,N) と (x,y) の 2D 相似変換（s,R,t）をロバストに推定
# =============================================================================

@dataclasses.dataclass
class Similarity2D:
    """2D 相似変換 Q ≈ s R P + t を表現。"""
    s: float
    R: np.ndarray  # shape=(2,2)
    t: np.ndarray  # shape=(2,)

    def apply(self, p: np.ndarray) -> np.ndarray:
        """P(2,) or P(N,2) に適用して Q を返す。"""
        if p.ndim == 1:
            return self.s * (self.R @ p) + self.t
        return (self.s * (p @ self.R.T)) + self.t  # (N,2)


class XYAligner:
    """(E,N) と (x,y) の相似変換を RANSAC + Procrustes で推定し、残差 RMS を返す。"""

    def __init__(self) -> None:
        self._sim: Optional[Similarity2D] = None
        self._rms_m: Optional[float] = None

    @property
    def has_model(self) -> bool:
        return self._sim is not None

    @property
    def rms(self) -> Optional[float]:
        return self._rms_m

    @staticmethod
    def _procrustes(P: np.ndarray, Q: np.ndarray) -> Similarity2D:
        """最小二乗 Procrustes（相似）で Q ≈ sRP + t を推定。"""
        # 中心化
        mu_P = P.mean(axis=0)
        mu_Q = Q.mean(axis=0)
        P0 = P - mu_P
        Q0 = Q - mu_Q

        # 回転・スケール推定 (P0 -> Q0)
        H = P0.T @ Q0  # (2,2)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            # 反転を回避
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        # スケール
        varP = (P0 ** 2).sum()
        s = (S.sum()) / varP if varP > 1e-12 else 1.0
        # 並進
        t = mu_Q - s * (R @ mu_P)
        return Similarity2D(s=float(s), R=R.astype(float), t=t.astype(float))

    @staticmethod
    def _rms_residual(sim: Similarity2D, P: np.ndarray, Q: np.ndarray) -> float:
        """RMS 残差（m）。"""
        pred = sim.apply(P)
        diff = pred - Q
        return float(np.sqrt((diff ** 2).sum(axis=1).mean()))

    def fit(
        self,
        EN: np.ndarray,
        XY: np.ndarray,
        ransac_iter: int = 200,
        inlier_thresh_m: float = 0.5,
        min_points: int = 4,
    ) -> Tuple[Optional[Similarity2D], Optional[float], np.ndarray]:
        """RANSAC + Procrustes でモデル推定。
        Args:
            EN: shape=(N,2) の (E,N) [m]
            XY: shape=(N,2) の (x,y)
        Returns:
            (Similarity2D or None, rms or None, inlier mask[bool])
        """
        n = min(len(EN), len(XY))
        if n < min_points:
            self._sim, self._rms_m = None, None
            return None, None, np.zeros(n, dtype=bool)

        rng = np.random.default_rng(12345)
        best_sim: Optional[Similarity2D] = None
        best_mask: Optional[np.ndarray] = None
        best_rms: float = float("inf")

        idx_all = np.arange(n)
        for _ in range(ransac_iter):
            # サンプル抽出（4点）
            idx = rng.choice(idx_all, size=min_points, replace=False)
            P = EN[idx]
            Q = XY[idx]
            try:
                sim = self._procrustes(P, Q)
            except Exception:
                continue
            # 残差
            pred = sim.apply(EN)
            err = np.linalg.norm(pred - XY, axis=1)
            mask = err <= inlier_thresh_m
            m = mask.sum()
            if m < min_points:
                continue
            # インライアで再推定
            try:
                sim2 = self._procrustes(EN[mask], XY[mask])
            except Exception:
                continue
            rms = self._rms_residual(sim2, EN[mask], XY[mask])
            if rms < best_rms:
                best_rms = rms
                best_sim = sim2
                best_mask = mask

        # ベストがなければ全点で素直に推定
        if best_sim is None:
            try:
                sim = self._procrustes(EN, XY)
                rms = self._rms_residual(sim, EN, XY)
                self._sim, self._rms_m = sim, rms
                return sim, rms, np.ones(n, dtype=bool)
            except Exception:
                self._sim, self._rms_m = None, None
                return None, None, np.zeros(n, dtype=bool)

        self._sim, self._rms_m = best_sim, best_rms
        return best_sim, best_rms, best_mask  # type: ignore[return-value]

    def en_to_xy(self, e: float, n: float) -> Tuple[float, float]:
        """(E,N)→(x,y)。モデルが無い場合は恒等を返す。"""
        if self._sim is None:
            return float(e), float(n)
        v = np.asarray([e, n], dtype=float)
        q = self._sim.apply(v)
        return float(q[0]), float(q[1])


# =============================================================================
# WaypointStore: CSV の読み書き・検証・列名検出
# =============================================================================

class WaypointStore:
    """CSV の読込・検証・保存と、行データの読み書きを管理する。"""

    # フラグ列（固定 5）
    FLAG_RIGHT_OPEN = "right_is_open"
    FLAG_LEFT_OPEN = "left_is_open"
    FLAG_LINE_STOP = "line_is_stop"
    FLAG_SIGNAL_STOP = "signal_is_stop"
    FLAG_ISNOT_SKIP = "isnot_skipnum"

    REQUIRED_FLAGS = [
        FLAG_RIGHT_OPEN,
        FLAG_LEFT_OPEN,
        FLAG_LINE_STOP,
        FLAG_SIGNAL_STOP,
        FLAG_ISNOT_SKIP,
    ]

    def __init__(self, csv_path: Union[str, Path]) -> None:
        self.csv_path = Path(csv_path)
        if not self.csv_path.exists():
            raise FileNotFoundError(f"CSV が見つかりません: {self.csv_path}")

        df = pd.read_csv(self.csv_path)
        if df.empty:
            raise ValueError("CSV が空です。")
        # 列名の検出
        cols = list(df.columns)

        def must(name: str) -> None:
            if name not in df.columns:
                raise KeyError(f"必須列がありません: {name}")

        must("latitude")
        must("longitude")
        must("x")
        must("y")
        must("z")
        for f in self.REQUIRED_FLAGS:
            must(f)
        # 四元数列名検出
        q_candidates = [
            ("q1", "q2", "q3", "q4"),
            ("qx", "qy", "qz", "qw"),
            ("quat_x", "quat_y", "quat_z", "quat_w"),
        ]
        self.q_cols: Optional[Tuple[str, str, str, str]] = None
        for cand in q_candidates:
            if all(c in cols for c in cand):
                self.q_cols = cand
                break
        if self.q_cols is None:
            raise KeyError("四元数列が見つかりません（例: q1,q2,q3,q4 または qx,qy,qz,qw）。")

        # DataFrame は列順維持のため元のまま持つ
        self.df = df
        self._changed: bool = False
        self._backup_made: bool = False
        self._change_labels: "OrderedDict[str, str]" = OrderedDict()

    # ---- DataFrame ユーティリティ ----

    @property
    def n(self) -> int:
        return int(len(self.df))

    @property
    def is_changed(self) -> bool:
        return self._changed

    def mark_changed(self) -> None:
        self._changed = True

    def _record_change_label(self, label: str, action: str = "更新") -> None:
        """変更の対象ラベルと種別を履歴に記録する。"""
        self._changed = True
        current = self._change_labels.get(label)
        if action == "削除":
            self._change_labels[label] = action
            return
        if current == "削除":
            # 既に削除が記録されていれば上書きしない。
            return
        if current is None:
            self._change_labels[label] = action

    def _record_change_idx(self, idx: int, action: str = "更新") -> None:
        label = self.get_label_display(idx)
        self._record_change_label(label, action)

    def get_row(self, idx: int) -> pd.Series:
        return self.df.iloc[idx]

    def delete_row(self, idx: int) -> None:
        label = self.get_label_display(idx)
        self._record_change_label(label, "削除")
        self.df = self.df.drop(self.df.index[idx]).reset_index(drop=True)

    def iter_latlon(self) -> Iterable[Tuple[float, float]]:
        for _, r in self.df.iterrows():
            yield float(r["latitude"]), float(r["longitude"])

    # ---- 値の読み書き ----

    def get_label_display(self, idx: int) -> str:
        """ラベル表示（数値は整数化、文字列はそのまま）。"""
        val = self.df.iloc[idx].get("label", "")
        val2 = ensure_int_if_integer_stringish(val)
        return str(val2)

    def read_pose_yaw_deg(self, idx: int) -> float:
        """四元数から yaw[deg] を返す。"""
        qx, qy, qz, qw = self._read_quat(idx)
        yaw = quat_to_yaw(qx, qy, qz, qw)
        return rad2deg(yaw)

    def write_pose_yaw_delta_deg(self, idx: int, ddeg: float) -> None:
        """yaw を ±ddeg だけ回して四元数を更新。"""
        qx, qy, qz, qw = self._read_quat(idx)
        yaw = quat_to_yaw(qx, qy, qz, qw)
        yaw2 = normalize_angle_rad(yaw + deg2rad(ddeg))
        qx2, qy2, qz2, qw2 = yaw_to_quat(yaw2)
        self._write_quat(idx, qx2, qy2, qz2, qw2)
        self._record_change_idx(idx)

    def read_latlon(self, idx: int) -> Tuple[float, float]:
        r = self.df.iloc[idx]
        return float(r["latitude"]), float(r["longitude"])

    def write_latlon(self, idx: int, lat: float, lon: float) -> None:
        self.df.iat[idx, self.df.columns.get_loc("latitude")] = float(lat)
        self.df.iat[idx, self.df.columns.get_loc("longitude")] = float(lon)
        self._record_change_idx(idx)

    def read_xy(self, idx: int) -> Tuple[float, float]:
        r = self.df.iloc[idx]
        return float(r["x"]), float(r["y"])

    def write_xy(self, idx: int, x: float, y: float) -> None:
        self.df.iat[idx, self.df.columns.get_loc("x")] = float(x)
        self.df.iat[idx, self.df.columns.get_loc("y")] = float(y)
        self._record_change_idx(idx)

    def read_z(self, idx: int) -> float:
        return float(self.df.iloc[idx]["z"])

    def read_flags(self, idx: int) -> Dict[str, Union[int, float]]:
        r = self.df.iloc[idx]
        return {
            self.FLAG_RIGHT_OPEN: float(r[self.FLAG_RIGHT_OPEN]),
            self.FLAG_LEFT_OPEN: float(r[self.FLAG_LEFT_OPEN]),
            self.FLAG_LINE_STOP: int(r[self.FLAG_LINE_STOP]),
            self.FLAG_SIGNAL_STOP: int(r[self.FLAG_SIGNAL_STOP]),
            self.FLAG_ISNOT_SKIP: int(r[self.FLAG_ISNOT_SKIP]),
        }

    def write_flags(self, idx: int, flags: Dict[str, Union[int, float]]) -> None:
        # 検証（*_is_open >= 0、他は 0/1）
        ro = float(flags[self.FLAG_RIGHT_OPEN])
        lo = float(flags[self.FLAG_LEFT_OPEN])
        ls = int(flags[self.FLAG_LINE_STOP])
        ss = int(flags[self.FLAG_SIGNAL_STOP])
        sk = int(flags[self.FLAG_ISNOT_SKIP])
        if ro < 0.0 or lo < 0.0:
            raise ValueError("*_is_open は 0 以上の数値である必要があります。")
        if ls not in (0, 1) or ss not in (0, 1) or sk not in (0, 1):
            raise ValueError("line_is_stop, signal_is_stop, isnot_skipnum は 0/1 である必要があります。")
        self.df.iat[idx, self.df.columns.get_loc(self.FLAG_RIGHT_OPEN)] = ro
        self.df.iat[idx, self.df.columns.get_loc(self.FLAG_LEFT_OPEN)] = lo
        self.df.iat[idx, self.df.columns.get_loc(self.FLAG_LINE_STOP)] = ls
        self.df.iat[idx, self.df.columns.get_loc(self.FLAG_SIGNAL_STOP)] = ss
        self.df.iat[idx, self.df.columns.get_loc(self.FLAG_ISNOT_SKIP)] = sk
        self._record_change_idx(idx)

    def _read_quat(self, idx: int) -> Tuple[float, float, float, float]:
        assert self.q_cols is not None
        c1, c2, c3, c4 = self.q_cols
        r = self.df.iloc[idx]
        return float(r[c1]), float(r[c2]), float(r[c3]), float(r[c4])

    def _write_quat(self, idx: int, qx: float, qy: float, qz: float, qw: float) -> None:
        assert self.q_cols is not None
        c1, c2, c3, c4 = self.q_cols
        self.df.iat[idx, self.df.columns.get_loc(c1)] = qx
        self.df.iat[idx, self.df.columns.get_loc(c2)] = qy
        self.df.iat[idx, self.df.columns.get_loc(c3)] = qz
        self.df.iat[idx, self.df.columns.get_loc(c4)] = qw

    # ---- 保存 ----

    def _make_backup_once(self) -> None:
        if self._backup_made:
            return
        ts = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = self.csv_path.with_name(self.csv_path.stem + f"_backup_{ts}" + self.csv_path.suffix)
        self.df.to_csv(backup, index=False)
        self._backup_made = True

    def save(self) -> Tuple[Path, List[str]]:
        """<元名>_edited.csv へ保存し、変更ラベル一覧を返す。"""
        self._make_backup_once()
        out = self.csv_path.with_name(self.csv_path.stem + "_edited" + self.csv_path.suffix)
        # 浮動小数の桁は既定（差分安定化のため最大 7 桁程度）
        self.df.to_csv(out, index=False, float_format="%.7f")
        changes = self.get_change_log_strings()
        self._change_labels.clear()
        self._changed = False
        return out, changes

    def get_change_log_strings(self) -> List[str]:
        """保存時に表示するラベル一覧を返す。"""
        formatted: List[str] = []
        for label, action in self._change_labels.items():
            if action == "削除":
                formatted.append(f"[削除] {label}")
            else:
                formatted.append(label)
        return formatted


# =============================================================================
# Viewer: 描画・ヒットテスト（16:9キャンバス、等方スケール、強調表示ルール）
# =============================================================================

class Viewer:
    """matplotlib 図の管理（描画・強調表示・近傍選択・自動ズーム）。"""

    def __init__(
        self,
        fig: Figure,
        ax,
        map_adapter: MapAdapter,
        store: WaypointStore,
        map_image: np.ndarray,
    ) -> None:
        self.fig = fig
        self.ax = ax
        self.map = map_adapter
        self.store = store

        # 背景地図
        self._map_image = map_image
        self._map_height = int(map_image.shape[0])
        self._map_width = int(map_image.shape[1])
        self._map_extent = (0, self._map_width, self._map_height, 0)

        # 可視化ハンドル
        self._scatter = None
        self._quiver = None
        self._highlight_pt = None
        self._highlight_quiv = None
        self._highlight_text = None
        self._quiver_indices: Optional[np.ndarray] = None

        # 表示状態
        self._pix: Optional[np.ndarray] = None  # shape=(N,2)
        self._yaw_deg: Optional[np.ndarray] = None  # shape=(N,)
        self._valid_mask: Optional[np.ndarray] = None  # shape=(N,)
        self._near_px_threshold = 10.0  # 近傍選択しきい値 [px]
        self._arrow_px_len = 16.0  # 矢印表示の基準長 [px]
        self._highlight_index: Optional[int] = None

        # 色・強調（優先順位: signal > line > *_is_open > isnot_skipnum）
        self.color_default = "#202020"
        self.edge_signal = "#ff3333"  # 赤枠
        self.edge_line = "#ffcc00"    # 黄枠
        self.edge_isopen_right = "#22aa22"  # 緑
        self.edge_isopen_left = "#8833aa"   # 紫
        self.edge_isnot_skip = "#2288ff"    # 青点線

        # 矢印スタイル
        self.arrow_width = 0.003  # 軽い太さ（相対）

        # 初期表示範囲
        self.ax.set_xlim(0, self._map_width)
        self.ax.set_ylim(self._map_height, 0)

        # ズーム・パンに合わせて矢印長やラベル表示を調整するコールバック
        self._axis_callbacks = [
            self.ax.callbacks.connect("xlim_changed", self._on_axes_changed),
            self.ax.callbacks.connect("ylim_changed", self._on_axes_changed),
        ]

    # ---- 公開 API ----

    def set_near_threshold_px(self, px: float) -> None:
        self._near_px_threshold = max(1.0, float(px))

    def compute_all_pixels(self) -> None:
        """全点の (lat,lon) をピクセル座標に変換。yaw 度配列も作成。"""
        N = self.store.n
        pix = np.full((N, 2), np.nan, dtype=float)
        yaw_deg = np.zeros((N,), dtype=float)
        valid = np.zeros((N,), dtype=bool)
        for i, (lat, lon) in enumerate(self.store.iter_latlon()):
            pxpy = self.map.latlon_to_pixel(lat, lon)
            if pxpy[0] is None or pxpy[1] is None:
                continue
            px, py = pxpy
            pix[i, 0] = px
            pix[i, 1] = py
            yaw_deg[i] = self.store.read_pose_yaw_deg(i)
            valid[i] = True
        self._pix = pix
        self._yaw_deg = yaw_deg
        self._valid_mask = valid

    def draw_all(self) -> None:
        """全点を scatter + quiver で描画（初回／全体更新）。"""
        if self._pix is None or self._yaw_deg is None:
            self.compute_all_pixels()

        pix = self._pix
        yaw_deg = self._yaw_deg
        valid = self._valid_mask if self._valid_mask is not None else np.ones(len(pix), dtype=bool)

        # 色決定（優先順位に応じて枠色/マーカーサイズ/色相を決める）
        edgecolors = []
        sizes = []
        facecolors = []
        for i in range(self.store.n):
            flags = self.store.read_flags(i)
            # 既定
            edge = self.color_default
            size = 14.0
            face = "#000000"

            # *_is_open > 0 はサイズと色相に反映（右=緑、左=紫）
            ro = float(flags[WaypointStore.FLAG_RIGHT_OPEN])
            lo = float(flags[WaypointStore.FLAG_LEFT_OPEN])
            if ro > 0.0:
                face = self.edge_isopen_right
                size = 24.0
            if lo > 0.0:
                # 左 open があれば紫を優先（重畳ルールは任意だが、見分けやすさ重視）
                face = self.edge_isopen_left
                size = max(size, 24.0)

            # 最後に枠色の優先順位（signal > line > *_is_open > isnot_skip）
            if int(flags[WaypointStore.FLAG_SIGNAL_STOP]) == 1:
                edge = self.edge_signal
            elif int(flags[WaypointStore.FLAG_LINE_STOP]) == 1:
                edge = self.edge_line
            elif ro > 0.0 or lo > 0.0:
                edge = face  # open 色に合わせる
            elif int(flags[WaypointStore.FLAG_ISNOT_SKIP]) == 1:
                edge = self.edge_isnot_skip

            edgecolors.append(edge)
            facecolors.append(face)
            sizes.append(size)

        # 既存アーティストを消して描画
        self.ax.clear()
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.imshow(
            self._map_image,
            extent=self._map_extent,
            origin="upper",
            zorder=0,
        )
        self.ax.set_xlim(0, self._map_width)
        self.ax.set_ylim(self._map_height, 0)

        # 散布
        self._scatter = self.ax.scatter(
            pix[:, 0],
            pix[:, 1],
            s=np.asarray(sizes),
            c=facecolors,
            edgecolors=edgecolors,
            linewidths=1.0,
            alpha=0.9,
        )

        # 矢印（ピクセル長を軸座標に変換しながら描くため、後段で再スケール）
        # とりあえず単位長ベクトルを置き、後で draw_idle の直前にスケーリング
        dx = np.cos(np.deg2rad(yaw_deg))
        dy = np.sin(np.deg2rad(yaw_deg))
        if valid.any():
            arrow_len = self._estimate_arrow_length()
            valid_idx = np.where(valid)[0]
            self._quiver_indices = valid_idx
            self._quiver = self.ax.quiver(
                pix[valid_idx, 0],
                pix[valid_idx, 1],
                dx[valid_idx] * arrow_len,
                dy[valid_idx] * arrow_len,
                angles="xy",
                scale_units="xy",
                scale=1.0,
                width=self.arrow_width,
                color="#333333",
                zorder=3,
            )
        else:
            self._quiver_indices = None
            self._quiver = None

        self._redraw_highlight()

    def update_one(self, idx: int) -> None:
        """単一点の再計算・再描画。"""
        if self._pix is None or self._yaw_deg is None:
            self.compute_all_pixels()
        lat, lon = self.store.read_latlon(idx)
        pxpy = self.map.latlon_to_pixel(lat, lon)
        if pxpy[0] is None or pxpy[1] is None:
            self._pix[idx, 0] = np.nan
            self._pix[idx, 1] = np.nan
            if self._valid_mask is not None:
                self._valid_mask[idx] = False
        else:
            px, py = pxpy
            self._pix[idx, 0] = px
            self._pix[idx, 1] = py
            if self._valid_mask is not None:
                self._valid_mask[idx] = True
        self._yaw_deg[idx] = self.store.read_pose_yaw_deg(idx)

        # 散布図の座標を差分更新
        if self._scatter is not None:
            self._scatter.set_offsets(self._pix)

        # 矢印は一旦描画し直す（本数は多くない前提）
        if self._quiver is not None:
            self._quiver.remove()
            self._quiver = None

        valid = self._valid_mask if self._valid_mask is not None else np.ones(len(self._pix), dtype=bool)
        if valid.any():
            arrow_len = self._estimate_arrow_length()
            valid_idx = np.where(valid)[0]
            self._quiver_indices = valid_idx
            yaw_rad = np.deg2rad(self._yaw_deg[valid_idx])
            self._quiver = self.ax.quiver(
                self._pix[valid_idx, 0],
                self._pix[valid_idx, 1],
                np.cos(yaw_rad) * arrow_len,
                np.sin(yaw_rad) * arrow_len,
                angles="xy",
                scale_units="xy",
                scale=1.0,
                width=self.arrow_width,
                color="#333333",
                zorder=3,
            )
        else:
            self._quiver_indices = None

        self._redraw_highlight()

    def set_view_rect_meters_around(
        self,
        center_px: Tuple[float, float],
        short_half_m: float,
        meter_to_pixel: Callable[[float, float], Tuple[float, float]],
    ) -> None:
        """短辺±short_half_m の矩形で表示（ピクセルへの換算には外部関数を使う）。

        注: 本実装では mapper がピクセル↔メートル換算の直接APIを持つ保証がないため、
            ラムダで (E,N)→px のような関数を受け取り、短辺 m を px に換算する。
            簡易には、「短辺 m が現在スケールで何 px か」を近傍サンプルで近似して使う。
        """
        cx, cy = center_px

        # 近似的に、等方スケールを仮定して「1m → dpx」の係数を推定する。
        # （厳密にはピクセル・ワールドのアフィンで場所依存の可能性があるため、
        #   中心位置での微小変化として推定）
        (px1, py1) = meter_to_pixel(0.0, 0.0)
        (px2, py2) = meter_to_pixel(short_half_m, 0.0)
        dpx = math.hypot(px2 - px1, py2 - py1)  # 25m 相当の px
        if dpx <= 1.0:
            dpx = 50.0  # フォールバック

        half_px_short = dpx
        # 16:9 に合わせて長辺 px を計算（縦:横 = 9:16 と想定）
        # ここでは、短辺=高さ(9) と仮定して長辺=幅(16) を導く（実キャンバス比に合わせる）
        # アスペクトは Figure 側で 16:9 を保証しておく前提。
        half_px_long = half_px_short * (16.0 / 9.0)

        # どちらが短辺かは、キャンバスの実際の px 比で決まるため、Axes の bbox から判定
        bbox = self.ax.get_window_extent().transformed(self.fig.dpi_scale_trans.inverted())
        width_px = bbox.width * self.fig.dpi
        height_px = bbox.height * self.fig.dpi
        if width_px >= height_px:  # 横長: 高さが短辺
            half_x = half_px_long
            half_y = half_px_short
        else:  # 縦長: 幅が短辺
            half_x = half_px_short
            half_y = half_px_long

        # 表示範囲を設定（地図端ではクランプするのが望ましいが、ここでは簡易にそのまま）
        x_min = clamp(cx - half_x, 0, self._map_width)
        x_max = clamp(cx + half_x, 0, self._map_width)
        y_min = clamp(cy - half_y, 0, self._map_height)
        y_max = clamp(cy + half_y, 0, self._map_height)
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_max, y_min)
        self._refresh_quiver_vectors()
        self._redraw_highlight()

    def pick_nearest_index(self, x_px: float, y_px: float) -> Optional[int]:
        """クリック位置（px）に最も近い点の index を返す（しきい値外なら None）。"""
        if self._pix is None:
            self.compute_all_pixels()
        assert self._pix is not None
        if len(self._pix) == 0:
            return None
        d = np.hypot(self._pix[:, 0] - x_px, self._pix[:, 1] - y_px)
        if self._valid_mask is not None:
            d = np.where(self._valid_mask, d, np.inf)
        i = int(np.argmin(d))
        if d[i] <= self._near_px_threshold:
            return i
        return None

    def highlight_index(self, idx: Optional[int]) -> None:
        """指定 index をハイライトする。None で解除。"""
        if idx == self._highlight_index:
            return
        self._highlight_index = idx
        self._redraw_highlight()

    def show_full_extent(self) -> None:
        """地図全体を表示範囲に設定する。"""
        self.ax.set_xlim(0, self._map_width)
        self.ax.set_ylim(self._map_height, 0)
        self._refresh_quiver_vectors()
        self._redraw_highlight()

    # ---- 内部ヘルパ ----

    def _clear_highlight(self) -> None:
        if self._highlight_pt is not None:
            self._highlight_pt.remove()
            self._highlight_pt = None
        if self._highlight_quiv is not None:
            self._highlight_quiv.remove()
            self._highlight_quiv = None
        if self._highlight_text is not None:
            self._highlight_text.remove()
            self._highlight_text = None

    def _redraw_highlight(self) -> None:
        self._clear_highlight()
        if self._highlight_index is None or self._pix is None or self._yaw_deg is None:
            self.fig.canvas.draw_idle()
            return
        idx = self._highlight_index
        px = float(self._pix[idx, 0])
        py = float(self._pix[idx, 1])
        if not np.isfinite(px) or not np.isfinite(py):
            self.fig.canvas.draw_idle()
            return
        self._highlight_pt = self.ax.scatter(
            [px],
            [py],
            s=160.0,
            facecolors="none",
            edgecolors="#00ffff",
            linewidths=2.0,
            zorder=4,
        )
        yaw = float(self._yaw_deg[idx])
        arrow_len = self._estimate_arrow_length()
        self._highlight_quiv = self.ax.quiver(
            [px],
            [py],
            [math.cos(math.radians(yaw)) * arrow_len],
            [math.sin(math.radians(yaw)) * arrow_len],
            angles="xy",
            scale_units="xy",
            scale=1.0,
            width=self.arrow_width * 1.5,
            color="#00ffff",
            zorder=4,
        )
        if self._should_show_label_text():
            label = self.store.get_label_display(idx)
            self._highlight_text = self.ax.text(
                px + 6.0,
                py - 6.0,
                label,
                color="#ffffff",
                fontsize=9,
                fontweight="bold",
                bbox={"facecolor": "#000000", "alpha": 0.6, "edgecolor": "none", "pad": 2},
                zorder=5,
            )
        self.fig.canvas.draw_idle()

    def _estimate_arrow_length(self) -> float:
        """現在の描画領域に基づく矢印長（データ座標）を返す。"""
        bbox = self.ax.get_window_extent()
        width_px = max(bbox.width, 1.0)
        height_px = max(bbox.height, 1.0)
        x_span = abs(self.ax.get_xlim()[1] - self.ax.get_xlim()[0])
        y_span = abs(self.ax.get_ylim()[1] - self.ax.get_ylim()[0])
        data_per_px_x = x_span / width_px
        data_per_px_y = y_span / height_px
        data_per_px = max((data_per_px_x + data_per_px_y) * 0.5, 1e-6)
        desired_px = 40.0
        return max(data_per_px * desired_px, 5.0)

    def _refresh_quiver_vectors(self) -> None:
        """ズーム倍率に合わせて矢印ベクトルを再計算する。"""
        if self._quiver is None or self._yaw_deg is None or self._quiver_indices is None:
            return
        arrow_len = self._estimate_arrow_length()
        idx = self._quiver_indices
        yaw_rad = np.deg2rad(self._yaw_deg[idx])
        u = np.cos(yaw_rad) * arrow_len
        v = np.sin(yaw_rad) * arrow_len
        self._quiver.set_UVC(u, v)
        self.fig.canvas.draw_idle()

    def _should_show_label_text(self) -> bool:
        """高倍率表示か判定する。"""
        x_span = abs(self.ax.get_xlim()[1] - self.ax.get_xlim()[0])
        y_span = abs(self.ax.get_ylim()[1] - self.ax.get_ylim()[0])
        short_span = min(x_span, y_span)
        return short_span <= max(self._map_width, self._map_height) / 3.0

    def _on_axes_changed(self, _axis) -> None:
        """パン・ズーム時に矢印とラベルを更新する。"""
        self._refresh_quiver_vectors()
        if self._highlight_index is not None:
            self._redraw_highlight()


# =============================================================================
# Editor（UI 制御・イベント・アプリ本体）
# =============================================================================

class App:
    """Tkinter アプリ本体。"""

    def __init__(self, csv_path: Path, map_path: Path, pgw_path: Path) -> None:
        # ---- データ層 ----
        self.store = WaypointStore(csv_path)
        self.map = MapAdapter(mapper_mod)
        # mapper 側が load_map を提供する場合に初期化
        try:
            self.map.load_map(map_path, pgw_path)
        except Exception:
            # 無ければスルー（mapper 側の設計に依存）
            pass

        self.map_image = mpimg.imread(str(map_path))

        # (E,N) と (x,y) の相似変換を推定
        self.aligner = XYAligner()
        self._similarity_rms: Optional[float] = None

        # ---- UI 構築 ----
        self.root = tk.Tk()
        self.root.title("Waypoint 地図連携エディタ")
        self.root.geometry("1280x720")  # 16:9 推奨（ウィンドウ比は調整可）
        self.status_var = tk.StringVar(value="RMS: ---")
        self._build_widgets()

        # 状態
        self.mode: str = "nav"  # "nav" / "focus"
        self.focus_idx: Optional[int] = None
        self.last_deleted: Optional[Tuple[int, pd.Series]] = None  # 簡易 Undo 用（1段）

        # 初期の相似変換推定と描画
        self._fit_similarity_model(max_points=20)
        # 初期描画
        self.viewer.compute_all_pixels()
        self.viewer.draw_all()
        self._update_similarity_status()

    # ---- 相似変換の推定 ----

    def _fit_similarity_model(self, max_points: int = 20) -> None:
        """CSV 先頭から K 点で (E,N)↔(x,y) の相似変換を推定。"""
        K = min(max_points, self.store.n)
        if K == 0:
            self._similarity_rms = None
            self._update_similarity_status()
            return

        EN = np.zeros((K, 2), dtype=float)
        XY = np.zeros((K, 2), dtype=float)
        try:
            for i in range(K):
                lat, lon = self.store.read_latlon(i)
                e, n = self.map.latlon_to_local_m(lat, lon)
                EN[i, 0] = e
                EN[i, 1] = n
                x, y = self.store.read_xy(i)
                XY[i, 0] = x
                XY[i, 1] = y
        except Exception as e:
            messagebox.showwarning("相似変換推定エラー", f"(E,N) 変換に失敗しました: {e}")
            self._similarity_rms = None
            self._update_similarity_status()
            return

        sim, rms, mask = self.aligner.fit(EN, XY)
        if sim is None:
            print(
                "警告: x,y と (E,N) の相似変換の推定に失敗しました。x,y の更新は恒等変換で行います。",
                file=sys.stderr,
            )
            self._similarity_rms = None
        else:
            print(f"相似変換推定: s={sim.s:.6f}, R=\n{sim.R}\n t={sim.t}, RMS={rms:.3f} m")
            self._similarity_rms = rms
        self._update_similarity_status()

    # ---- UI 構築 ----

    def _build_widgets(self) -> None:
        """上部ツールバー・中央キャンバス（16:9）・下部編集パネルを作る。"""
        # 上部ツールバー
        toolbar_frame = ttk.Frame(self.root)
        toolbar_frame.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)

        btn_reset = ttk.Button(toolbar_frame, text="全体表示", command=self._on_reset_view)
        btn_toggle = ttk.Button(toolbar_frame, text="モード切替 (ナビ↔フォーカス)", command=self._on_toggle_mode)
        btn_save = ttk.Button(toolbar_frame, text="保存", command=self._on_save)
        btn_settings = ttk.Button(toolbar_frame, text="設定", command=self._on_settings)

        btn_reset.pack(side=tk.LEFT, padx=4)
        btn_toggle.pack(side=tk.LEFT, padx=4)
        btn_save.pack(side=tk.LEFT, padx=4)
        btn_settings.pack(side=tk.LEFT, padx=4)

        # 中央：16:9 の図領域
        fig_w, fig_h = 16, 9
        self.fig = Figure(figsize=(fig_w, fig_h), dpi=72)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # matplotlib 標準ツールバー（ナビゲーション）
        self.nav_toolbar = NavigationToolbar2Tk(self.canvas, self.root, pack_toolbar=False)
        self.nav_toolbar.update()
        self.nav_toolbar.pack(side=tk.TOP, fill=tk.X)

        # Viewer を構築
        self.viewer = Viewer(self.fig, self.ax, self.map, self.store, np.asarray(self.map_image))

        # キャンバスのクリックイベント（ナビ: 選択／フォーカス: 無し）
        self.canvas.mpl_connect("button_press_event", self._on_canvas_click)
        self.canvas.mpl_connect("motion_notify_event", self._on_canvas_motion)

        # 下部：編集パネル
        self._build_editor_panel()

    def _build_editor_panel(self) -> None:
        """フォーカスモードで使う編集パネル。"""
        panel = ttk.Frame(self.root, padding=8)
        panel.pack(side=tk.BOTTOM, fill=tk.X)
        self.editor_panel = panel

        # 1行目：Label / LatLon / Pose 表示（RO）
        row1 = ttk.Frame(panel)
        row1.pack(side=tk.TOP, fill=tk.X, pady=2)
        ttk.Label(row1, text="Label:").pack(side=tk.LEFT)
        self.var_label = tk.StringVar(value="-")
        ttk.Entry(row1, textvariable=self.var_label, width=12, state="readonly").pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(row1, text="Lat:").pack(side=tk.LEFT)
        self.var_lat = tk.StringVar(value="-")
        ttk.Entry(row1, textvariable=self.var_lat, width=16, state="readonly").pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(row1, text="Lon:").pack(side=tk.LEFT)
        self.var_lon = tk.StringVar(value="-")
        ttk.Entry(row1, textvariable=self.var_lon, width=16, state="readonly").pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(row1, text="Pose x:").pack(side=tk.LEFT)
        self.var_x = tk.StringVar(value="-")
        ttk.Entry(row1, textvariable=self.var_x, width=10, state="readonly").pack(side=tk.LEFT, padx=(0, 4))

        ttk.Label(row1, text="y:").pack(side=tk.LEFT)
        self.var_y = tk.StringVar(value="-")
        ttk.Entry(row1, textvariable=self.var_y, width=10, state="readonly").pack(side=tk.LEFT, padx=(0, 4))

        ttk.Label(row1, text="yaw[deg]:").pack(side=tk.LEFT)
        self.var_yaw = tk.StringVar(value="-")
        ttk.Entry(row1, textvariable=self.var_yaw, width=8, state="readonly").pack(side=tk.LEFT, padx=(0, 4))

        # 2行目：Flags 編集
        row2 = ttk.LabelFrame(panel, text="Flags")
        row2.pack(side=tk.TOP, fill=tk.X, pady=4)
        self.var_right_open = tk.StringVar(value="0")
        self.var_left_open = tk.StringVar(value="0")
        self.var_line_stop = tk.StringVar(value="0")
        self.var_signal_stop = tk.StringVar(value="0")
        self.var_isnot_skip = tk.StringVar(value="0")

        for (label, var, width, hint) in [
            ("right_is_open", self.var_right_open, 8, "0以上（小数可）"),
            ("left_is_open", self.var_left_open, 8, "0以上（小数可）"),
            ("line_is_stop", self.var_line_stop, 4, "0/1"),
            ("signal_is_stop", self.var_signal_stop, 4, "0/1"),
            ("isnot_skipnum", self.var_isnot_skip, 4, "0/1"),
        ]:
            frm = ttk.Frame(row2)
            frm.pack(side=tk.LEFT, padx=8)
            ttk.Label(frm, text=label).pack(side=tk.TOP, anchor=tk.W)
            ttk.Entry(frm, textvariable=var, width=width).pack(side=tk.TOP)
            ttk.Label(frm, text=hint, foreground="#666666").pack(side=tk.TOP, anchor=tk.W)

        # 3行目：位置・向きボタン
        row3 = ttk.Frame(panel)
        row3.pack(side=tk.TOP, fill=tk.X, pady=4)

        # 位置（8方向）
        grid = ttk.Frame(row3)
        grid.pack(side=tk.LEFT, padx=8)
        ttk.Label(grid, text="位置 0.25 m シフト").grid(row=0, column=0, columnspan=3)

        def mkbtn(txt: str, r: int, c: int, cb: Callable[[], None]) -> None:
            b = ttk.Button(grid, text=txt, width=4, command=cb)
            b.grid(row=r, column=c, padx=1, pady=1)

        mkbtn("↖", 1, 0, lambda: self._on_shift(+1, -1))
        mkbtn("↑", 1, 1, lambda: self._on_shift(+1, 0))
        mkbtn("↗", 1, 2, lambda: self._on_shift(+1, +1))
        mkbtn("←", 2, 0, lambda: self._on_shift(0, -1))
        mkbtn("→", 2, 2, lambda: self._on_shift(0, +1))
        mkbtn("↙", 3, 0, lambda: self._on_shift(-1, -1))
        mkbtn("↓", 3, 1, lambda: self._on_shift(-1, 0))
        mkbtn("↘", 3, 2, lambda: self._on_shift(-1, +1))

        # 向き
        rotf = ttk.Frame(row3)
        rotf.pack(side=tk.LEFT, padx=16)
        ttk.Label(rotf, text="向き ±5° 回転").pack()
        ttk.Button(rotf, text="↺ -5°", width=8, command=lambda: self._on_rotate(-5.0)).pack(side=tk.LEFT, padx=4)
        ttk.Button(rotf, text="↻ +5°", width=8, command=lambda: self._on_rotate(+5.0)).pack(side=tk.LEFT, padx=4)

        # 遷移・削除・保存
        ctl = ttk.Frame(row3)
        ctl.pack(side=tk.RIGHT, padx=8)
        ttk.Button(ctl, text="前へ", width=8, command=self._on_prev).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctl, text="次へ", width=8, command=self._on_next).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctl, text="削除", width=8, command=self._on_delete).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctl, text="保存", width=8, command=self._on_save).pack(side=tk.LEFT, padx=4)

        # 変換 RMS 表示
        ttk.Label(panel, textvariable=self.status_var, anchor=tk.W).pack(side=tk.TOP, anchor=tk.W, pady=(4, 0))

    # ---- ユーティリティ ----

    def _meter_to_pixel_local(self, center_lat: float, center_lon: float) -> Callable[[float, float], Tuple[float, float]]:
        """(ΔE,ΔN) をピクセル Δ に近似変換する関数を返す。

        ここでは mapper が局所的に線形と見なせる前提で、
        (lat,lon)→pixel の微小変化を中心点で近似して px 換算を得る。
        """
        cx_px, cy_px = self.map.latlon_to_pixel(center_lat, center_lon)
        if cx_px is None or cy_px is None:
            raise RuntimeError("対象 waypoint が地図外にあります。")

        def f(de: float, dn: float) -> Tuple[float, float]:
            # (E,N) を lat,lon に戻し、その pixel を得る（mapper API で往復）
            e0, n0 = self.map.latlon_to_local_m(center_lat, center_lon)
            lat1, lon1 = self.map.local_m_to_latlon(e0 + de, n0 + dn)
            px1, py1 = self.map.latlon_to_pixel(lat1, lon1)
            if px1 is None or py1 is None:
                return cx_px, cy_px
            return (cx_px + (px1 - cx_px), cy_px + (py1 - cy_px))

        return f

    def _update_similarity_status(self) -> None:
        """相似変換 RMS の表示を更新する。"""
        if self._similarity_rms is None:
            self.status_var.set("RMS: N/A")
        else:
            self.status_var.set(f"RMS: {self._similarity_rms:.3f} m")

    def _update_panel_from_index(self, idx: int) -> None:
        """フォーカス対象の情報をパネルへ反映。"""
        self.focus_idx = idx
        label = self.store.get_label_display(idx)
        lat, lon = self.store.read_latlon(idx)
        x, y = self.store.read_xy(idx)
        yaw_deg = self.store.read_pose_yaw_deg(idx)
        flags = self.store.read_flags(idx)

        self.var_label.set(str(label))
        self.var_lat.set(f"{lat:.7f}")
        self.var_lon.set(f"{lon:.7f}")
        self.var_x.set(f"{x:.3f}")
        self.var_y.set(f"{y:.3f}")
        self.var_yaw.set(f"{yaw_deg:.1f}")

        self.var_right_open.set(str(flags[WaypointStore.FLAG_RIGHT_OPEN]))
        self.var_left_open.set(str(flags[WaypointStore.FLAG_LEFT_OPEN]))
        self.var_line_stop.set(str(flags[WaypointStore.FLAG_LINE_STOP]))
        self.var_signal_stop.set(str(flags[WaypointStore.FLAG_SIGNAL_STOP]))
        self.var_isnot_skip.set(str(flags[WaypointStore.FLAG_ISNOT_SKIP]))

    def _apply_flags_from_panel(self) -> bool:
        """フラグ Entry の値を検証して DataFrame に書き戻す。"""
        idx = self.focus_idx
        if idx is None:
            return False
        try:
            flags = {
                WaypointStore.FLAG_RIGHT_OPEN: float(self.var_right_open.get()),
                WaypointStore.FLAG_LEFT_OPEN: float(self.var_left_open.get()),
                WaypointStore.FLAG_LINE_STOP: int(self.var_line_stop.get()),
                WaypointStore.FLAG_SIGNAL_STOP: int(self.var_signal_stop.get()),
                WaypointStore.FLAG_ISNOT_SKIP: int(self.var_isnot_skip.get()),
            }
            self.store.write_flags(idx, flags)
            self.viewer.draw_all()
            return True
        except Exception as e:
            messagebox.showerror("入力エラー", f"フラグの値が不正です: {e}")
            return False

    # ---- イベントハンドラ ----

    def _on_canvas_click(self, event) -> None:
        """キャンバス左クリック（ナビモードでフォーカス対象を選ぶ）。"""
        if event.button != 1:
            return
        if self.mode != "nav":
            return
        if event.xdata is None or event.ydata is None:
            return
        idx = self.viewer.pick_nearest_index(event.xdata, event.ydata)
        if idx is None:
            return
        # フォーカスモードに移行
        if self.store.is_changed:
            if not messagebox.askyesno("未保存の変更", "未保存の変更があります。破棄して続行しますか？"):
                return
        self.mode = "focus"
        self._enter_focus(idx)

    def _on_canvas_motion(self, event) -> None:
        """ナビモード時のホバーで近傍 waypoint をハイライトする。"""
        if self.mode != "nav":
            return
        if event.xdata is None or event.ydata is None:
            self.viewer.highlight_index(None)
            return
        idx = self.viewer.pick_nearest_index(event.xdata, event.ydata)
        self.viewer.highlight_index(idx)

    def _enter_focus(self, idx: int) -> None:
        """フォーカスモードへ。自動ズーム＋右ペイン更新。"""
        self.viewer.highlight_index(idx)
        self._update_panel_from_index(idx)
        lat, lon = self.store.read_latlon(idx)
        pxpy = self.map.latlon_to_pixel(lat, lon)
        if pxpy[0] is None or pxpy[1] is None:
            messagebox.showwarning("注意", "選択した waypoint は地図外のためフォーカス表示できません。")
            self.viewer.show_full_extent()
            return
        try:
            meter_to_px = self._meter_to_pixel_local(lat, lon)
        except Exception as e:
            messagebox.showwarning("注意", f"フォーカス用の局所変換が取得できませんでした: {e}")
            self.viewer.show_full_extent()
            return
        self.viewer.set_view_rect_meters_around((pxpy[0], pxpy[1]), short_half_m=25.0, meter_to_pixel=meter_to_px)

    def _on_reset_view(self) -> None:
        """全体表示（ナビに適する）。"""
        self.viewer.show_full_extent()

    def _on_toggle_mode(self) -> None:
        """モード切替（ナビ↔フォーカス）。"""
        if self.mode == "focus" and self.store.is_changed:
            if not messagebox.askyesno("未保存の変更", "未保存の変更があります。破棄して続行しますか？"):
                return
        self.mode = "nav" if self.mode == "focus" else "focus"
        if self.mode == "nav":
            self.viewer.highlight_index(None)
            self._on_reset_view()
        else:
            if self.store.n == 0:
                messagebox.showinfo("情報", "編集可能な waypoint が存在しません。")
                self.mode = "nav"
                self.viewer.highlight_index(None)
                self._on_reset_view()
                return
            # フォーカスに入る場合は、最後の idx か 0 を選ぶ
            idx = self.focus_idx if self.focus_idx is not None else 0
            self._enter_focus(idx)

    def _on_settings(self) -> None:
        """設定ダイアログ（近傍しきい値など）。"""
        win = tk.Toplevel(self.root)
        win.title("設定")
        ttk.Label(win, text="近傍選択しきい値 [px]").pack(side=tk.TOP, anchor=tk.W, padx=8, pady=4)
        var_px = tk.StringVar(value=str(int(self.viewer._near_px_threshold)))
        ent = ttk.Entry(win, textvariable=var_px, width=8)
        ent.pack(side=tk.TOP, padx=8)
        ttk.Label(win, text="※ 小さ過ぎると選びづらくなります（既定 10）").pack(side=tk.TOP, anchor=tk.W, padx=8)

        def ok() -> None:
            try:
                v = float(var_px.get())
                self.viewer.set_near_threshold_px(v)
                win.destroy()
            except Exception:
                messagebox.showerror("エラー", "数値を入力してください。")

        ttk.Button(win, text="OK", command=ok).pack(side=tk.TOP, pady=8)

        def refit() -> None:
            self._fit_similarity_model(max_points=20)
            self.viewer.compute_all_pixels()
            self.viewer.draw_all()
            messagebox.showinfo("再推定完了", "(E,N) と (x,y) の相似変換を再推定しました。")

        ttk.Button(win, text="相似変換を再推定", command=refit).pack(side=tk.TOP, pady=4)

    def _on_save(self) -> None:
        """CSV 保存。"""
        try:
            if self.mode == "focus":
                # フォーカス中はフラグ編集の書戻しを優先（失敗なら保存中止）
                if not self._apply_flags_from_panel():
                    return
            out, labels = self.store.save()
            if labels:
                summary = f"\n変更件数: {len(labels)}\n対象: {', '.join(labels)}"
            else:
                summary = "\n変更件数: 0\n対象: なし"
            message = f"保存しました: {out.name}{summary}"
            messagebox.showinfo("保存完了", message)
            print(f"[保存] {out} 変更件数={len(labels)} 対象={labels}")
        except Exception as e:
            messagebox.showerror("保存エラー", str(e))

    def _on_prev(self) -> None:
        idx = self.focus_idx
        if idx is None:
            idx = 0
        else:
            idx = (idx - 1) % self.store.n
        self._enter_focus(idx)

    def _on_next(self) -> None:
        idx = self.focus_idx
        if idx is None:
            idx = 0
        else:
            idx = (idx + 1) % self.store.n
        self._enter_focus(idx)

    def _on_delete(self) -> None:
        idx = self.focus_idx
        if idx is None:
            return
        label = self.store.get_label_display(idx)
        if not messagebox.askyesno("削除確認", f"この waypoint を削除しますか？\nlabel={label}"):
            return
        # 簡易 Undo 用に保持（1段のみ）
        self.last_deleted = (idx, self.store.get_row(idx).copy())
        self.store.delete_row(idx)
        if self.store.n == 0:
            self.focus_idx = None
            self.viewer.draw_all()
            self.viewer.highlight_index(None)
            return
        # 次へ遷移
        nx = min(idx, self.store.n - 1)
        self.viewer.compute_all_pixels()
        self.viewer.draw_all()
        self._enter_focus(nx)

    def _on_shift(self, dn_sign: int, de_sign: int) -> None:
        """位置 0.25 m シフト（短辺基準ではなく、(E,N) の直交系で固定）。"""
        if self.mode != "focus" or self.focus_idx is None:
            return
        idx = self.focus_idx
        # 現在の lat,lon をベースに (E,N) を得る
        lat0, lon0 = self.store.read_latlon(idx)
        e0, n0 = self.map.latlon_to_local_m(lat0, lon0)
        de = 0.25 * float(de_sign)
        dn = 0.25 * float(dn_sign)

        e1 = e0 + de
        n1 = n0 + dn
        lat1, lon1 = self.map.local_m_to_latlon(e1, n1)

        # lat/lon を更新
        self.store.write_latlon(idx, lat1, lon1)

        # x,y は相似変換で (E,N) から更新
        x1, y1 = self.aligner.en_to_xy(e1, n1)
        self.store.write_xy(idx, x1, y1)

        # 表示更新
        self._update_panel_from_index(idx)
        self.viewer.update_one(idx)

        # 自動ズームは維持（フォーカス内での微修正に留める）

    def _on_rotate(self, ddeg: float) -> None:
        """向き ±5° 回転。"""
        if self.mode != "focus" or self.focus_idx is None:
            return
        idx = self.focus_idx
        self.store.write_pose_yaw_delta_deg(idx, ddeg)
        self._update_panel_from_index(idx)
        self.viewer.update_one(idx)

    # ---- メインループ ----

    def run(self) -> None:
        self.root.mainloop()


# =============================================================================
# メイン（引数処理）
# =============================================================================

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """引数を解析する。"""
    p = argparse.ArgumentParser(description="Waypoint 地図連携エディタ")
    p.add_argument("--csv", required=True, type=Path, help="編集対象 CSV（例: waypoint.csv）")
    p.add_argument("--map", required=True, type=Path, help="地図画像 PNG（16:9 推奨）")
    p.add_argument("--pgw", required=True, type=Path, help="ワールドファイル PGW")
    return p.parse_args(argv)


def main() -> None:
    """エントリポイント。"""
    args = parse_args()
    app = App(csv_path=args.csv, map_path=args.map, pgw_path=args.pgw)
    app.run()


if __name__ == "__main__":
    main()

