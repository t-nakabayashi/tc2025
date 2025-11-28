#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地理座標（緯度・経度, WGS84）をPGWに基づく画像座標へマッピングするスクリプト。
PGWの座標単位を自動的に判定し、必要に応じて投影変換（Web Mercator）を行う。

使用例:
    python latlon_to_pixel_mapper.py map.png map.pgw 36.081263 140.079143 36.081500 140.080000

依存:
    Pillow (pip install pillow)
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple, Union

from PIL import Image, ImageDraw


# ----------------------------------------------------------------------
# 内部ユーティリティ
# ----------------------------------------------------------------------

def _read_pgw(worldfile_path: str) -> Tuple[float, float, float, float, float, float]:
    """PGWファイルを読み込み、6つのパラメータ(A,D,B,E,C,F)を返す。"""
    try:
        with open(worldfile_path, "r", encoding="utf-8") as f:
            vals = [float(line.strip()) for line in f.readlines()]
        if len(vals) != 6:
            raise ValueError("PGWファイルの行数が不正です（6行必要）。")
        return tuple(vals)
    except Exception as e:
        raise RuntimeError(f"PGWファイル読み込みエラー: {e}") from e


def _is_projected_pgw(C: float, F: float, A: float, E: float) -> bool:
    """PGWの値から座標単位を推定する。
    True: 投影座標系（メートル単位） / False: 地理座標系（度単位）
    """
    # 絶対値が1e6オーダーならWebMercator/EPSG:3857相当とみなす
    return (abs(C) > 1e6 or abs(F) > 1e6) and (abs(A) < 1.0 and abs(E) < 1.0)


# WGS84⇄WebMercator 相互変換
RADIUS_EARTH_M = 6378137.0


def latlon_to_webmercator(lat: float, lon: float) -> Tuple[float, float]:
    """WGS84(度)→Web Mercator(メートル)"""
    x = math.radians(lon) * RADIUS_EARTH_M
    y = RADIUS_EARTH_M * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))
    return x, y


def webmercator_to_latlon(x: float, y: float) -> Tuple[float, float]:
    """Web Mercator(メートル)→WGS84(度)"""
    lon = math.degrees(x / RADIUS_EARTH_M)
    lat = math.degrees(2 * math.atan(math.exp(y / RADIUS_EARTH_M)) - math.pi / 2)
    return lat, lon


# ----------------------------------------------------------------------
# 主要関数
# ----------------------------------------------------------------------

@dataclass
class _MapInfo:
    """PGWと画像情報を保持し、座標変換を提供する内部構造。"""

    image_path: str
    worldfile_path: str
    width: int
    height: int
    A: float
    D: float
    B: float
    E: float
    C: float
    F: float
    projected: bool
    det: float
    invA: float
    invB: float
    invD: float
    invE: float

    def latlon_to_map(self, lat: float, lon: float) -> Tuple[float, float]:
        """緯度経度を PGW 座標系（投影 or 地理）へ変換する。"""
        if self.projected:
            return latlon_to_webmercator(lat, lon)
        return lon, lat

    def map_to_latlon(self, mapx: float, mapy: float) -> Tuple[float, float]:
        """PGW 座標系から緯度経度へ変換する。"""
        if self.projected:
            lat, lon = webmercator_to_latlon(mapx, mapy)
            return lat, lon
        # 地理座標系（度）の場合は (lon, lat) 順を戻す
        return mapy, mapx

    def latlon_to_pixel_float(self, lat: float, lon: float) -> Tuple[float, float]:
        """緯度経度→ピクセル座標（float）。"""
        mapx, mapy = self.latlon_to_map(lat, lon)
        x_img = self.invA * (mapx - self.C) + self.invB * (mapy - self.F)
        y_img = self.invD * (mapx - self.C) + self.invE * (mapy - self.F)
        return x_img, y_img

    def pixel_to_latlon(self, px: float, py: float) -> Tuple[float, float]:
        """ピクセル座標→緯度経度。"""
        mapx = self.A * px + self.B * py + self.C
        mapy = self.D * px + self.E * py + self.F
        return self.map_to_latlon(mapx, mapy)


_LOADED_MAP: Optional[_MapInfo] = None


def _load_map_info(image_path: str, worldfile_path: str) -> _MapInfo:
    """画像・PGWを読み込み `_MapInfo` を生成する。"""
    A, D, B, E, C, F = _read_pgw(worldfile_path)
    projected = _is_projected_pgw(C, F, A, E)
    with Image.open(image_path) as img:
        width, height = img.size

    det = A * E - B * D
    if abs(det) < 1e-12:
        raise ValueError("PGWアフィン行列が特異です。")
    invA = E / det
    invB = -B / det
    invD = -D / det
    invE = A / det

    return _MapInfo(
        image_path=str(image_path),
        worldfile_path=str(worldfile_path),
        width=width,
        height=height,
        A=A,
        D=D,
        B=B,
        E=E,
        C=C,
        F=F,
        projected=projected,
        det=det,
        invA=invA,
        invB=invB,
        invD=invD,
        invE=invE,
    )


def load_map(image_path: Union[str, Path], worldfile_path: Union[str, Path]) -> None:
    """画像と PGW を読み込み、以降の単一座標変換用にキャッシュする。"""
    global _LOADED_MAP
    _LOADED_MAP = _load_map_info(str(image_path), str(worldfile_path))


def _require_loaded_map(
    image_path: Optional[Union[str, Path]] = None,
    worldfile_path: Optional[Union[str, Path]] = None,
) -> _MapInfo:
    """キャッシュされた `_MapInfo` を取得する。必要に応じてロードする。"""
    global _LOADED_MAP
    if _LOADED_MAP is not None:
        return _LOADED_MAP
    if image_path is None or worldfile_path is None:
        raise RuntimeError("load_map() を先に呼び出してください。")
    load_map(image_path, worldfile_path)
    assert _LOADED_MAP is not None
    return _LOADED_MAP


def get_map_size() -> Tuple[int, int]:
    """ロード済み地図の (width, height) を返す。"""
    info = _require_loaded_map()
    return info.width, info.height


def _latlon_iterable(latlon_list: Sequence[Tuple[float, float]]) -> Iterable[Tuple[float, float]]:
    for lat, lon in latlon_list:
        yield float(lat), float(lon)


def latlon_to_pixel(
    *args: Union[str, Path, Sequence[Tuple[float, float]], float],
    verbose: bool = False,
) -> Union[List[Tuple[int | None, int | None]], Tuple[float, float]]:
    """緯度経度→ピクセル座標変換。

    2種類の呼出し方法をサポートする：

    1. 既存互換: ``latlon_to_pixel(image_path, worldfile_path, latlon_list, verbose=False)``
    2. キャッシュ使用: ``load_map()`` 呼出後に ``latlon_to_pixel(lat, lon)``
    """

    # --- 単一座標（キャッシュ利用） ---
    if len(args) == 2 and all(isinstance(v, (int, float)) for v in args):
        lat = float(args[0])
        lon = float(args[1])
        info = _require_loaded_map()
        px, py = info.latlon_to_pixel_float(lat, lon)
        return px, py

    # --- 既存API互換（バッチ変換） ---
    if len(args) != 3:
        raise TypeError(
            "latlon_to_pixel() の引数が不正です。旧API: (image, worldfile, list) / 新API: (lat, lon)"
        )

    image_path = str(args[0])
    worldfile_path = str(args[1])
    latlon_list = args[2]
    if not isinstance(latlon_list, Sequence):
        raise TypeError("latlon_list には (lat, lon) シーケンスを指定してください。")

    info = _load_map_info(image_path, worldfile_path)
    # バッチ呼出時もキャッシュを更新しておく（後続の単体変換に備える）
    global _LOADED_MAP
    _LOADED_MAP = info

    if verbose:
        print("=== PGW parameters ===")
        print(f"A={info.A}, D={info.D}, B={info.B}, E={info.E}, C={info.C}, F={info.F}")
        print(f"Image size: width={info.width}, height={info.height}")
        mode = "Projected (m)" if info.projected else "Geographic (deg)"
        print(f"PGW interpreted as {mode}\n")

    results: List[Tuple[int | None, int | None]] = []
    for lat, lon in _latlon_iterable(latlon_list):
        px_f, py_f = info.latlon_to_pixel_float(lat, lon)
        x_pix = round(px_f)
        y_pix = round(py_f)
        inside = 0 <= x_pix < info.width and 0 <= y_pix < info.height

        if verbose:
            mapx, mapy = info.latlon_to_map(lat, lon)
            print(
                f"lat={lat:.6f}, lon={lon:.6f} -> "
                f"mapX={mapx:.3f}, mapY={mapy:.3f}, "
                f"x_img={px_f:.3f}, y_img={py_f:.3f}, "
                f"x_pix={x_pix}, y_pix={y_pix}, inside={inside}"
            )

        if inside:
            results.append((x_pix, y_pix))
        else:
            results.append((None, None))

    if verbose:
        print("")
    return results


def pixel_to_latlon(
    px: float,
    py: float,
    *,
    image_path: Optional[Union[str, Path]] = None,
    worldfile_path: Optional[Union[str, Path]] = None,
) -> Tuple[float, float]:
    """ピクセル→緯度経度。load_map() 済み、もしくはパス指定で利用する。"""
    info = _require_loaded_map(image_path, worldfile_path)
    return info.pixel_to_latlon(float(px), float(py))


def latlon_to_local_meters(lat: float, lon: float) -> Tuple[float, float]:
    """緯度経度→ローカル平面直交座標 (E,N) [m]。"""
    info = _require_loaded_map()
    lat_f = float(lat)
    lon_f = float(lon)
    if info.projected:
        return info.latlon_to_map(lat_f, lon_f)
    # PGWが度の場合は Web Mercator を採用してローカル直交座標とする
    return latlon_to_webmercator(lat_f, lon_f)


def local_meters_to_latlon(e: float, n: float) -> Tuple[float, float]:
    """ローカル平面直交座標 (E,N) [m] → 緯度経度。"""
    info = _require_loaded_map()
    e_f = float(e)
    n_f = float(n)
    if info.projected:
        return info.map_to_latlon(e_f, n_f)
    lat, lon = webmercator_to_latlon(e_f, n_f)
    return lat, lon

def _draw_points(image_path: str, points: List[Tuple[int | None, int | None]], output_path: str) -> None:
    """変換結果を画像上に描画して保存する。"""
    with Image.open(image_path).convert("RGB") as img:
        draw = ImageDraw.Draw(img)
        for pt in points:
            if pt == (None, None):
                continue
            x, y = pt
            r = 5
            draw.ellipse((x - r, y - r, x + r, y + r), outline="red", fill="red")
        img.save(output_path)
        print(f"出力画像を保存しました: {output_path}")


def main() -> None:
    """コマンドライン実行エントリポイント。"""
    if len(sys.argv) < 5 or (len(sys.argv) - 3) % 2 != 0:
        print(
            "使用方法: python latlon_to_pixel_mapper.py <image.png> <worldfile.pgw> <lat1> <lon1> [<lat2> <lon2> ...]"
        )
        sys.exit(1)

    image_path = sys.argv[1]
    worldfile_path = sys.argv[2]

    # (lat, lon) のペアを収集
    args = sys.argv[3:]
    latlon_pairs: List[Tuple[float, float]] = []
    for i in range(0, len(args), 2):
        try:
            lat = float(args[i])
            lon = float(args[i + 1])
            latlon_pairs.append((lat, lon))
        except ValueError:
            print(f"緯度経度の形式が不正です: {args[i]}, {args[i+1]}")
            sys.exit(1)

    # 変換実行
    try:
        pixels = latlon_to_pixel(image_path, worldfile_path, latlon_pairs, verbose=True)
    except Exception as e:
        print(f"エラー: {e}")
        sys.exit(1)

    # 結果出力
    for (lat, lon), (x, y) in zip(latlon_pairs, pixels):
        print(f"({lat:.6f}, {lon:.6f}) -> ({x}, {y})")

    # 出力画像の保存
    out_path = Path(image_path).with_name(Path(image_path).stem + "_mapped.png")
    _draw_points(image_path, pixels, str(out_path))


if __name__ == "__main__":
    main()

