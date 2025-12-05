#!/usr/bin/env python3
# coding=utf-8
"""
ウェイポイントCSVをグラフ上に描画するスクリプト

使用方法:
    python3 plot_waypoints.py --input <入力CSV> [--output <出力画像>] [--show]

機能:
    - ウェイポイントを点として描画
    - 進行方向を矢印で表示
    - line_is_stop/signal_is_stop フラグが立っている点を色分け表示
    - ラベル番号を表示（オプション）
"""

import argparse
import csv
import math
import os
from pathlib import Path
from typing import List, Dict, Optional

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


def quaternion_to_yaw(q3: float, q4: float) -> float:
    """
    クォータニオン(q3=z, q4=w)からyaw角を計算する
    """
    siny_cosp = 2.0 * q4 * q3
    cosy_cosp = 1.0 - 2.0 * q3 * q3
    return math.atan2(siny_cosp, cosy_cosp)


def load_waypoints(csv_path: str) -> List[Dict]:
    """
    CSVファイルからウェイポイントを読み込む
    """
    waypoints = []
    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append(row)
    return waypoints


def plot_waypoints(
    csv_path: str,
    output_path: Optional[str] = None,
    show: bool = True,
    show_labels: bool = False,
    show_arrows: bool = True,
    arrow_scale: float = 0.8,
    figsize: tuple = (14, 10)
):
    """
    ウェイポイントをグラフ上に描画する

    Parameters:
    -----------
    csv_path : str
        入力CSVファイルのパス
    output_path : str, optional
        出力画像ファイルのパス
    show : bool
        グラフを表示するかどうか
    show_labels : bool
        ラベル番号を表示するかどうか
    show_arrows : bool
        進行方向の矢印を表示するかどうか
    arrow_scale : float
        矢印の長さスケール
    figsize : tuple
        図のサイズ
    """
    waypoints = load_waypoints(csv_path)
    print(f"Loaded {len(waypoints)} waypoints from {csv_path}")

    # データを抽出
    x_coords = []
    y_coords = []
    yaws = []
    labels = []
    line_stops = []
    signal_stops = []

    for wp in waypoints:
        x = float(wp.get('x', 0))
        y = float(wp.get('y', 0))
        q3 = float(wp.get('q3', 0))
        q4 = float(wp.get('q4', 1))
        yaw = quaternion_to_yaw(q3, q4)

        x_coords.append(x)
        y_coords.append(y)
        yaws.append(yaw)
        labels.append(wp.get('label', ''))

        line_stop = int(wp.get('line_is_stop', 0) or 0)
        signal_stop = int(wp.get('signal_is_stop', 0) or 0)
        line_stops.append(line_stop)
        signal_stops.append(signal_stop)

    x_coords = np.array(x_coords)
    y_coords = np.array(y_coords)
    yaws = np.array(yaws)
    line_stops = np.array(line_stops)
    signal_stops = np.array(signal_stops)

    # 図の作成
    fig, ax = plt.subplots(figsize=figsize)

    # 通常のウェイポイント（フラグなし）
    normal_mask = (line_stops == 0) & (signal_stops == 0)
    ax.scatter(
        x_coords[normal_mask],
        y_coords[normal_mask],
        c='blue',
        s=30,
        alpha=0.7,
        label='Normal waypoint',
        zorder=2
    )

    # line_is_stop のウェイポイント
    line_stop_mask = line_stops == 1
    ax.scatter(
        x_coords[line_stop_mask],
        y_coords[line_stop_mask],
        c='red',
        s=100,
        marker='s',
        alpha=0.9,
        label='Line stop',
        zorder=3
    )

    # signal_is_stop のウェイポイント
    signal_stop_mask = signal_stops == 1
    ax.scatter(
        x_coords[signal_stop_mask],
        y_coords[signal_stop_mask],
        c='orange',
        s=100,
        marker='^',
        alpha=0.9,
        label='Signal stop',
        zorder=3
    )

    # 両方立っているウェイポイント
    both_mask = (line_stops == 1) & (signal_stops == 1)
    ax.scatter(
        x_coords[both_mask],
        y_coords[both_mask],
        c='purple',
        s=150,
        marker='*',
        alpha=0.9,
        label='Line + Signal stop',
        zorder=4
    )

    # 経路線を描画
    ax.plot(x_coords, y_coords, 'b-', alpha=0.3, linewidth=1, zorder=1)

    # 進行方向の矢印を描画
    if show_arrows:
        for i in range(len(x_coords)):
            dx = arrow_scale * math.cos(yaws[i])
            dy = arrow_scale * math.sin(yaws[i])

            # フラグによって矢印の色を変える
            if line_stops[i] == 1 and signal_stops[i] == 1:
                color = 'purple'
            elif line_stops[i] == 1:
                color = 'red'
            elif signal_stops[i] == 1:
                color = 'orange'
            else:
                color = 'green'

            ax.arrow(
                x_coords[i], y_coords[i],
                dx, dy,
                head_width=0.3,
                head_length=0.15,
                fc=color,
                ec=color,
                alpha=0.6,
                zorder=2
            )

    # ラベルを表示
    if show_labels:
        for i, label in enumerate(labels):
            ax.annotate(
                label,
                (x_coords[i], y_coords[i]),
                textcoords="offset points",
                xytext=(5, 5),
                fontsize=6,
                alpha=0.7
            )

    # スタートとゴールをマーク
    if len(x_coords) > 0:
        ax.scatter(
            x_coords[0], y_coords[0],
            c='lime', s=200, marker='o', edgecolors='black',
            linewidths=2, label='Start', zorder=5
        )
        ax.scatter(
            x_coords[-1], y_coords[-1],
            c='magenta', s=200, marker='X', edgecolors='black',
            linewidths=2, label='Goal', zorder=5
        )

    # グラフ設定
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_title(f'Waypoint Map: {Path(csv_path).name}', fontsize=14)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', fontsize=9)

    # 統計情報を表示
    stats_text = (
        f"Total: {len(waypoints)}\n"
        f"Line stops: {np.sum(line_stops)}\n"
        f"Signal stops: {np.sum(signal_stops)}"
    )
    ax.text(
        0.98, 0.02, stats_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment='bottom',
        horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    )

    plt.tight_layout()

    # 保存
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved to {output_path}")

    # 表示
    if show:
        plt.show()

    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Plot waypoints on a graph'
    )
    parser.add_argument(
        '--input', '-i',
        type=str,
        required=True,
        help='Path to input CSV file'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Path to output image file (default: input_plot.png)'
    )
    parser.add_argument(
        '--show',
        action='store_true',
        default=False,
        help='Show the plot interactively'
    )
    parser.add_argument(
        '--labels',
        action='store_true',
        default=False,
        help='Show waypoint labels'
    )
    parser.add_argument(
        '--no-arrows',
        action='store_true',
        default=False,
        help='Hide direction arrows'
    )
    parser.add_argument(
        '--arrow-scale',
        type=float,
        default=0.8,
        help='Arrow length scale (default: 0.8)'
    )
    parser.add_argument(
        '--figsize',
        type=str,
        default='14,10',
        help='Figure size as "width,height" (default: 14,10)'
    )

    args = parser.parse_args()

    # 入力ファイルの存在確認
    if not os.path.exists(args.input):
        raise FileNotFoundError(f"Input file not found: {args.input}")

    # 出力ファイル名を決定
    if args.output is None:
        input_path = Path(args.input)
        output_path = str(input_path.parent / (input_path.stem + '_plot.png'))
    else:
        output_path = args.output

    # figsize をパース
    figsize = tuple(map(float, args.figsize.split(',')))

    print(f"Input: {args.input}")
    print(f"Output: {output_path}")
    print("=" * 60)

    plot_waypoints(
        csv_path=args.input,
        output_path=output_path,
        show=args.show,
        show_labels=args.labels,
        show_arrows=not args.no_arrows,
        arrow_scale=args.arrow_scale,
        figsize=figsize
    )

    print("=" * 60)
    print("Done!")


if __name__ == '__main__':
    main()
