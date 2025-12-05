#!/usr/bin/env python3
# coding=utf-8
"""
line_is_stop または signal_is_stop フラグが立っているウェイポイントの
XY座標を進行方向から見て指定距離だけ後ろに移動するスクリプト

使用方法:
    python3 adjust_stop_waypoints.py --input <入力CSV> [--output <出力CSV>] [--offset 0.5]
"""

import argparse
import csv
import math
import os
from pathlib import Path


def quaternion_to_yaw(q3: float, q4: float) -> float:
    """
    クォータニオン(q3=z, q4=w)からyaw角を計算する

    Parameters:
    -----------
    q3 : float
        クォータニオンのz成分
    q4 : float
        クォータニオンのw成分

    Returns:
    --------
    float
        yaw角（ラジアン）
    """
    # 2D回転の場合、q1=q2=0なので簡略化
    # yaw = atan2(2*(w*z), 1 - 2*(z*z))
    siny_cosp = 2.0 * q4 * q3
    cosy_cosp = 1.0 - 2.0 * q3 * q3
    return math.atan2(siny_cosp, cosy_cosp)


def adjust_stop_waypoints(input_path: str, output_path: str, offset: float = 0.5):
    """
    line_is_stop/signal_is_stopフラグが立っているウェイポイントを
    進行方向から見て後ろにオフセットする

    Parameters:
    -----------
    input_path : str
        入力CSVファイルのパス
    output_path : str
        出力CSVファイルのパス
    offset : float
        後ろにずらす距離（メートル）
    """
    rows = []
    fieldnames = []

    # CSVを読み込む
    with open(input_path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            rows.append(row)

    print(f"Loaded {len(rows)} waypoints from {input_path}")
    print(f"Columns: {fieldnames}")

    # line_is_stop, signal_is_stop の列インデックスを確認
    line_stop_col = 'line_is_stop' if 'line_is_stop' in fieldnames else None
    signal_stop_col = 'signal_is_stop' if 'signal_is_stop' in fieldnames else None

    if not line_stop_col and not signal_stop_col:
        print("Error: line_is_stop または signal_is_stop 列が見つかりません")
        return

    adjusted_count = 0

    for i, row in enumerate(rows):
        # フラグをチェック
        line_stop = int(row.get(line_stop_col, 0) or 0) if line_stop_col else 0
        signal_stop = int(row.get(signal_stop_col, 0) or 0) if signal_stop_col else 0

        if line_stop == 1 or signal_stop == 1:
            # 現在の座標を取得
            x = float(row['x'])
            y = float(row['y'])

            # クォータニオンからyaw角を取得
            q3 = float(row.get('q3', 0))
            q4 = float(row.get('q4', 1))
            yaw = quaternion_to_yaw(q3, q4)

            # 進行方向の逆方向（後ろ）にオフセット
            # 進行方向: (cos(yaw), sin(yaw))
            # 後ろ方向: (-cos(yaw), -sin(yaw))
            x_new = x - offset * math.cos(yaw)
            y_new = y - offset * math.sin(yaw)

            # 更新
            row['x'] = str(x_new)
            row['y'] = str(y_new)

            # x_rotated, y_rotated 列があれば同様に更新
            if 'x_rotated' in row and row['x_rotated']:
                row['x_rotated'] = str(x_new)
            if 'y_rotated' in row and row['y_rotated']:
                row['y_rotated'] = str(y_new)

            flag_type = []
            if line_stop == 1:
                flag_type.append("line_stop")
            if signal_stop == 1:
                flag_type.append("signal_stop")

            print(f"  [{row.get('label', i)}] {', '.join(flag_type)}: "
                  f"({x:.3f}, {y:.3f}) -> ({x_new:.3f}, {y_new:.3f}) "
                  f"[yaw={math.degrees(yaw):.1f}°, offset={offset}m]")

            adjusted_count += 1

    # CSVを書き出す
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nAdjusted {adjusted_count} waypoints")
    print(f"Saved to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Adjust stop waypoint positions backward along travel direction'
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
        help='Path to output CSV file (default: input_adjusted.csv)'
    )
    parser.add_argument(
        '--offset',
        type=float,
        default=0.5,
        help='Offset distance in meters (default: 0.5)'
    )

    args = parser.parse_args()

    # 入力ファイルの存在確認
    if not os.path.exists(args.input):
        raise FileNotFoundError(f"Input file not found: {args.input}")

    # 出力ファイル名を決定
    if args.output is None:
        input_path = Path(args.input)
        output_path = str(input_path.parent / (input_path.stem + '_adjusted' + input_path.suffix))
    else:
        output_path = args.output

    print(f"Input: {args.input}")
    print(f"Output: {output_path}")
    print(f"Offset: {args.offset} m (backward)")
    print("=" * 60)

    adjust_stop_waypoints(args.input, output_path, args.offset)

    print("=" * 60)
    print("Done!")


if __name__ == '__main__':
    main()
