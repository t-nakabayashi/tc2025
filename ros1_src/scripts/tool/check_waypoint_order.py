#!/usr/bin/env python3
# coding=utf-8
"""
ウェイポイントの順序整合性をチェックするスクリプト

line_is_stop/signal_is_stop フラグが立っているウェイポイントを
指定距離だけ後ろにずらした場合に、前後のウェイポイントとの
位置関係が逆転しないかを確認する。

使用方法:
    python3 check_waypoint_order.py --input <入力CSV> [--offset 0.5]

チェック内容:
    1. ずらした後の位置が、前のウェイポイントより後ろになっていないか
    2. ずらした後の位置が、前のウェイポイントと重なっていないか
    3. 連続するstopウェイポイント間の距離が近すぎないか
"""

import argparse
import csv
import math
import os
from pathlib import Path
from typing import List, Dict, Tuple, Optional


def quaternion_to_yaw(q3: float, q4: float) -> float:
    """
    クォータニオン(q3=z, q4=w)からyaw角を計算する
    """
    siny_cosp = 2.0 * q4 * q3
    cosy_cosp = 1.0 - 2.0 * q3 * q3
    return math.atan2(siny_cosp, cosy_cosp)


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """2点間の距離を計算"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def dot_product_along_direction(
    px: float, py: float,
    ref_x: float, ref_y: float,
    dir_x: float, dir_y: float
) -> float:
    """
    参照点から点Pへのベクトルを、方向ベクトルに射影した値を返す
    正の値: 方向ベクトルと同じ向き（前方）
    負の値: 方向ベクトルと逆向き（後方）
    """
    dx = px - ref_x
    dy = py - ref_y
    return dx * dir_x + dy * dir_y


def load_waypoints(csv_path: str) -> List[Dict]:
    """CSVファイルからウェイポイントを読み込む"""
    waypoints = []
    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append(row)
    return waypoints


def check_waypoint_order(
    csv_path: str,
    offset: float = 0.5,
    min_distance_threshold: float = 0.3
) -> List[Dict]:
    """
    ウェイポイントの順序整合性をチェックする

    Parameters:
    -----------
    csv_path : str
        入力CSVファイルのパス
    offset : float
        後ろにずらす距離（メートル）
    min_distance_threshold : float
        最小距離の閾値（これ以下だと警告）

    Returns:
    --------
    List[Dict]
        警告のリスト
    """
    waypoints = load_waypoints(csv_path)
    print(f"Loaded {len(waypoints)} waypoints from {csv_path}")
    print(f"Checking with offset: {offset} m")
    print("=" * 70)

    alerts = []

    for i, wp in enumerate(waypoints):
        # フラグをチェック
        line_stop = int(wp.get('line_is_stop', 0) or 0)
        signal_stop = int(wp.get('signal_is_stop', 0) or 0)

        if line_stop != 1 and signal_stop != 1:
            continue

        # 現在のウェイポイント情報
        label = wp.get('label', str(i))
        x = float(wp['x'])
        y = float(wp['y'])
        q3 = float(wp.get('q3', 0))
        q4 = float(wp.get('q4', 1))
        yaw = quaternion_to_yaw(q3, q4)

        # 進行方向の単位ベクトル
        dir_x = math.cos(yaw)
        dir_y = math.sin(yaw)

        # オフセット後の位置
        x_new = x - offset * dir_x
        y_new = y - offset * dir_y

        flag_type = []
        if line_stop == 1:
            flag_type.append("line_stop")
        if signal_stop == 1:
            flag_type.append("signal_stop")

        # 前のウェイポイントとの比較
        if i > 0:
            prev_wp = waypoints[i - 1]
            prev_x = float(prev_wp['x'])
            prev_y = float(prev_wp['y'])
            prev_label = prev_wp.get('label', str(i - 1))

            # オフセット前後の距離
            dist_before = distance(prev_x, prev_y, x, y)
            dist_after = distance(prev_x, prev_y, x_new, y_new)

            # 前のウェイポイントから見て、オフセット後の位置が進行方向に対してどうか
            # 前のウェイポイントの進行方向を使用
            prev_q3 = float(prev_wp.get('q3', 0))
            prev_q4 = float(prev_wp.get('q4', 1))
            prev_yaw = quaternion_to_yaw(prev_q3, prev_q4)
            prev_dir_x = math.cos(prev_yaw)
            prev_dir_y = math.sin(prev_yaw)

            # 前のウェイポイントからオフセット後の位置への射影
            proj_after = dot_product_along_direction(
                x_new, y_new, prev_x, prev_y, prev_dir_x, prev_dir_y
            )

            # 前のウェイポイントからオフセット前の位置への射影
            proj_before = dot_product_along_direction(
                x, y, prev_x, prev_y, prev_dir_x, prev_dir_y
            )

            # チェック1: オフセット後の位置が前のウェイポイントより後ろになっていないか
            if proj_after < 0:
                alert = {
                    'type': 'ORDER_REVERSAL',
                    'severity': 'ERROR',
                    'label': label,
                    'index': i,
                    'flags': flag_type,
                    'message': f"オフセット後の位置が前のWP[{prev_label}]より後ろに！",
                    'details': {
                        'prev_label': prev_label,
                        'proj_before': proj_before,
                        'proj_after': proj_after,
                        'original_pos': (x, y),
                        'new_pos': (x_new, y_new),
                        'prev_pos': (prev_x, prev_y)
                    }
                }
                alerts.append(alert)

            # チェック2: オフセット後の距離が非常に近い
            if dist_after < min_distance_threshold:
                alert = {
                    'type': 'TOO_CLOSE',
                    'severity': 'WARNING',
                    'label': label,
                    'index': i,
                    'flags': flag_type,
                    'message': f"オフセット後、前のWP[{prev_label}]との距離が{dist_after:.3f}mと近すぎる",
                    'details': {
                        'prev_label': prev_label,
                        'dist_before': dist_before,
                        'dist_after': dist_after
                    }
                }
                alerts.append(alert)

            # チェック3: オフセット前の距離がオフセット値より小さい
            if dist_before < offset:
                alert = {
                    'type': 'INSUFFICIENT_DISTANCE',
                    'severity': 'WARNING',
                    'label': label,
                    'index': i,
                    'flags': flag_type,
                    'message': f"前のWP[{prev_label}]との距離({dist_before:.3f}m)がオフセット値({offset}m)より小さい",
                    'details': {
                        'prev_label': prev_label,
                        'dist_before': dist_before,
                        'offset': offset
                    }
                }
                alerts.append(alert)

        # 次のウェイポイントとの比較
        if i < len(waypoints) - 1:
            next_wp = waypoints[i + 1]
            next_x = float(next_wp['x'])
            next_y = float(next_wp['y'])
            next_label = next_wp.get('label', str(i + 1))

            # オフセット前後の距離
            dist_to_next_before = distance(x, y, next_x, next_y)
            dist_to_next_after = distance(x_new, y_new, next_x, next_y)

            # チェック4: 次のウェイポイントとの距離が大幅に増加
            if dist_to_next_after > dist_to_next_before * 1.5 and dist_to_next_before < 2.0:
                alert = {
                    'type': 'DISTANCE_INCREASE',
                    'severity': 'INFO',
                    'label': label,
                    'index': i,
                    'flags': flag_type,
                    'message': f"次のWP[{next_label}]との距離が{dist_to_next_before:.3f}m→{dist_to_next_after:.3f}mに増加",
                    'details': {
                        'next_label': next_label,
                        'dist_before': dist_to_next_before,
                        'dist_after': dist_to_next_after
                    }
                }
                alerts.append(alert)

    return alerts


def print_alerts(alerts: List[Dict]):
    """アラートを表示する"""
    if not alerts:
        print("\n✓ 問題は検出されませんでした。")
        return

    # 重大度でソート
    severity_order = {'ERROR': 0, 'WARNING': 1, 'INFO': 2}
    alerts_sorted = sorted(alerts, key=lambda x: severity_order.get(x['severity'], 99))

    # 重大度ごとにカウント
    error_count = sum(1 for a in alerts if a['severity'] == 'ERROR')
    warning_count = sum(1 for a in alerts if a['severity'] == 'WARNING')
    info_count = sum(1 for a in alerts if a['severity'] == 'INFO')

    print(f"\n検出された問題: ERROR={error_count}, WARNING={warning_count}, INFO={info_count}")
    print("=" * 70)

    for alert in alerts_sorted:
        severity = alert['severity']
        if severity == 'ERROR':
            icon = '❌'
        elif severity == 'WARNING':
            icon = '⚠️'
        else:
            icon = 'ℹ️'

        print(f"\n{icon} [{severity}] WP[{alert['label']}] (index={alert['index']})")
        print(f"   フラグ: {', '.join(alert['flags'])}")
        print(f"   {alert['message']}")

        if alert['type'] == 'ORDER_REVERSAL':
            details = alert['details']
            print(f"   元の位置: ({details['original_pos'][0]:.3f}, {details['original_pos'][1]:.3f})")
            print(f"   新しい位置: ({details['new_pos'][0]:.3f}, {details['new_pos'][1]:.3f})")
            print(f"   前のWP位置: ({details['prev_pos'][0]:.3f}, {details['prev_pos'][1]:.3f})")
            print(f"   射影値: {details['proj_before']:.3f} → {details['proj_after']:.3f}")


def main():
    parser = argparse.ArgumentParser(
        description='Check waypoint order consistency after offset adjustment'
    )
    parser.add_argument(
        '--input', '-i',
        type=str,
        required=True,
        help='Path to input CSV file'
    )
    parser.add_argument(
        '--offset',
        type=float,
        default=0.5,
        help='Offset distance in meters (default: 0.5)'
    )
    parser.add_argument(
        '--threshold',
        type=float,
        default=0.3,
        help='Minimum distance threshold for warning (default: 0.3)'
    )

    args = parser.parse_args()

    # 入力ファイルの存在確認
    if not os.path.exists(args.input):
        raise FileNotFoundError(f"Input file not found: {args.input}")

    print(f"Input: {args.input}")
    print(f"Offset: {args.offset} m")
    print(f"Min distance threshold: {args.threshold} m")

    alerts = check_waypoint_order(
        csv_path=args.input,
        offset=args.offset,
        min_distance_threshold=args.threshold
    )

    print_alerts(alerts)

    print("\n" + "=" * 70)
    print("Done!")

    # エラーがあった場合は終了コード1を返す
    if any(a['severity'] == 'ERROR' for a in alerts):
        exit(1)


if __name__ == '__main__':
    main()
