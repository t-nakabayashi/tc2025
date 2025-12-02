#!/usr/bin/env python3
# coding=utf-8

import os
import sys
import argparse

def read_pcd_header(file_path):
    """PCDファイルのヘッダーを読み込む"""
    header = []
    data_start = 0

    with open(file_path, 'rb') as f:
        while True:
            line = f.readline()
            data_start = f.tell()

            try:
                line_str = line.decode('ascii').strip()
            except:
                break

            header.append(line_str)

            if line_str.startswith('DATA'):
                break

    return header, data_start

def read_pcd_data(file_path, data_start):
    """PCDファイルのデータ部分を読み込む"""
    with open(file_path, 'rb') as f:
        f.seek(data_start)
        data = f.read()
    return data

def get_point_count(header):
    """ヘッダーからポイント数を取得"""
    for line in header:
        if line.startswith('POINTS'):
            return int(line.split()[1])
    return 0

def merge_pcd_files(input_files, output_file):
    """複数のPCDファイルを1つに統合"""

    print(f"Merging {len(input_files)} PCD files...")

    # 最初のファイルからヘッダー情報を取得
    first_header, first_data_start = read_pcd_header(input_files[0])

    # 全ファイルのデータを読み込み
    all_data = []
    total_points = 0

    for i, input_file in enumerate(input_files):
        if not os.path.exists(input_file):
            print(f"Warning: File not found: {input_file}")
            continue

        print(f"Reading file {i+1}/{len(input_files)}: {os.path.basename(input_file)}")

        header, data_start = read_pcd_header(input_file)
        point_count = get_point_count(header)
        data = read_pcd_data(input_file, data_start)

        all_data.append(data)
        total_points += point_count

        print(f"  Points: {point_count}")

    print(f"\nTotal points: {total_points}")

    # 新しいヘッダーを作成（POINTSとWIDTHを更新）
    new_header = []
    for line in first_header:
        if line.startswith('POINTS'):
            new_header.append(f'POINTS {total_points}')
        elif line.startswith('WIDTH'):
            new_header.append(f'WIDTH {total_points}')
        else:
            new_header.append(line)

    # 出力ファイルに書き込み
    print(f"\nWriting merged PCD to: {output_file}")

    with open(output_file, 'wb') as f:
        # ヘッダーを書き込み
        for line in new_header:
            f.write((line + '\n').encode('ascii'))

        # 全データを書き込み
        for data in all_data:
            f.write(data)

    print(f"Successfully merged {len(input_files)} files into {output_file}")
    print(f"Output file size: {os.path.getsize(output_file) / (1024*1024):.2f} MB")

def main():
    parser = argparse.ArgumentParser(
        description='Merge multiple PCD files into one'
    )
    parser.add_argument(
        '--input-dir',
        type=str,
        default='/home/nkb/catkin_ws/src/FAST_LIO/PCD',
        help='Directory containing input PCD files'
    )
    parser.add_argument(
        '--pattern',
        type=str,
        default='scans_{}_rot.pcd',
        help='File name pattern with {} as placeholder for number'
    )
    parser.add_argument(
        '--start',
        type=int,
        default=1,
        help='Start number (default: 1)'
    )
    parser.add_argument(
        '--end',
        type=int,
        default=3,
        help='End number (default: 14)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='/home/nkb/catkin_ws/src/FAST_LIO/PCD/scans_merged.pcd',
        help='Output merged PCD file path'
    )

    args = parser.parse_args()

    # 入力ファイルリストを生成
    input_files = []
    for i in range(args.start, args.end + 1):
        file_name = args.pattern.format(i)
        file_path = os.path.join(args.input_dir, file_name)
        input_files.append(file_path)

    print(f"Input directory: {args.input_dir}")
    print(f"File pattern: {args.pattern}")
    print(f"Range: {args.start} to {args.end}")
    print(f"Output file: {args.output}")
    print(f"\nFiles to merge:")
    for f in input_files:
        print(f"  - {os.path.basename(f)}")
    print()

    # PCDファイルを統合
    merge_pcd_files(input_files, args.output)

if __name__ == '__main__':
    main()
