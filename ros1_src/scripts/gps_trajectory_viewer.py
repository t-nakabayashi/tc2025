#!/usr/bin/env python3
"""ROSLAUNCHからGPS軌跡ビューアを起動するための薄いラッパースクリプト。"""
import os
import sys


def main():
    """gps/gps_trajectory_viewer.py の main() を呼び出してノードを立ち上げる。"""
    # 現在のscriptsディレクトリを基準に、実体ファイルが存在するgpsサブディレクトリを解決
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    module_dir = os.path.join(scripts_dir, "gps")

    # Pythonのモジュール検索パスにgpsサブディレクトリを追加し、直接インポートできるようにする
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)

    # 実体のメイン関数をインポートして即時に実行
    from gps_trajectory_viewer import main as inner_main

    inner_main()


if __name__ == "__main__":
    main()
