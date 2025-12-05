#!/usr/bin/env python3
# coding=utf-8

"""
Launch Switcher GUI

ボタン操作でローカリゼーション用のlaunchファイルを切り替えるGUI

使用方法:
    python3 launch_switcher_gui.py

機能:
    - FASTLIO (1st): 3-2_locarization_fastlio.launch を起動
    - GPS: 現在のlaunchを停止し、3-2_locarization_gps.launch を起動
    - FASTLIO (2nd): 3-2_locarization_fastlio_2nd.launch を起動
    - STOP: 現在のlaunchを停止
"""

import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import signal
import os
import threading
import time


class LaunchSwitcherGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Launch Switcher")
        self.root.geometry("400x350")
        self.root.resizable(False, False)

        # 現在実行中のプロセス
        self.current_process = None
        self.current_launch = None

        # ログ用のロック
        self.log_lock = threading.Lock()

        # 切り替え中フラグ
        self.switching = False

        self.setup_ui()

    def setup_ui(self):
        """UIをセットアップ"""
        # メインフレーム
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # タイトル
        title_label = ttk.Label(
            main_frame,
            text="Localization Launch Switcher",
            font=("Helvetica", 14, "bold")
        )
        title_label.pack(pady=(0, 15))

        # ステータス表示
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="5")
        status_frame.pack(fill=tk.X, pady=(0, 15))

        self.status_label = ttk.Label(
            status_frame,
            text="停止中",
            font=("Helvetica", 12),
            foreground="gray"
        )
        self.status_label.pack()

        # ボタンフレーム
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=(0, 15))

        # FASTLIO (1st) ボタン
        self.btn_fastlio1 = tk.Button(
            button_frame,
            text="FASTLIO (1st)",
            font=("Helvetica", 11),
            bg="#4CAF50",
            fg="white",
            activebackground="#45a049",
            width=15,
            height=2,
            command=lambda: self.start_launch("fastlio")
        )
        self.btn_fastlio1.pack(pady=5)

        # GPS ボタン
        self.btn_gps = tk.Button(
            button_frame,
            text="GPS",
            font=("Helvetica", 11),
            bg="#2196F3",
            fg="white",
            activebackground="#1976D2",
            width=15,
            height=2,
            command=lambda: self.start_launch("gps")
        )
        self.btn_gps.pack(pady=5)

        # FASTLIO (2nd) ボタン
        self.btn_fastlio2 = tk.Button(
            button_frame,
            text="FASTLIO (2nd)",
            font=("Helvetica", 11),
            bg="#FF9800",
            fg="white",
            activebackground="#F57C00",
            width=15,
            height=2,
            command=lambda: self.start_launch("fastlio_2nd")
        )
        self.btn_fastlio2.pack(pady=5)

        # 停止ボタン
        self.btn_stop = tk.Button(
            button_frame,
            text="STOP",
            font=("Helvetica", 11, "bold"),
            bg="#f44336",
            fg="white",
            activebackground="#d32f2f",
            width=15,
            height=2,
            command=self.stop_launch
        )
        self.btn_stop.pack(pady=5)

        # ログ表示エリア
        log_frame = ttk.LabelFrame(main_frame, text="Log", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(log_frame, height=4, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # ウィンドウを閉じるときの処理
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def log(self, message):
        """ログにメッセージを追加"""
        with self.log_lock:
            self.log_text.config(state=tk.NORMAL)
            timestamp = time.strftime("%H:%M:%S")
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)

    def update_status(self, launch_name, running=True):
        """ステータスを更新"""
        if running:
            status_map = {
                "fastlio": "FASTLIO (1st) 実行中",
                "gps": "GPS 実行中",
                "fastlio_2nd": "FASTLIO (2nd) 実行中"
            }
            self.status_label.config(
                text=status_map.get(launch_name, "実行中"),
                foreground="green"
            )
        else:
            self.status_label.config(text="停止中", foreground="gray")

    def get_launch_file(self, launch_name):
        """launch名からファイルパスを取得"""
        launch_files = {
            "fastlio": "3-2_locarization_fastlio.launch",
            "gps": "3-2_locarization_gps.launch",
            "fastlio_2nd": "3-2_locarization_fastlio_2nd.launch"
        }
        return launch_files.get(launch_name)

    def start_launch(self, launch_name):
        """指定したlaunchファイルを起動"""
        # 切り替え中は無視
        if self.switching:
            self.log("Switching in progress, please wait...")
            return

        # バックグラウンドで実行
        thread = threading.Thread(
            target=self._start_launch_thread,
            args=(launch_name,),
            daemon=True
        )
        thread.start()

    def _start_launch_thread(self, launch_name):
        """バックグラウンドでlaunchを開始"""
        self.switching = True
        self.root.after(0, lambda: self.set_buttons_enabled(False))

        try:
            # 現在のプロセスを停止
            if self.current_process is not None:
                self._stop_launch_internal()
                # ROSノードが完全に終了するまで待つ
                self.root.after(0, lambda: self.log("Waiting for nodes to shutdown..."))
                time.sleep(3)

            launch_file = self.get_launch_file(launch_name)
            if launch_file is None:
                self.root.after(0, lambda: self.log(f"Unknown launch: {launch_name}"))
                return

            self.root.after(0, lambda: self.log(f"Starting {launch_file}..."))

            # roslaunchをサブプロセスで起動（出力は/dev/nullへ）
            cmd = ["roslaunch", "tc2025", launch_file]
            with open(os.devnull, 'w') as devnull:
                self.current_process = subprocess.Popen(
                    cmd,
                    stdout=devnull,
                    stderr=devnull,
                    preexec_fn=os.setsid
                )
            self.current_launch = launch_name
            self.root.after(0, lambda: self.update_status(launch_name, running=True))
            self.root.after(0, lambda: self.log(f"{launch_file} started (PID: {self.current_process.pid})"))

            # バックグラウンドでプロセスの終了を監視
            monitor_thread = threading.Thread(
                target=self.monitor_process,
                daemon=True
            )
            monitor_thread.start()

        except Exception as e:
            self.root.after(0, lambda: self.log(f"Error starting launch: {e}"))
        finally:
            self.switching = False
            self.root.after(0, lambda: self.set_buttons_enabled(True))

    def set_buttons_enabled(self, enabled):
        """ボタンの有効/無効を切り替え"""
        state = tk.NORMAL if enabled else tk.DISABLED
        self.btn_fastlio1.config(state=state)
        self.btn_gps.config(state=state)
        self.btn_fastlio2.config(state=state)
        self.btn_stop.config(state=state)

    def monitor_process(self):
        """プロセスの終了を監視"""
        if self.current_process:
            self.current_process.wait()
            if self.current_launch is not None:
                self.root.after(0, lambda: self.log("Launch process ended"))
                self.root.after(0, lambda: self.update_status(None, running=False))
                self.current_process = None
                self.current_launch = None

    def stop_launch(self):
        """現在のlaunchを停止（UIから呼ばれる）"""
        if self.switching:
            self.log("Switching in progress, please wait...")
            return

        if self.current_process is None:
            self.log("No launch running")
            return

        # バックグラウンドで実行
        thread = threading.Thread(
            target=self._stop_launch_thread,
            daemon=True
        )
        thread.start()

    def _stop_launch_thread(self):
        """バックグラウンドでlaunchを停止"""
        self.switching = True
        self.root.after(0, lambda: self.set_buttons_enabled(False))

        try:
            self._stop_launch_internal()
        finally:
            self.switching = False
            self.root.after(0, lambda: self.set_buttons_enabled(True))

    def _stop_launch_internal(self):
        """launchを停止（内部用）"""
        if self.current_process is None:
            return

        launch_file = self.get_launch_file(self.current_launch)
        self.root.after(0, lambda: self.log(f"Stopping {launch_file}..."))

        try:
            # プロセスグループ全体にSIGINTを送信（Ctrl+Cと同じ）
            os.killpg(os.getpgid(self.current_process.pid), signal.SIGINT)

            # 終了を待つ（最大10秒）
            try:
                self.current_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                # タイムアウトしたらSIGTERMを試す
                self.root.after(0, lambda: self.log("Sending SIGTERM..."))
                os.killpg(os.getpgid(self.current_process.pid), signal.SIGTERM)
                try:
                    self.current_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # それでもダメならSIGKILL
                    self.root.after(0, lambda: self.log("Force killing..."))
                    os.killpg(os.getpgid(self.current_process.pid), signal.SIGKILL)
                    self.current_process.wait()

            self.root.after(0, lambda: self.log("Launch stopped"))
            self.root.after(0, lambda: self.update_status(None, running=False))
            self.current_process = None
            self.current_launch = None

        except ProcessLookupError:
            # プロセスが既に終了している
            self.root.after(0, lambda: self.log("Process already terminated"))
            self.current_process = None
            self.current_launch = None
            self.root.after(0, lambda: self.update_status(None, running=False))
        except Exception as e:
            self.root.after(0, lambda: self.log(f"Error stopping launch: {e}"))

    def on_closing(self):
        """ウィンドウを閉じるときの処理"""
        if self.switching:
            messagebox.showwarning("Warning", "処理中です。しばらくお待ちください。")
            return

        if self.current_process is not None:
            result = messagebox.askyesnocancel(
                "Confirm Exit",
                "Launchプロセスが実行中です。\n\n"
                "Yes: 停止して終了\n"
                "No: 実行したまま終了\n"
                "Cancel: キャンセル"
            )
            if result is None:  # Cancel
                return
            elif result:  # Yes
                self._stop_launch_internal()

        self.root.destroy()

    def run(self):
        """GUIを起動"""
        self.root.mainloop()


def main():
    gui = LaunchSwitcherGUI()
    gui.run()


if __name__ == '__main__':
    main()
