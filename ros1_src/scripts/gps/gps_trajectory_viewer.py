#!/usr/bin/env python3
"""GPSのNavSatFixをリアルタイムに可視化し、RVizへも配信するROSノード。"""
import math
import threading

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from ublox_msgs.msg import NavPVT


class GPSTrajectoryViewer:
    """GPS軌跡をリアルタイムで描画し、FIX状態を色分けするクラス。"""

    def __init__(self):
        # ノード名を設定し、ROSノードを初期化する
        rospy.init_node("gps_trajectory_viewer", anonymous=False)

        # 座標と状態を格納するリストを初期化（Matplotlib描画とRViz出力を兼用）
        self.x_positions = []   # 東西方向の相対距離[m]
        self.y_positions = []   # 南北方向の相対距離[m]
        self.solution_states = []  # 解決状態(FIX/FLOAT/OTHER)を追跡

        # RViz連携関連のパラメータをROSパラメータサーバから取得
        self.frame_id = rospy.get_param("~frame_id", "gps_local")  # RVizで利用する座標系名
        self.max_points = int(rospy.get_param("~max_points", 20000))  # 保存する最大軌跡点数
        self.marker_scale = float(rospy.get_param("~marker_scale", 0.3))  # RViz上の点サイズ
        self.publish_path = rospy.get_param("~publish_path", True)  # Pathトピックの配信有無
        self.publish_marker = rospy.get_param("~publish_marker", True)  # Markerトピックの配信有無
        self.enable_plot = rospy.get_param("~enable_matplotlib", True)  # Matplotlib描画の有効/無効

        # nav_msgs/PathとMarkerのPublisherを用意（必要なもののみ）
        self.path_publishers = {}
        self.path_messages = {}
        if self.publish_path:
            # 解決状態ごとに個別のPathを用意
            for solution_name in ("FIX", "FLOAT", "OTHER"):
                topic_suffix = solution_name.lower()
                self.path_publishers[solution_name] = rospy.Publisher(
                    f"~path_{topic_suffix}", Path, queue_size=1
                )
                path_msg = Path()
                path_msg.header.frame_id = self.frame_id
                self.path_messages[solution_name] = path_msg

        self.marker_pub = rospy.Publisher("~fix_points", Marker, queue_size=1) if self.publish_marker else None

        # RVizで色分け表示するためのMarkerメッセージを準備
        self.marker_msg = Marker()
        self.marker_msg.header.frame_id = self.frame_id
        self.marker_msg.ns = "gps_fix"
        self.marker_msg.id = 0
        self.marker_msg.type = Marker.SPHERE_LIST
        self.marker_msg.action = Marker.ADD
        self.marker_msg.scale.x = self.marker_scale
        self.marker_msg.scale.y = self.marker_scale
        self.marker_msg.scale.z = self.marker_scale
        self.marker_msg.pose.orientation.w = 1.0
        self.marker_msg.lifetime = rospy.Duration(0.0)  # 0で永続表示

        # NavPVTメッセージの受信状況を保持し、RTKのFIX/FLOAT判定へ利用
        self.latest_navpvt_solution = None  # 直近のRTK解決状態（FIX/FLOAT/OTHER）
        self.latest_navpvt_stamp = rospy.Time(0)  # NavPVT受信時刻
        self.navpvt_timeout = rospy.Duration(rospy.get_param("~navpvt_timeout", 1.0))

        # 初回の経緯度を原点として採用するための変数
        self.origin_lat = None
        self.origin_lon = None

        # Matplotlibの描画オブジェクトを準備
        self.figure = None
        self.ax = None
        self.scatter = None  # 点群のハンドルを保持
        if self.enable_plot:
            # MatplotlibのFigureとAxesを生成し、見やすいラベルを設定
            self.figure, self.ax = plt.subplots()
            self.ax.set_xlabel("東西方向距離 [m]")
            self.ax.set_ylabel("南北方向距離 [m]")
            self.ax.set_title("GPS軌跡 (色=FIX状態)")
            self.ax.grid(True)
            self.ax.set_aspect("equal", adjustable="box")
            plt.ion()
            self.figure.show()

        # スレッド間で安全にデータを共有するためのロック
        self.data_lock = threading.Lock()

        # NavSatFixメッセージの購読を開始
        topic_name = rospy.get_param("~topic", "/ublox/fix")
        rospy.loginfo("GPSトピック %s を購読します", topic_name)
        self.subscription = rospy.Subscriber(topic_name, NavSatFix, self._fix_callback, queue_size=10)

        # NavPVTメッセージも購読し、RTKの状態判定に利用する
        navpvt_topic = rospy.get_param("~navpvt_topic", "/ublox/navpvt")
        rospy.loginfo("NavPVTトピック %s を購読します", navpvt_topic)
        self.navpvt_sub = rospy.Subscriber(navpvt_topic, NavPVT, self._navpvt_callback, queue_size=10)
        self.navpvt_warned = False  # NavPVTが欠損している際に過剰ログを抑制

    def _fix_callback(self, msg: NavSatFix):
        """NavSatFixを受け取り、軌跡データを更新するコールバック。"""
        # 有効な位置データが含まれているか確認
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            rospy.logwarn("無効なGPS測位データを受信しました")
            return

        with self.data_lock:
            # 初回は原点を設定
            if self.origin_lat is None or self.origin_lon is None:
                self.origin_lat = msg.latitude
                self.origin_lon = msg.longitude
                rospy.loginfo("原点を lat=%.8f, lon=%.8f に設定", self.origin_lat, self.origin_lon)

            # 経緯度を平面直角座標に近似変換（簡易的なメルカトル投影）
            x, y = self._latlon_to_local_xy(msg.latitude, msg.longitude)
            self.x_positions.append(x)
            self.y_positions.append(y)

            # NavPVTの情報を用いてRTKの解決状態を判定
            solution_state = self._determine_solution_state(msg)
            self.solution_states.append(solution_state)

            # RVizへ送るPoseStampedを生成し、Pathに蓄積
            stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # 水平方向のみなので姿勢は単位クォータニオン

            if self.publish_path:
                path_msg = self.path_messages[solution_state]
                path_msg.poses.append(pose)
                if len(path_msg.poses) > self.max_points:
                    # 古いデータを削除してメモリ使用量を抑制
                    path_msg.poses = path_msg.poses[-self.max_points:]
                path_msg.header.stamp = stamp
                self.path_publishers[solution_state].publish(path_msg)

            if self.publish_marker:
                point = Point(x=x, y=y, z=0.0)
                color = ColorRGBA()
                if solution_state == "FIX":
                    # RTK固定解（Integer Fix）は緑で表示
                    color.r = 0.0
                    color.g = 0.8
                    color.b = 0.0
                elif solution_state == "FLOAT":
                    # RTK浮動解（Float）は橙色で表示
                    color.r = 1.0
                    color.g = 0.55
                    color.b = 0.0
                else:
                    # その他はグレーで表示
                    color.r = 0.5
                    color.g = 0.5
                    color.b = 0.5
                color.a = 0.9

                self.marker_msg.points.append(point)
                self.marker_msg.colors.append(color)
                if len(self.marker_msg.points) > self.max_points:
                    # 点数が多すぎる場合は末尾側だけ残す
                    self.marker_msg.points = self.marker_msg.points[-self.max_points:]
                    self.marker_msg.colors = self.marker_msg.colors[-self.max_points:]
                self.marker_msg.header.stamp = stamp
                self.marker_pub.publish(self.marker_msg)

            # Matplotlib用の座標配列も最大長を超えたら整理
            if len(self.x_positions) > self.max_points:
                self.x_positions = self.x_positions[-self.max_points:]
                self.y_positions = self.y_positions[-self.max_points:]
                self.solution_states = self.solution_states[-self.max_points:]

    def _navpvt_callback(self, msg: NavPVT):
        """NavPVTメッセージを受信し、RTK解決状態を最新化する。"""
        with self.data_lock:
            # flagsのキャリア位相解情報からRTKのステータスを抽出
            carrier_state = msg.flags & NavPVT.FLAGS_CARRIER_PHASE_MASK

            if not (msg.flags & NavPVT.FLAGS_GNSS_FIX_OK):
                # GNSS自体がFIXしていない場合はOTHER扱い
                solution = "OTHER"
            elif carrier_state == NavPVT.CARRIER_PHASE_FIXED:
                # キャリア位相が固定解（Integer Fix）
                solution = "FIX"
            elif carrier_state == NavPVT.CARRIER_PHASE_FLOAT:
                # キャリア位相が浮動解（Float）
                solution = "FLOAT"
            else:
                # それ以外は固定解でないためOTHER
                solution = "OTHER"

            self.latest_navpvt_solution = solution
            self.latest_navpvt_stamp = rospy.Time.now()

    def _determine_solution_state(self, navsat: NavSatFix) -> str:
        """NavSatFixと直近のNavPVT情報から解決状態を判定する。"""
        # NavPVTが新鮮であればそれを優先して判定
        if self.latest_navpvt_solution is not None:
            now = rospy.Time.now()
            if now - self.latest_navpvt_stamp <= self.navpvt_timeout:
                return self.latest_navpvt_solution

        # NavPVTが得られない場合はRTK状態を判断できないためOTHERとみなす
        if not self.navpvt_warned:
            rospy.logwarn("NavPVT情報が得られないためRTK状態をOTHERとして扱います")
            self.navpvt_warned = True
        return "OTHER"

    def _latlon_to_local_xy(self, lat: float, lon: float):
        """経緯度から原点基準の相対座標[m]を求める補助関数。"""
        # 原点が未設定の場合は0,0を返す
        if self.origin_lat is None or self.origin_lon is None:
            return 0.0, 0.0

        # 緯度1度あたりの距離は約111,319.49m
        meters_per_deg_lat = 111_319.49
        # 経度方向は緯度によって変動するので、原点緯度の余弦を掛ける
        meters_per_deg_lon = meters_per_deg_lat * math.cos(math.radians(self.origin_lat))

        delta_lat = lat - self.origin_lat
        delta_lon = lon - self.origin_lon

        x = delta_lon * meters_per_deg_lon
        y = delta_lat * meters_per_deg_lat
        return x, y

    def update_plot(self):
        """新しいデータがあれば描画を更新する。"""
        if not self.enable_plot:
            return

        with self.data_lock:
            if not self.x_positions:
                return

            # FIX状態ごとに色を割り当てる
            colors = []
            for state in self.solution_states:
                if state == "FIX":
                    colors.append("tab:green")
                elif state == "FLOAT":
                    colors.append("tab:orange")
                else:
                    colors.append("tab:gray")

            if self.scatter is None:
                # 初回は散布図を作成
                self.scatter = self.ax.scatter(self.x_positions, self.y_positions, c=colors, s=20)
            else:
                # 既存の散布図を更新
                self.scatter.set_offsets(list(zip(self.x_positions, self.y_positions)))
                self.scatter.set_color(colors)

            # 描画範囲を自動調整して見やすさを確保
            self.ax.relim()
            self.ax.autoscale_view()

        # GUIイベントを処理し、最新状態を表示
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def spin(self):
        """ROSループを回しながら定期的に描画を更新する。"""
        rate_hz = rospy.get_param("~refresh_rate", 5.0)
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()


def main():
    """エントリーポイント。ノードを生成して稼働させる。"""
    viewer = GPSTrajectoryViewer()
    try:
        viewer.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPS軌跡ビューアを終了します")
    finally:
        if viewer.enable_plot:
            plt.ioff()
            plt.show()


if __name__ == "__main__":
    main()
