# robot_console パッケージ README (phase3 実装版)

## 概要
`robot_console` は ROS2 ベースの自律走行システムを遠隔監視・操作するための統合ダッシュボードです。`rclpy` ノード (`RobotConsoleNode`) と tkinter GUI (`UiMain`) を同一プロセスで動作させ、走行状態・障害物回避・経路進捗・ノード起動状態を一画面で俯瞰できます。GUI と ROS 通信は `GuiCore` が橋渡しし、GUI 操作からのコマンドと ROS トピックからの最新情報を双方向に同期します。

## 主な機能
- `/route_state`・`/manager_status`・`/follower_state` などのトピックを購読し、走行状況や再計画履歴をカード形式で可視化。
- `/sensor_viewer` および外部カメラ映像（走行・信号監視）を 3 つの画像パネルに表示し、障害物ヒントをオーバレイ。
- `manual_start`・`sig_recog`・`road_blocked`・`obstacle_avoidance_hint` の送信 UI を備え、運用者がラッチ値や回避指示を即時に発行可能。
- `NodeLaunchManager` により `ros2 launch` コマンドを GUI から起動／停止し、主要ノードの稼働状況とログをサイドバーとタブで確認。
- 走行距離・速度・目標到達率を自動算出し、閾値を超えた場合に色分けで警告。
- `tools/mock_ui.py` を用いたダミーデータ表示と `tools/tests/` によるロジック単体テストで回帰検出に対応。

## 画面構成
### Dashboard タブ
- **ステータスカード列**：ルート進捗、フォロワ状態、速度・目標距離カードを 5Hz 以内で更新。
- **イベントバナー**：`road_blocked` → `manual_start` → `sig_recog` の優先順位で最新イベントを表示。60 秒経過で自動クリア。
- **制御コマンドタブ**：各トピックの送信 UI を Notebook 形式でまとめ、Spinbox やラジオボタンで値を入力。`frame_image_path` タグでは静止画パスを入力して `/frame_image_path` トピックへ単発 publish できます。送信結果は最終送信値と時刻として即座に反映されます。
- **画像パネル**：ルート地図（`/active_route` の付帯画像）、障害物ビュー（`/sensor_viewer`）、カメラ映像（`/usb_cam/image_raw`・`/sig_det_imgs`）。レターボックス処理でアスペクト比を保持し、障害物ヒント値を左上にオーバレイ表示。
- **ノード起動サイドバー**：主要ノードカードに加え `Yolo Detector` カードを配置。インストール済み `road_blockage_detector` の `params/` 配下から YAML を自動列挙し、`yolo_ncnn_node` / `yolo_node` をトグルスイッチで切り替えて起動できます。Simulator をオンにすると `camera_simulator_node` を同時起動し、他カード同様に起動・停止やパラメータ切替が可能です。

### Console Logs タブ
- ノードごとに最新ログをリングバッファで保持。GUI 右クリックからコピーでき、検索バーでフィルタリング可能です。
- 各セクションには `RUNNING/STOPPED/ERROR` インジケータと、直近の起動／停止時刻が表示されます。
- `Yolo Detector` も他ノード同様にパラメータ一覧・コンソールログタブへカードを追加し、選択した `road_blockage_detector` の YAML 内容表示やログファイルの直接オープンが可能です。

## 起動方法
### 通常起動
```bash
ros2 launch robot_console robot_console.launch.py \
  config_file:=/path/to/robot_console.yaml
```
- `config_file` は任意の YAML で、運用環境に合わせて `ros2_src/robot_console/config/` などに配置してください（ディレクトリが存在しない場合は作成が必要です）。省略時はノード組み込みの既定値が使用されます。
- GUI は 1280x720 を基準解像度とし、リサイズ時は比率を維持します。

### 主要パラメータ
| パラメータ | 型 | 既定値 | 説明 |
|------------|----|--------|------|
| `ui.refresh_period_ms` | int | 200 | GUI が `GuiCore.snapshot()` を呼ぶ周期。|
| `ui.image_rate_limit_hz.*` | double | route:2 / sensor:5 / camera_drive:5 / camera_signal:5 | 画像パネルごとの最大更新頻度。|
| `commands.cooldown_ms.*` | int | 500 | manual_start / sig_recog / road_blocked の連打抑止時間。|
| `commands.override_timer_hz.obstacle_hint` | double | 0.5 | 障害物ヒント固定送出の周期。|
| `topics.camera.drive` / `topics.camera.signal` | string | `/usb_cam/image_raw` / `/sig_det_imgs` | 外部カメラの購読トピック名。|
| `topics.road_blocked.external_priority` | bool | true | 外部 `/road_blocked` を GUI 送信より優先するか。|

## ROS インタフェース
### Publisher
| トピック | 型 | 用途 |
|----------|----|------|
| `/manual_start` | `std_msgs/msg/Bool` | 手動開始フラグ（Transient Local）。|
| `/sig_recog` | `std_msgs/msg/Int32` | 信号認識結果 1=GO / 2=STOP。|
| `/obstacle_avoidance_hint` | `route_msgs/msg/ObstacleAvoidanceHint` | GUI 指定の固定回避指示。停止時はゼロ値を送信。|
| `/road_blocked` | `std_msgs/msg/Bool` | 道路封鎖通知（外部入力と競合した場合は外部を優先）。|

### Subscription（抜粋）
| トピック | 型 | 表示場所 |
|----------|----|----------|
| `/route_state` / `/manager_status` | `route_msgs/msg/RouteState` / `route_msgs/msg/ManagerStatus` | ルート進捗カード、再計画履歴。|
| `/follower_state` | `route_msgs/msg/FollowerState` | フォロワ状態カード、イベントログ。|
| `/active_route` | `route_msgs/msg/Route` | ルート地図画像とウェイポイント一覧。|
| `/sensor_viewer` | `sensor_msgs/msg/Image` | 障害物ビュー画像パネル。|
| `/usb_cam/image_raw` / `/sig_det_imgs` | `sensor_msgs/msg/Image` | 走行カメラ・信号監視パネル。|
| `/active_target` / `/amcl_pose` | `geometry_msgs/msg/PoseStamped` / `PoseWithCovarianceStamped` | 目標距離計算。|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度カード。|
| `/manual_start` / `/sig_recog` / `/road_blocked` | `std_msgs/msg/Bool` / `Int32` / `Bool` | イベントバナー、タブ表示の現在値。|

## ノード起動管理
- プロファイル定義は `config/node_launch_profiles.yaml`（必要に応じて作成）にまとめられ、起動対象・既定パラメータ・シミュレータ有無を記述します。
- GUI で選択した YAML は `ros2 launch <package> <file> param_file:=<path>` として渡され、`NodeLaunchManager` が `SIGINT → SIGTERM → SIGKILL` の順に安全に停止処理を行います。
- ログはノード単位でリングバッファへ収集され、Console Logs タブから確認できます。

## 運用上のヒント
- `road_blocked` の値は外部ノードからの購読値が優先されるため、GUI で送信後に値が戻る場合は外部ノードが上書きしています。必要に応じて `topics.road_blocked.external_priority=false` に変更してください。
- 障害物ヒントの固定送出は 0.5Hz で継続送信します。現場復帰時は「送出停止」ボタンを押してゼロ値を送信し、`obstacle_monitor` に制御を戻してください。
- 「全起動」は `launch_priority` の昇順で処理し、途中で失敗した場合は残りのノードを停止状態で維持します。ログタブでエラーメッセージを確認のうえ再試行してください。

## 開発・テスト
- GUI なしでロジックを確認したい場合は `python3 -m robot_console.gui_core` でユニットテスト用メインを実行できます（PyYAML / Pillow / OpenCV が未導入でもフォールバック動作）。
- モック画面は `python3 tools/mock_ui.py` で起動し、ROS 環境なしに画面レイアウトと操作フローを確認できます。
- `tools/tests/` 配下に pytest ベースのテストを収録しています。`pytest tools/tests` を実行してロジックの回帰を検出してください。

## 依存パッケージ
- GUI 機能：`tkinter`（標準ライブラリ）、`Pillow`（画像描画）、`opencv-python`（画像デコード）。Pillow / OpenCV は未導入でも縮退動作します。
- ROS2 メッセージ：`route_msgs`、`geometry_msgs`、`sensor_msgs`、`std_msgs`。

以上の内容を参考に、運用開始前に `config/node_launch_profiles.yaml` や `config/robot_console.yaml`（必要に応じて作成）を実際の環境に合わせて整備してください。
