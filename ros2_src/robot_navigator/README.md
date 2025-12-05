# robot_navigator パッケージ README (phase2正式版)

## 概要
`robot_navigator` は `/active_target`（`PoseStamped`）を追従する時間最適制御を実装し、
`/cmd_vel` を出力する移動体ナビゲーションノードです。Phase2 では障害物距離を
`ObstacleAvoidanceHint` または `LaserScan` から取得して減速・停止を行い、進行方向を
`visualization_msgs/Marker` で可視化します。試験用に `/cmd_vel` を受け取って自己位置を
配信する `robot_simulator` ノード（同パッケージ内）も併載しています。

## 主な機能
- `/odom`・`/amcl_pose`・`/active_target` を監視し、線速度・角速度の時間最適解を近似計算。
- `obstacle_distance_mode` に応じて `/scan` もしくは `/obstacle_avoidance_hint` から前方距離を取得し、
  `safety_distance`・`min_obstacle_distance` に基づき減速／停止を制御する。
- 角速度は PID（`kp=0.65`, `ki=0.001`, `kd=0.02`）で生成し、角度誤差に応じて線速度をスケール。
- `/direction_marker` に進行方向を示す矢印 Marker を Publish。
- `log_csv_path` が書き込み可能な場合、制御内部状態を CSV として逐次出力。
- 必要なトピックが揃わない場合は `/cmd_vel` にゼロを出力し、WARN ログを一定周期で発行。

## 起動方法
### launch を用いた起動
```bash
ros2 launch robot_navigator robot_navigator.launch.py \
  obstacle_hint_topic:=/obstacle_avoidance_hint cmd_vel_topic:=/cmd_vel
```
- launch 引数で入出力トピックをリマップ可能。`param_file` で任意の YAML を指定できます。
- `obstacle_distance_mode` を `scan` に設定すると `/scan` を購読し、`hint` の場合は
  `/obstacle_avoidance_hint` を使用します。

### 実行ファイルを直接起動
```bash
ros2 run robot_navigator robot_navigator
```
- 動作確認には同梱の `robot_simulator`（`ros2 run robot_navigator robot_simulator`）と組み合わせて
  `/cmd_vel` の挙動を確認できます。

## 外部インタフェース
### Subscriber
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/odom` | `nav_msgs/Odometry` | 現在速度を取得し加速度制限に利用。 | RELIABLE / VOLATILE / depth=10 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 現在姿勢（位置・ヨー角）を取得。 | RELIABLE / VOLATILE / depth=10 |
| `/active_target` | `geometry_msgs/PoseStamped` | 追従対象の目標姿勢。 | RELIABLE / VOLATILE / depth=10 |
| `/obstacle_avoidance_hint` | `route_msgs/ObstacleAvoidanceHint` | `obstacle_distance_mode=hint` のとき使用。 | BEST_EFFORT / VOLATILE / depth=1 |
| `/scan` | `sensor_msgs/LaserScan` | `obstacle_distance_mode=scan` のとき使用（SensorDataQoS）。 | BEST_EFFORT / VOLATILE / depth=1 |
| `/amcl_glitch_trigger` | `std_msgs/Bool` | `robot_simulator` が停止中に受信すると `/amcl_pose` へ単発オフセットを加算。 | RELIABLE / VOLATILE / depth=10 |

### Publisher
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/cmd_vel` | `geometry_msgs/Twist` | 車体指令速度。 | RELIABLE / VOLATILE / depth=10 |
| `/direction_marker` | `visualization_msgs/Marker` | 進行方向矢印。`marker_frame` で指定したフレームに出力。 | RELIABLE / VOLATILE / depth=1 |

> サービス・アクションは提供しません。

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `max_vel` | double | `1.1` | 線速度の上限 [m/s]。
| `max_w` | double | `1.8` | 角速度の上限 [rad/s]。
| `max_acc_v` | double | `1.0` | 線加速度上限 [m/s^2]。
| `max_acc_w` | double | `1.5` | 角加速度上限 [rad/s^2]。
| `pos_tol` | double | `0.5` | 位置許容誤差 [m]。近接時に角度誤差処理へ切替。
| `ang_tol` | double | `0.25` | 角度許容誤差 [rad]。
| `control_rate_hz` | double | `20.0` | 制御ループ周期 [Hz]。
| `robot_width` | double | `0.6` | 障害物距離評価に用いるロボット幅 [m]。
| `safety_distance` | double | `0.8` | 減速を開始する距離 [m]。
| `min_obstacle_distance` | double | `0.5` | 停止を指示する距離 [m]。
| `obst_max_dist` | double | `5.0` | 障害物距離として採用する最大値 [m]。
| `obstacle_distance_mode` | string | `"hint"` | `hint` または `scan`。距離取得ソースを切替。
| `marker_frame` | string | `"map"` | Marker の出力フレーム。
| `log_csv_path` | string | `~/control_log.csv` | CSV ログ出力先パス。

## 状態管理・処理フロー
1. `/odom`・`/amcl_pose`・`/active_target` の受信状況を監視し、欠損時は `/cmd_vel` にゼロを出力して
   WARN を 5 秒周期で報告する。
2. 入力が揃うと `compute_time_optimal_cmd_vel()` を呼び出し、角度誤差の PID 制御で角速度を算出。
3. 線速度は角度誤差および障害物距離に基づくスケールを適用し、`max_acc_v` に従って加速度を制限。
4. `/obstacle_avoidance_hint` または `/scan` から得た前方距離が `min_obstacle_distance` 未満なら停止し、
   `safety_distance` 未満では線速度を線形減衰させる。
5. 指令を `/cmd_vel` に Publish し、`/direction_marker` で現在の進行方向を可視化する。
6. CSV ログが有効な場合は制御ループの各種値（速度、誤差、障害物距離）を逐次書き出す。

## 動作確認手順
1. `robot_simulator`（当パッケージ内）を起動し、`/cmd_vel` を受け取って `/amcl_pose`・`/odom` を配信させる。
2. `route_follower` もしくは手動で `/active_target` を Publish し、目標指令を入力する。
3. `obstacle_monitor` を起動して `/obstacle_avoidance_hint` を供給するか、`obstacle_distance_mode:=scan`
   として `/scan` を直接購読させる。
4. `robot_navigator` を launch し、`/cmd_vel`・`/direction_marker` の挙動、CSV ログの内容を確認する。

## デバッグのヒント
- CSV ログが生成されない場合は `log_csv_path` のディレクトリ権限を確認してください。
- `obstacle_distance_mode` が `scan` で距離が常に `None` となる場合は `/scan` の FOV がロボット幅帯を
  カバーしているか確認します。
- WARN ログのスロットルが頻発する場合は、入力トピックの QoS やリマップ設定を再確認してください。
- `/amcl_pose` に付与されるノイズはガウス分布（位置 `pose_noise_std_m`、ヨー角 `yaw_noise_std_deg`）を
  各 publish ごとにサンプリングして加算する実装です。標準偏差を 1.0 に設定すれば平均的には 1m 規模の
  ばらつきが常時発生しますが、「停止中にごくまれに 1m だけジャンプする」ような突発外れ値を再現する
  仕組みは現状ありません。単発のステップ状誤差を模擬する場合は、別途一時的にオフセットを加える
  ロジックの追加が必要です。

## 自己位置外れ値を模擬するための実装（外部トリガ方式）
実ロボットで報告された「停止時に突発的に 1m ほど姿勢が跳ぶ」事象を再現するため、`robot_simulator`
に外部トリガ入力で単発オフセットを付与する機能を実装しました。確率分布による自動発火は行わず、
テストスクリプトや人間操作で明示的に発火させることで再現性を確保します。

- **トリガトピック**: `/amcl_glitch_trigger`（`std_msgs/Bool`）。`data=true` を受信した瞬間に 1 回
  だけオフセットを予約し、停止状態で `glitch_wait_after_stop_sec`（既定 5 秒）の待機後に
  `/amcl_pose` へ反映します。走行中にトリガを受けた場合は「停止するまで予約」を保持し、
  停止とクールダウンを満たしたタイミングから待機を開始します。連続発火を防ぐため、
  `glitch_cooldown_sec` 経過まで後続トリガは無視します。
- **停止条件の併用**: `cmd_vel` の線速度・角速度が閾値（`glitch_linear_stop_threshold`、
  `glitch_angular_stop_threshold`）未満のときだけトリガを有効化し、走行中の意図しない外れ値挿入を
  防ぎます。
- **オフセット生成**: 既存のガウスノイズとは別に、半径 `glitch_radius_m`・方向一様の 2D オフセット
  と、ヨー角ガウスノイズ（`glitch_yaw_std_deg`）を単発で加算します。
- **共分散更新**: 外れ値付与時は共分散の下限を位置 `glitch_cov_floor_m2`、ヨー角
  `glitch_yaw_cov_floor_deg2` に引き上げ、自己位置信頼度低下を表現します。通常 publish 時は従来の
  共分散を使用します。
- **ログとデバッグ**: 受信値・適用オフセット・クールダウン残り秒数を INFO で記録し、false トリガ
  など無視した条件も DEBUG/INFO で把握できます。

## 将来拡張メモ
- default.yaml の `obst_fov_deg` や `enable_debug_pub` など未使用パラメータは Phase3 での
  ナビゲーション高度化向けに予約されています。
- `robot_simulator_node` の TF 連携と組み合わせた統合試験手順書を docs 配下に追記予定です。
