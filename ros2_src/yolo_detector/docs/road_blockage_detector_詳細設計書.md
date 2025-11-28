# 日本語文書
# road_blockage_detector 詳細設計書

## 1. 目的とスコープ
road_blockage_detector ノードは YOLO 推論結果と自己位置を入力として、経路封鎖看板の有無と封鎖位置を判定する。判定結果は robot_navigator を経由してロボット動作（停止／再開）へ反映される。本設計書ではノードの責務、I/F、内部処理、保持データ構造、異常時動作を定義する。

## 2. 背景と前提条件
- YOLO 推論結果は yolo_detector パッケージが `vision_msgs/msg/Detection2DArray` として publish 済みである。
- ノードは `/amcl_pose` を唯一の自己位置情報として使用し、TF2 には依存しない。
- 画像オーバーレイ描画と `/yolo_detector/image_raw` publish 要件は削除済みのため、本ノードでは行わない。
- 過去の経路封鎖位置はノード内メモリで累積保持すればよい。永続化要件は無し。

## 3. ロボット挙動との連携と全体フロー
- YOLO で経路封鎖看板を検出し、`road_blocked` を true で publish した時点で robot_navigator が走行を一時停止する。停止中も本ノードは検知を継続する。
- 検知開始から `confirmation_duration` 秒経過前に封鎖検知が途切れた場合、`road_blocked` を false で publish し、robot_navigator が走行を再開する（誤検知扱い）。
- 検知が `confirmation_duration` を超えて継続した場合は封鎖を確定し、封鎖位置を `blocked_positions` に追加する。`road_blocked` は true のまま維持し、ロボットは停止を継続する。
- 封鎖確定後、`route_follower` 側で `stagnation_duration_sec` が経過すると滞留と判定され、`report_stuck` を `route_manager` に要求する。`route_manager` はリルートを実施し、封鎖を回避した経路で走行を再開する。
- 検知が取り消され `road_blocked=false` を publish した場合、robot_navigator は hold 時間経過後に解除候補として取り込むため、誤検知時に自動で走行再開できる。

## 4. ノードの役割・責務
| 区分 | 内容 |
| ---- | ---- |
| 主要役割 | YOLO 検知から経路封鎖看板候補を抽出し、時系列の判定カウントを管理して仮／確定封鎖を判断する。 |
| 責務1 | 検知パラメータ（クラス ID・スコア・バウンディングボックス閾値）に基づくフィルタリング処理を実装する。 |
| 責務2 | 判定期間内のカウント履歴から封鎖確率を評価し、`road_blocked` Bool を publish する。 |
| 責務3 | 確定封鎖とみなした自己位置を履歴として保持し、再侵入時の多重検知抑止を行う。 |
| 責務4 | `/amcl_pose` が取得できない場合や Detection と `/amcl_pose` の時刻差が大きい場合に警告ログを出力する。 |

## 5. 外部 I/F
### 5.1 サブスクライブ
| トピック | 型 | QoS | 用途 |
| -------- | -- | --- | ---- |
| `/yolo_detector/detections` | `vision_msgs/msg/Detection2DArray` | SensorData | YOLO 推論結果受信。 |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Default | 最新の自己位置キャッシュ。Detection 時刻との差も確認する。 |

### 5.2 Publish
| トピック | 型 | QoS | 条件 |
| -------- | -- | --- | ---- |
| `/road_blocked` | `std_msgs/msg/Bool` | Default | 仮判定状態遷移時（false→true、true→false）にのみ publish し、robot_navigator へ通知する。 |

### 5.3 自己位置取得
- `/amcl_pose` を subscribe し、メッセージヘッダーの時刻を含めてキャッシュする。
- Detection2DArray のヘッダー時刻と最新の `/amcl_pose` ヘッダー時刻に 3 秒以上の差がある場合は警告を出すが、処理自体は続行する。
- 最新 `/amcl_pose` が未取得の場合は警告を出して検知処理をスキップする。

## 6. パラメータ仕様
| 名称 | 型 | 既定値 | 説明 |
| ---- | -- | ------ | ---- |
| `target_class_id` | int | 0 | 経路封鎖看板に対応するクラス ID。 |
| `score_threshold` | float | 0.7 | 最小スコア。未満の検知は除外。 |
| `bbox_width_min` | float | -1 | 幅閾値下限 [pixel]。負値なら判定スキップ。 |
| `bbox_width_max` | float | -1 | 幅閾値上限 [pixel]。負値なら判定スキップ。 |
| `bbox_height_min` | float | -1 | 高さ閾値下限 [pixel]。負値なら判定スキップ。 |
| `bbox_height_max` | float | -1 | 高さ閾値上限 [pixel]。負値なら判定スキップ。 |
| `bbox_bottom_max` | float | -1 | バウンディングボックス下端の最大位置 [pixel]。画像下端からの距離で定義し、負値なら判定スキップ。 |
| `decision_duration` | float | 2.0 | 判定期間長 [秒]。この期間のカウント履歴を保持。 |
| `decision_frame_ratio` | float | 50.0 | 判定期間内で「カウント>=1」の秒バケット割合 [%] がこの値以上で仮判定成立。 |
| `confirmation_duration` | float | 10.0 | road_blocked=true 継続時間が本値を超えたら封鎖確定（ノード内固定。YAMLでは変更不可）。 |
| `multi_detection_suppression_range` | float | 10.0 | 過去封鎖位置中心から本距離[m]以内に現在位置が入った場合はカウント0としてスキップ。 |

## 7. 内部データ構造
| 名称 | 形式 | 内容 |
| ---- | ---- | ---- |
| `count_history` | `collections.deque[tuple[float, int]]` | 「秒単位バケットの開始時刻（float, ROS time 秒）」と、その1秒間に記録した判定カウントの合計を保持。`decision_duration` 秒を超えた古いバケットは随時削除する。検知周期に依存せず秒単位のスライディングウィンドウを構成する。 |
| `last_detection_time` | `builtin_interfaces/msg/Time` | 最後に `detections` を処理した時刻。間引きや欠損検出に利用。 |
| `temporary_decision_count` | int | 仮判定カウント。秒バケット割合に応じて加算／リセット。 |
| `blocked_positions` | `list[geometry_msgs.msg.Pose]` | 確定封鎖と判断した際の `map` 座標（`/amcl_pose` 基準）。多重検知抑止に使用。 |
| `latest_amcl_pose` | `Pose` | `/amcl_pose` からの最新値キャッシュ。 |
| `latest_amcl_time` | `rclpy.time.Time` | `/amcl_pose` メッセージヘッダーの時刻。Detection のヘッダー時刻との乖離チェックに利用。 |
| `blocked_state_started_at` | float | road_blocked=true に遷移した ROS 時刻 (秒)。経過時間で確定判定。 |

## 8. 処理フロー
1. **起動処理**
   - パラメータ宣言・取得。
   - サブスクライバ／パブリッシャを生成。
   - 判定期間から秒単位バケットの上限幅を決定し、deque を初期化。
   - ロガーで起動を通知。

2. **検知メッセージ受信 (`Detection2DArray` コールバック)**
   1. 最新の `/amcl_pose` を取得できていない場合は警告を出して処理を終了。取得済みの場合はヘッダー時刻差を確認し、3 秒以上ずれていれば警告を出す。
   2. `blocked_positions` と比較し、現在位置がいずれかの確定封鎖地点から `multi_detection_suppression_range` 未満なら、`count_history` へ 0 を push し残処理をスキップ。
   3. 各 `Detection2D` について以下を実施：
      - `results` を走査し、最大スコアクラスを決定。
      - スコアが `score_threshold` 未満の場合は除外。
      - クラス ID が `target_class_id` と一致しない場合は除外。
      - バウンディングボックスの幅・高さ・下端位置を算出し、指定閾値の範囲に入らなければ除外。ただし負値指定の閾値は判定をスキップ。
   4. 条件を満たした検知数を計数し、メッセージ時刻の秒バケットに加算する。新規バケットを生成する際は `decision_duration` 秒より古いバケットを削除する。

3. **判定ロジック**
   - `count_history` の秒バケット数を分母とし、値が `>=1` のバケット割合を算出。
   - 割合が `decision_frame_ratio` 以上の場合：
     - `temporary_decision_count` を +1。
     - 直前まで 0 だった場合は road_blocked=true を publish、`blocked_state_started_at` を現在時刻でセットし、`confirmation_duration` 計測を開始する。
   - 割合が閾値未満の場合：
     - `temporary_decision_count` を 0 にリセット。
     - 直前まで >0 だった場合は road_blocked=false を publish し、封鎖計測時間をログに出したうえで `blocked_state_started_at` を None に戻す（誤検知として走行再開）。

4. **封鎖確定処理**
   - `temporary_decision_count > 0` かつ `blocked_state_started_at` が設定済みの場合に経過時間をチェック。
   - `now - blocked_state_started_at >= confirmation_duration` なら、最新 `/amcl_pose` から取得した pose を `blocked_positions` に追加し、仮判定カウントを 0 にクリア、`blocked_state_started_at` も None に戻す。road_blocked は true のまま維持し、以降は `route_follower` の滞留判定と `route_manager` のリルート処理により走行再開が行われる。

## 9. ロギング方針
| レベル | タイミング |
| ------ | ---------- |
| info | ノード起動、road_blocked true/false への遷移、封鎖確定時（位置情報含む）。 |
| warn | `/amcl_pose` 未取得時、Detection と `/amcl_pose` の時刻差が 3 秒以上ある場合、検知メッセージが連続で欠損した場合。 |
| debug | フィルタ後の検知数、割合計算結果、履歴長などの内部状態（パラメータでオンオフ可）。 |

## 10. エラー／例外ハンドリング
- `Detection2DArray` に要素が無い場合でも `count_history` へ 0 を push し、割合計算を継続する。
- `/amcl_pose` が未取得の場合は処理をスキップし、警告ログを出力する。回復後は通常処理へ復帰する。
- Detection のヘッダー時刻と `/amcl_pose` のヘッダー時刻の差が大きい場合は警告のみを出し、処理は継続する。

## 11. 今後の検討事項
- road_blocked publish の QoS（信頼性・一回送信）を要件に応じて調整する。
- `blocked_positions` の上限や経過時間による自動削除の必要性を実走テストで検討する。
