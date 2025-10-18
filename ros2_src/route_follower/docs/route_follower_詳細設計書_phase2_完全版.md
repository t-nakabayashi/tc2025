# route_follower_詳細設計書（Phase2 最終版・完全統合版）

## 0. 文書目的
本書は route_follower ノード（Phase2）における詳細設計内容を示す。
Phase1 からの変更点として滞留検知・障害物回避（L字2段階）・状態骨格構成を実装し、
obstacle_monitor、route_manager との連携仕様を含めた完全設計情報を提供する。

---

## 1. ノード概要

| 項目 | 内容 |
|------|------|
| ノード名 | route_follower |
| 対象ROS2ディストリビューション | Foxy |
| 主機能 | 経路追従、滞留検知、障害物回避、停止制御 |
| 開発目的 | 経路追従中の一時障害物に対する局所的な自律回避 |
| 想定利用 | つくばチャレンジ等の自律走行実験 |
| 実装言語 | Python3（Google Python Style + 型ヒント） |

---

## 2. 責務とスコープ

本ノードは route_manager により供給される経路情報 `/active_route` に従い、
ロボットを逐次目標Poseへ誘導する。  
障害物監視ノード（obstacle_monitor）からの Hint 情報に基づき、滞留発生時に回避行動を行う。  
再経路生成の判断・適用は route_manager の責務とする。

---

## 3. 入出力インタフェース

### 3.1 購読トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| /active_route | route_msgs/Route | 経路情報（Waypoint配列、TRANSIENT_LOCAL） |
| /amcl_pose | geometry_msgs/PoseStamped | 現在推定位置（map座標系） |
| /obstacle_avoidance_hint | route_msgs/ObstacleAvoidanceHint | 回避ヒント情報（front_blocked / 左右開放度[m]） |
| /manual_start | std_msgs/Bool | 手動再開信号（Trueで解除） |
| /sig_recog | std_msgs/Int32 | 信号認識結果（1=GO, 2=NOGO, 他=未定義） |

### 3.2 公開トピック

| トピック名 | 型 | 内容 |
|-------------|----|------|
| /active_target | geometry_msgs/PoseStamped | 現在の目標Pose（1Hzで再送） |
| /follower_state | route_msgs/FollowerState | 状態情報（Phase2拡張版） |

### 3.3 サービス

| 名称 | 型 | 内容 |
|------|----|------|
| /report_stuck | route_srvs/ReportStuck | 滞留／回避失敗時の報告、再経路指示待ち |

---

## 4. パラメータ定義

| 名称 | 型 | 既定値 | 説明 |
|------|----|---------|------|
| arrival_threshold | float | 0.6 | waypoint到達判定距離[m] |
| control_rate_hz | float | 20.0 | 制御周期[Hz] |
| stagnation_duration_sec | float | 15.0 | 滞留成立までの継続時間[s] |
| window_sec | float | 2.0 | 滞留検知窓幅[s] |
| progress_epsilon_m | float | 0.1 | 進捗距離閾値[m] |
| min_speed_mps | float | 0.05 | 速度閾値[m/s] |
| avoid_min_offset_m | float | 0.35 | 最小回避オフセット[m] |
| avoid_max_offset_m | float | 5.0 | 最大回避オフセット（全体上限）[m] |
| avoid_forward_clearance_m | float | 0.5 | L字前進距離[m] |
| max_avoidance_attempts_per_wp | int | 2 | waypointごとの最大回避試行回数 |
| reroute_timeout_sec | float | 30.0 | 再経路待機タイムアウト[s] |
| avoid_stagnation_grace_sec | float | 2.0 | サブゴール切替後の滞留猶予[s] |
| hint_cache_window_sec | float | 5.0 | Hint評価期間[s] |
| hint_majority_true_ratio | float | 0.8 | front_blocked多数決判定比率 |
| hint_min_samples | int | 5 | Hintサンプル最少数 |

---

## 5. データ構造

### 5.1 Waypoint.msg

| フィールド | 型 | 説明 |
|-------------|----|------|
| label | string | Waypoint名 |
| pose | geometry_msgs/Pose | 位置・姿勢 |
| line_stop | bool | 停止ラインフラグ |
| signal_stop | bool | 信号停止フラグ |
| left_open | float | 左回避許容量[m] |
| right_open | float | 右回避許容量[m] |

### 5.2 FollowerState.msg（Phase2拡張）

| フィールド | 型 | 説明 |
|-------------|----|------|
| route_version | int32 | 経路バージョン |
| state | string | 現在状態 |
| current_index | int32 | 現在Waypointインデックス |
| current_pose | geometry_msgs/Pose | 現在位置 |
| distance_to_target | float | 現在地〜目標距離[m] |
| avoidance_attempt_count | int32 | 回避試行回数 |
| last_stagnation_reason | string | 最後の滞留理由 |
| front_blocked_majority | bool | front_blocked多数決結果 |
| hint_left_open_m_median | float | Hint左中央値[m] |
| hint_right_open_m_median | float | Hint右中央値[m] |

### 5.3 ReportStuck.srv

| Req/Res | フィールド | 型 | 説明 |
|----------|-------------|----|------|
| Req | route_version | int32 | 経路バージョン |
| Req | current_index | int32 | 現在Waypointインデックス |
| Req | current_wp_label | string | 現在のWaypointラベル |
| Req | current_pose_map | geometry_msgs/Pose | 現在地(map基準) |
| Req | reason | string | 滞留・回避失敗理由（"no_hint" 等） |
| Req | avoid_trial_count | uint32 | 当該Waypointでの回避試行回数 |
| Req | last_hint_blocked | bool | 直近Hintが閉塞を示したか |
| Req | last_applied_offset_m | float32 | 直前に適用した横オフセット[m] |
| Res | decision | uint8 | 1=replan, 2=skip, 3=failed |
| Res | waiting_deadline | Duration | WAITING_REROUTE継続上限 |
| Res | offset_hint | float32 | plannerへ提示する左右オフセット提案 |
| Res | note | string | 任意メモ |

---

## 6. 状態遷移仕様

### 6.1 状態一覧

| 状態 | 概要 |
|------|------|
| IDLE | 経路待機 |
| RUNNING | 経路追従中 |
| WAITING_STOP | 停止ライン・信号待ち |
| STAGNATION_DETECTED | 滞留検知後の一時状態 |
| AVOIDING | L字回避サブゴール中 |
| WAITING_REROUTE | report_stuck応答待ち |
| FINISHED | 経路完了 |
| ERROR | 異常終了 |

### 6.2 状態遷移図（概要）

```
IDLE → RUNNING → WAITING_STOP → RUNNING/FINISHED
        │
        ├─滞留検知→STAGNATION_DETECTED→AVOIDING
        │       ├─再滞留→report_stuck(avoidance_failed)
        │       └─空間無/HINT無→report_stuck(unknown_stuck)
        ├─report_stuck応答→WAITING_REROUTE
        │       ├─新route受信→RUNNING
        │       └─timeout30s→ERROR
```

---

## 7. 主処理仕様

### 7.1 滞留検知アルゴリズム
1. 直近2秒間で移動距離 < 0.1m かつ平均速度 < 0.05m/s。  
2. 上記状態が15秒間継続で滞留成立。  
3. STOPフラグ中は除外。  
4. 回避サブゴール切替後2秒間は滞留カウントを無視。

### 7.2 回避動作
1. Hintキャッシュ評価により左右開放度中央値を取得。  
2. waypoint上限と比較し、小さい方を採用。  
3. offset_min=0.35m〜offset_max=5.0mに制限。  
4. L字回避を2段階サブゴール（横→前進）として `_avoid_queue` に登録。  
5. 各サブゴール到達判定で次段発行。  
6. いずれかで再滞留検出→avoidance_failed報告。  
7. 完了後RUNNINGへ復帰。

### 7.3 STOP制御
- line_stop 到達→WAITING_STOP、解除は `/manual_start=True`。  
- signal_stop 到達→WAITING_STOP、解除は `/manual_start=True` または `/sig_recog==1(GO)`。

### 7.4 /report_stuck 処理
- サービスready確認→同期呼び出し。  
- 応答decision=replan/skip→WAITING_REROUTE、failed→ERROR。  
- timeout発生→WAITING_REROUTE + timeout監視。

### 7.5 WARN出力
- Pose未受信, route未設定, waypoint範囲外は1秒間隔でWARN。  
- timeoutやframe不一致も同様にWARN出力。

---

## 8. クラス構成（主メソッド一覧）

| 種別 | メソッド | 概要 |
|------|-----------|------|
| Subscriber | _on_route | 経路受信処理 |
| 〃 | _on_pose | 現在位置更新 |
| 〃 | _on_hint | Hintキャッシュ更新 |
| 〃 | _on_manual_start | 手動再開フラグ更新 |
| 〃 | _on_sig_recog | 信号認識更新 |
| Timer | _on_timer | 状態骨格管理・メインループ |
| State handler | _on_idle | 経路待機処理 |
| 〃 | _on_running | 追従＋滞留監視 |
| 〃 | _on_waiting_stop | 停止解除処理 |
| 〃 | _on_avoiding | L字サブゴール管理 |
| 〃 | _on_waiting_reroute | 再経路待機処理 |
| Utility | _detect_stagnation | 滞留検知 |
| 〃 | _handle_stagnation | 回避開始処理 |
| 〃 | _make_lshape_goals | L字サブゴール生成 |
| 〃 | _publish_target_pose | /active_target送信 |
| 〃 | _resend_target_if_needed | 目標再送 |
| 〃 | _publish_state | /follower_state送信 |
| 〃 | _change_state | 状態更新＋INFO出力 |

---

## 9. QoS設計

| トピック | 信頼性 | 永続性 | 履歴 | 深度 |
|-----------|----------|----------|--------|------|
| /active_route | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 |
| /amcl_pose | RELIABLE | VOLATILE | KEEP_LAST | 10 |
| /obstacle_avoidance_hint | BEST_EFFORT | VOLATILE | KEEP_LAST | 5 |
| /active_target | RELIABLE | VOLATILE | KEEP_LAST | 10 |
| /follower_state | RELIABLE | VOLATILE | KEEP_LAST | 10 |

---

## 10. 例外・エラー処理方針

| 状況 | 処理 | 状態遷移 |
|------|------|----------|
| route frame 不一致 | エラーログ出力 | ERROR |
| /report_stuck 応答timeout | WARN出力・timeout監視 | WAITING_REROUTE |
| Pose未受信 | WARN出力・処理スキップ | 維持 |
| index範囲外 | WARN出力・処理中断 | 維持 |

---

## 11. Phase3 拡張前提

- L字回避失敗時の反対側リトライ。  
- report_stuckの拡張decision（replan/skip/failed/retry）。  
- 状態遷移追加（RETRYINGなど）。

---

## 12. テスト観点

| テスト種別 | 目的 | 検証項目 |
|-------------|------|----------|
| ユニットテスト | 内部関数の精度確認 | 滞留検知・L字生成式・ヒント評価 |
| コンポーネントテスト | ノード単体起動確認 | 各トピック入出力・状態遷移 |
| 結合テスト | route_manager連携確認 | report_stuck応答と再経路遷移 |
| シナリオテスト | 実環境検証 | 障害物回避挙動・停止信号動作 |

---

## 13. 付録：状態遷移図（概念）

```
          +-----------+
          |   IDLE    |
          +-----------+
                |
                v
          +-----------+
          |  RUNNING  |<-------------+
          +-----------+              |
          |滞留検知 ↓                |
    +---------------------+          |
    | STAGNATION_DETECTED |          |
    +---------------------+          |
                |回避成功             |
                v                    |
           +-----------+             |
           | AVOIDING  |             |
           +-----------+             |
                |再滞留               |
                v                    |
         +----------------+          |
         | report_stuck   |          |
         +----------------+          |
                | replan/skip        |
                v                    |
         +----------------+          |
         |WAITING_REROUTE |----------+
                |
          timeout→ERROR
```

---

## 14. 結論
本詳細設計に基づき、`route_follower_phase2_final.py` を実装した。
Phase2では堅牢な滞留検知およびL字回避機構を実装し、状態骨格構造により
コードの可読性・拡張性を大幅に向上させた。

以上。
