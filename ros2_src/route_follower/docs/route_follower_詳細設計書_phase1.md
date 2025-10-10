# route_follower 詳細設計書 (phase1-正式版)

## 1. 概要

本ノードは、route_manager から受信した `/active_route` に含まれる Waypoints を順次追従し、
下位ノードが参照する `/active_target` (PoseStamped) を一定周期で発行する ROS2 ノードである。

本書は、**phase1 の正式仕様および実装設計**を記載するものであり、
他の開発者が本ドキュメントをもとに **同一機能を再実装可能な粒度**で記述する。

phase1 現在の実装範囲では、以下の機能を実装済みである。

- 状態機械の導入（IDLE / RUNNING / WAITING_STOP / FINISHED / ERROR）
- `line_stop` 到達時の WAITING_STOP 遷移
- `/manual_start` による再開制御（True 受信で RUNNING に復帰）
- `/follower_state` による状態通知（100ms デバウンス周期発行）
- `/active_target` の 1Hz 再送
- `/active_route` の frame_id 厳格チェック
- 全ての状態遷移はタイマーコールバック内で完結

signal_stop や rerouting 遷移は、次フェーズ以降に実装予定である。

---
## 2. 責務とスコープ

### 2.1 責務
`route_follower` ノードは、`route_manager` から配信される `/active_route` に基づき、経路追従を行う。
経路上の各ウェイポイントを順次追従し、進行対象を `/active_target` トピックとして下位ノードへ供給する。
また、停止属性（line_stop）や手動再開指令（/manual_start）を処理し、走行状態を `/follower_state` で上位へ通知する。

### 2.2 スコープ（phase1）
- 経路追従処理
- line_stop属性による停止制御
- /manual_start による再開制御
- /active_target および /follower_state の発行
- frame_id 整合チェック（空文字時の補完を含む）

### 2.3 非スコープ（phase2以降で対応）
- rerouting（経路再生成）
- signal_stop対応
- 障害物回避やサブゴール生成

### 2.4 他ノードとの関係
| 入出力 | トピック名 | 型 | 説明 |
|--------|-------------|----|------|
| 入力 | /active_route | custom_msgs/ActiveRoute | 経路情報の受信 |
| 入力 | /amcl_pose | geometry_msgs/PoseWithCovarianceStamped | 現在位置 |
| 入力 | /manual_start | std_msgs/Bool | 停止後の再開指令 |
| 出力 | /active_target | geometry_msgs/PoseStamped | 現在の追従目標 |
| 出力 | /follower_state | custom_msgs/FollowerState | 走行状態通知 |

---

## 3. フェーズ構成

### phase1（本書で対象とするフェーズ）
- 経路追従、停止属性(line_stop)、手動再開(/manual_start)
- エラー検証、状態通知
- rerouting 状態は定義のみ（遷移未実装）

### phase2（次フェーズ予定）
- rerouting の実装（新ルートへの動的切替、最近傍WP再マップ）
- signal_stop 対応

### phase3（将来フェーズ予定）
- 障害物検知によるサブゴール設定および回避行動

---

## 4. 外部I/F仕様

### 購読トピック

| トピック名 | 型 | 説明 | QoS |
|-------------|----|------|-----|
| `/active_route` | `route_msgs/Route` | 経路入力。frame_id が target_frame と異なる場合は ERROR に遷移。 | RELIABLE / TRANSIENT_LOCAL |
| `/amcl_pose` | `geometry_msgs/PoseStamped` | 現在姿勢を購読し到達判定に使用。 | RELIABLE / VOLATILE |
| `/manual_start` | `std_msgs/Bool` | True 受信時に WAITING_STOP を解除し RUNNING に復帰。 | RELIABLE / VOLATILE |

### 発行トピック

| トピック名 | 型 | 説明 | 発行条件 | QoS |
|-------------|----|------|-----------|-----|
| `/active_target` | `geometry_msgs/PoseStamped` | 現在の追従ターゲット。一定周期(1Hz)で再送。 | RUNNING状態時または初期送信時 | RELIABLE / TRANSIENT_LOCAL |
| `/follower_state` | `route_msgs/FollowerState` | 現在状態、距離、waypointラベルを発行。100ms デバウンスで周期発行。 | 常時 | RELIABLE / VOLATILE |

---

## 5. 状態機械

### 状態一覧

| 状態 | 説明 |
|------|------|
| **IDLE** | 経路受信後、手動開始待ちの静止状態。 |
| **RUNNING** | 通常走行中。到達判定を実施。 |
| **WAITING_STOP** | line_stop 到達により一時停止中。/manual_start=True により RUNNING に復帰。 |
| **REROUTING** | 未実装。将来フェーズで動的経路再生成時に使用予定。 |
| **FINISHED** | 経路最終WPに到達し走行終了。 |
| **ERROR** | frame_id 不一致などの異常検出により停止。 |

### 状態遷移条件（全てタイマー内で処理）

| 現在状態 | 条件 | 次状態 |
|-----------|-------|--------|
| IDLE | start_immediately=True で新経路受信 | RUNNING |
| RUNNING | line_stop 到達 | WAITING_STOP |
| WAITING_STOP | `/manual_start=True` 受信 | RUNNING |
| RUNNING | 最終WP到達 | FINISHED |
| 任意 | `/active_route.header.frame_id` 不一致 | ERROR |

---

## 6. 動作シーケンス

1. `/active_route` 受信  
   - frame_id を検証。異なる場合は ERROR 遷移。  
   - `start_immediately=True` の場合、RUNNING に遷移し最初のWPを `/active_target` に発行。  

2. RUNNING 状態  
   - 現在位置との距離が arrival_threshold 未満の場合、次WPに遷移。  
   - 次WPの `line_stop=True` の場合、WAITING_STOP に遷移。  

3. WAITING_STOP 状態  
   - `/manual_start=True` を受信すると RUNNING に復帰し次WPを `/active_target` に発行。  
   - False は無視。自動復帰は行わない。  

4. FINISHED 状態  
   - 経路最終WPに到達後、停止。状態を `/follower_state` で通知。  

5. ERROR 状態  
   - `/active_route` frame_id 不一致時などに遷移。  
   - 復帰にはノード再起動または新経路受信が必要。  

---

## 7. エラー処理

| 条件 | 動作 |
|------|------|
| `/active_route.header.frame_id` が target_frame と異なる | ERROR に遷移。経路は適用しない。 |
| 経路が空 | 警告ログ出力。状態は維持。 |
| current_pose 未受信 | ターゲットを再送して待機。 |
| 未定義状態 | ERROR にフォールバック。 |

---

## 8. パラメータ

| 名称 | 型 | 既定値 | 説明 |
|------|----|--------|------|
| `arrival_threshold` | double | 0.6 | Waypoint到達判定の距離閾値[m] |
| `control_rate_hz` | double | 20.0 | 制御周期[Hz] |
| `republish_target_hz` | double | 1.0 | `/active_target` の再送周期[Hz] |
| `target_frame` | string | "map" | 座標系frame |
| `start_immediately` | bool | True | 経路受信後すぐに走行開始するか |
| `state_debounce_ms` | int | 100 | `/follower_state` のデバウンス間隔[ms] |

---

## 9. QoS設定

| トピック | reliability | durability | depth |
|-----------|--------------|-------------|--------|
| `/active_route` | RELIABLE | TRANSIENT_LOCAL | 1 |
| `/amcl_pose` | RELIABLE | VOLATILE | 10 |
| `/manual_start` | RELIABLE | VOLATILE | 10 |
| `/active_target` | RELIABLE | TRANSIENT_LOCAL | 1 |
| `/follower_state` | RELIABLE | VOLATILE | 10 |

---

## 10. 内部構成および処理概要

### 主なクラス

`RouteFollower(Node)`  
- タイマー周期で状態更新および遷移処理を行う。  
- 外部イベント（/active_route, /manual_start）はコールバックで受け、フラグにラッチし、タイマー内で適用する。  
- これにより状態遷移を一元管理し、将来的な障害物検知やrerouting処理の追加を容易にする。  

### 処理の流れ

```
_on_route()      → 経路を受信し保留
_on_manual_start() → True受信時にフラグセット
_on_timer()      → フラグ・状態・到達判定を統合処理
_publish_target() → PoseStampedを生成し送信
_publish_state()  → follower_stateを100ms周期で通知
_reached()        → 2D距離で到達判定
```

---

## 11. 将来拡張方針

- rerouting 状態を有効化し、新ルート受信時に最近傍WPを再マップして再開可能にする。  
- signal_stop 属性の実装を追加し、信号検知時の停止を制御する。  
- 障害物検知ノード (obstacle_monitor) からの入力により、サブゴールを一時設定し、AVOIDING_OBSTACLE 状態を導入する。  

---
## 12. 外部I/F仕様

### `/active_route`（入力）
- 経路情報を受信し、内部経路を更新。
- frame_id が空文字列の場合は、自動的に target_frame（既定値 "map"）を補う。
- frame_id が異なる場合は ERROR 状態へ遷移する。

### `/active_target`（出力）
- RUNNING 状態で未到達のとき、または current_pose 未受信時に保険的に再送する。
- 再送周期は `republish_target_hz` パラメータ（既定値 1.0Hz）に基づく。
- QoS: RELIABLE / TRANSIENT_LOCAL

### `/follower_state`（出力）
- 代表的なフィールド例：
  ```
  route_version: int
  state: str
  current_index: int
  current_waypoint_label: str
  next_waypoint_label: str
  distance_to_target: float
  current_pose: geometry_msgs/PoseStamped
  ```
- これらを100msデバウンス付きで送信。QoS: RELIABLE / VOLATILE

---

## 13. 保守・拡張上の留意事項

- 状態遷移はタイマーコールバック内に閉じており、スレッド競合を起こさない構造。  
- /manual_start, rerouting, obstacle などの外部入力イベントはフラグとして一時保持し、タイマーで統一処理する。  
- rerouting および signal_stop は phase2 以降で追加可能な構成を保持している。  
- QoS 設定は全ノード間で統一すること（特に TRANSIENT_LOCAL の整合性に注意）。  

---

## 14. まとめ

本詳細設計書は、route_follower ノードの phase1 完了版（正式仕様）を示す。  
本書の記載内容のみで、他の開発者が同機能を再実装できる粒度で情報を提供している。  
本フェーズの成果物は、phase2 以降（動的経路再生成・障害物回避）の基盤となる。
