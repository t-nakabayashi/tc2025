# route_follower パッケージ README (phase1正式版)

## 概要
`route_follower` は、`route_manager` から受信した `/active_route` に含まれるウェイポイント列を順次追従し、下位ノードに `/active_target` (PoseStamped) を配信する **経路追従ノード** です。状態は `/follower_state` で通知します（状態機械：IDLE / RUNNING / WAITING_STOP / REROUTING(未使用) / FINISHED / ERROR）。  
本READMEは **phase1正式版** に対応し、詳細設計書および実装・launchの内容に準拠しています。

---

## 主な機能（phase1対応範囲）
- `line_stop` 到達で **WAITING_STOP** に遷移、`/manual_start=True` で再開  
- `/active_target` の **1Hz 再送**（保険的再送を含む）  
- `/follower_state` の **100msデバウンス**による周期通知  
- `/active_route` の **frame_id 厳格チェック**（空文字は `target_frame` を補完）  
- すべての状態遷移は **タイマーコールバック内で一元処理**

---

## 起動方法

### 1. 単独起動
```bash
ros2 launch route_follower route_follower.launch.py
```

主要な引数を指定する例：
```bash
ros2 launch route_follower route_follower.launch.py \
  arrival_threshold:=0.6 control_rate_hz:=20.0 start_immediately:=true \
  target_frame:=map \
  active_route_topic:=/active_route \
  current_pose_topic:=/amcl_pose \
  active_target_topic:=/active_target
```

> **補足:** `resend_interval_sec` という引数が launch に存在しますが、現行ノードは `republish_target_hz` パラメータで再送周期を管理しており、launch 側の `resend_interval_sec` は **未使用** です（将来リネーム予定）。既定では **1.0 Hz** で再送されます。

### 2. 起動後のトピック
```
/active_route     (subscribe)
/amcl_pose        (subscribe)
/manual_start     (subscribe)
/active_target    (publish)
/follower_state   (publish)
```

---

## 外部インタフェース一覧

### トピック購読
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/active_route` | `route_msgs/Route` | 経路入力。`header.frame_id` が `target_frame` と異なる場合は ERROR。空文字は `target_frame` を補完。 | RELIABLE / TRANSIENT_LOCAL |
| `/amcl_pose` | `geometry_msgs/PoseStamped` | 現在姿勢。到達判定に使用。 | RELIABLE / VOLATILE |
| `/manual_start` | `std_msgs/Bool` | True 受信で WAITING_STOP を解除し RUNNING に復帰。 | RELIABLE / VOLATILE |

### トピック配信
| 名称 | 型 | 説明 | 発行条件 | QoS |
|------|----|------|-----------|-----|
| `/active_target` | `geometry_msgs/PoseStamped` | 現在の追従ターゲット。| RUNNING 中は未到達なら再送、`current_pose` 未受信時も保険的再送。既定 **1.0 Hz** | RELIABLE / TRANSIENT_LOCAL |
| `/follower_state` | `route_msgs/FollowerState` | 状態・距離・ラベル等を100msデバウンスで通知。 | 常時 | RELIABLE / VOLATILE |

---

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `arrival_threshold` | double | 0.6 | Waypoint 到達判定閾値[m] |
| `control_rate_hz` | double | 20.0 | 制御周期[Hz] |
| `republish_target_hz` | double | 1.0 | `/active_target` の再送周期[Hz] |
| `target_frame` | string | "map" | 座標系 frame |
| `start_immediately` | bool | True | 経路受信後に即走行を開始するか |
| `state_debounce_ms` | int | 100 | `/follower_state` のデバウンス間隔[ms] |

> **注意:** launch 引数 `resend_interval_sec` は現行ノードでは使用されません。再送周期を変更したい場合は **ノード側パラメータ `republish_target_hz`** を変更してください。

---

## QoS設定
| トピック | reliability | durability | depth |
|-----------|--------------|-------------|--------|
| `/active_route` | RELIABLE | TRANSIENT_LOCAL | 1 |
| `/amcl_pose` | RELIABLE | VOLATILE | 10 |
| `/manual_start` | RELIABLE | VOLATILE | 10 |
| `/active_target` | RELIABLE | TRANSIENT_LOCAL | 1 |
| `/follower_state` | RELIABLE | VOLATILE | 10 |

---

## 状態遷移（要約）
- **IDLE** → `start_immediately=True` で新経路受信時に **RUNNING**  
- **RUNNING** → `line_stop` 到達で **WAITING_STOP** / 最終WP到達で **FINISHED**  
- **WAITING_STOP** → `/manual_start=True` で **RUNNING**  
- 任意状態 → `/active_route.header.frame_id` 不一致で **ERROR**

---

## remap / launch 引数

### remap（launch 内既定）
- `/active_route` ← `active_route_topic`  
- `/amcl_pose` ← `current_pose_topic`  
- `/active_target` ← `active_target_topic`

### launch 引数
| 引数名 | 既定値 | 説明 |
|--------|--------|------|
| `arrival_threshold` | 0.6 | 到達判定閾値[m] |
| `control_rate_hz` | 20.0 | 制御周期[Hz] |
| `resend_interval_sec` | 1.0 | **未使用（将来、republish_target_hzに統合予定）** |
| `start_immediately` | true | 経路受信直後に開始 |
| `target_frame` | map | 全Poseの frame_id |
| `node_name` | route_follower | ノード名 |
| `active_route_topic` | /active_route | 経路入力トピック |
| `current_pose_topic` | /amcl_pose | 現在姿勢トピック |
| `active_target_topic` | /active_target | 目標Pose出力トピック |

---

## 動作確認方法

1) **ノード起動**
```bash
ros2 launch route_follower route_follower.launch.py
```

2) **経路配信（route_manager 側）**  
`/active_route` を Publish し、follower が `RUNNING` へ遷移することを確認。

3) **停止／再開の確認**
```bash
# line_stop付きのWPへ到達 → WAITING_STOPへ遷移
# 再開:
ros2 topic pub /manual_start std_msgs/Bool "{data: true}" -1
```

4) **状態確認**
```bash
ros2 topic echo /follower_state
```

5) **frame_id 検証**  
`/active_route.header.frame_id` が `target_frame` と異なると **ERROR** に遷移。空文字は `target_frame` が補完される。

---

## 既知の注意点 / 差分
- **launchとノードの周期設定の名称が不一致**  
  - launch: `resend_interval_sec`（秒）  
  - ノード: `republish_target_hz`（Hz）  
  → 現行は **ノード側パラメータのみ有効**。将来のリファクタで統一予定。
- **frame_id の扱い**  
  - `header.frame_id` が空なら `target_frame` を自動補完。異なる場合は ERROR。

---

## 将来拡張（phase2以降）
- REROUTING 遷移の実装（新ルート受信時の最近傍WP再マップ・再開）
- `signal_stop` 対応
- 障害物検知との連携によるサブゴール設定

---

## 参考
- 詳細設計書（phase1正式版）  
- ほかノードREADMEの構成方針を踏襲しています。

