# route_manager_詳細設計書_phase2.md

## 第1章　概要

`route_manager` は、全体経路の管理および再計画を担うROS2ノードである。  
本ノードは `route_planner` と `route_follower` の間に位置し、経路データの生成・更新・配信を行う。  
Phase2では、滞留検知に基づく**障害物回避・スキップ・再計画・保留（HOLD）** の判断処理を新たに追加し、  
動的な経路更新機能を実現する。  

---

## 第2章　責務とスコープ

### 責務
- 経路管理：`route_planner` から生成された経路を保持し、`route_follower` に配信。
- 再計画制御：滞留通報（`/report_stuck`）に応じて再計画／スキップ／HOLDを判断。
- 状態管理：ノード自身の運用状態を `/manager_status` トピックで公開。
- 経路更新配信：新しい `/active_route` を TRANSIENT_LOCAL QoS で再送信。
- GUI・他ノードとの整合を保ち、シナリオ全体の状態遷移を統括。

### スコープ外
- 障害物検知（obstacle_monitor）やローカル制御（follower）は対象外。

---

## 第3章　状態遷移設計

Phase2では、内部処理の簡素化とfollowerとの整合性確保のため、状態を4つに統合した。

| 状態名 | 概要 | 主なイベント | 次状態 |
|---------|------|----------------|---------|
| **IDLE** | 初期または全停止状態。経路未設定。 | `/get_route`完了、初回ルート受信 | RUNNING |
| **RUNNING** | 通常運用中。経路追従ノード稼働。 | `/report_stuck`受信 | UPDATING_ROUTE |
| **UPDATING_ROUTE** | 再計画またはスキップ中。 | `/active_route`配信完了 | RUNNING |
| **HOLDING** | planner失敗・通信エラー・封鎖などによる停止状態。 | operator再開／新ルート受信 | RUNNINGまたはIDLE |

### 状態遷移図（テキスト）

```
[初期状態]
    ↓
┌────────────┐
│   IDLE     │
└────────────┘
    │
    │ シナリオ開始／初回ルート受信
    ↓
┌────────────┐
│  RUNNING   │
└────────────┘
    │
    │ followerから report_stuck 受信
    ↓
┌──────────────────┐
│  UPDATING_ROUTE  │
└──────────────────┘
    │
    ├─ 成功／skip完了 → RUNNING
    └─ timeout／失敗 → HOLDING
              │
              ├─ operator再開 → RUNNING
              └─ mission終了 → IDLE
```

---

## 第4章　ノード構成概要

| 要素 | 役割 |
|------|------|
| `/active_route` (Publisher) | followerへ経路を配信。QoS=RELIABLE, TRANSIENT_LOCAL |
| `/manager_status` (Publisher) | manager自身の状態を通知 |
| `/report_stuck` (ServiceServer) | followerからのスタック通報を受け、方針を決定して応答 |
| `/update_route` (ServiceClient) | route_plannerへの再計画要求 |
| `/get_route` (ServiceClient) | 初期ルート要求 |
| `/block_detected` (Subscriber, Phase3対応) | 経路封鎖報告を受け、再計画を発火 |

---

## 第5章　機能仕様

### 5.1 通常運用（RUNNING）
- `/active_route` にて現在有効な経路をfollowerへ配信。
- `/report_stuck` 要求が到着するまで通常運用を継続。
- `/manager_status.state="running"` を周期1Hzで発行。

### 5.2 滞留通報処理（/report_stuck）
followerから滞留検知を受けた場合、以下の3層判断ロジックに基づき処理を行う。

#### 第1層：左右オフセット判定
- 経路上に左右余地があればオフセット付きで再計画を指示。  
- 条件：`reason in ("stagnation","avoidance_failed")`, `avoid_trial_count < avoid_max_retry`, `last_hint_blocked=True`
- 結果：`offset_hint` に ±offset_step を設定し、`decision=REPLAN`。

#### 第2層：スキップ判定
- 現WPが `skippable=True` かつ `dist_to_next < skip_threshold_m` の場合、短区間を飛ばして更新。
- 結果：`decision=SKIP`。内部で `active_route` をスライスし再配信。

#### 第3層：再計画／失敗判定
- planner通信が成功 → `decision=REPLAN`
- 失敗またはtimeout → `decision=FAILED`（HOLDINGへ遷移）

### 5.3 HOLDING状態
- planner連続失敗や経路喪失時に遷移。
- `/manager_status.state="holding"` を発行。
- GUIまたはoperator指令により `/get_route` を再要求可能。

---

## 第6章　入出力インタフェース

### トピック一覧

| 名称 | 型 | QoS | 方向 | 説明 |
|------|----|-----|------|------|
| `/active_route` | `route_msgs/Route` | RELIABLE / TRANSIENT_LOCAL | Pub | 経路配信 |
| `/manager_status` | `route_msgs/ManagerStatus` | RELIABLE / VOLATILE | Pub | ノード状態通知 |
| `/block_detected` | `route_msgs/BlockInfo` | RELIABLE / VOLATILE | Sub | 経路封鎖検知（Phase3） |

### サービス一覧

| 名称 | 型 | 方向 | 説明 |
|------|----|------|------|
| `/report_stuck` | `route_msgs/srv/ReportStuck` | Server | follower滞留通報受付 |
| `/update_route` | `route_msgs/srv/UpdateRoute` | Client | 再計画要求 |
| `/get_route` | `route_msgs/srv/GetRoute` | Client | 初期経路取得 |

---

## 第7章　メッセージ仕様

### ReportStuck.srv (再掲)

**Request**
```
string reason
string current_wp_label
uint32 avoid_trial_count
bool last_hint_blocked
float32 last_applied_offset_m
```

**Response**
```
uint8 decision  # 1=REPLAN,2=SKIP,3=FAILED
builtin_interfaces/Duration waiting_deadline
float32 offset_hint
string note
```

### ManagerStatus.msg
```
std_msgs/Header header
string state        # "idle","running","updating_route","holding"
string decision     # "none","replan","skip","failed"
string last_cause   # "stagnation","no_hint","no_space","road_block"
uint32 route_version
```

---

## 第8章　パラメータ

| パラメータ | 型 | 既定値 | 説明 |
|-------------|----|---------|------|
| `planner_timeout_sec` | float | 5.0 | route_planner応答待機時間 |
| `waiting_deadline_sec` | float | 8.0 | followerのWAITING最大待機時間 |
| `skip_threshold_m` | float | 0.8 | スキップ距離閾値 |
| `avoid_max_retry` | int | 3 | follower側局所回避上限回数 |
| `offset_step` | float | 0.5 | オフセットステップ幅[m] |

---

## 第9章　エラー処理・リカバリ

| 事象 | 処理 | 状態遷移 |
|------|------|----------|
| route_planner応答timeout | HOLDINGへ移行 | UPDATING_ROUTE→HOLDING |
| 再計画失敗 | HOLDINGへ移行しGUI通知 | UPDATING_ROUTE→HOLDING |
| follower未応答 | 無処理（再通報待ち） | RUNNING維持 |
| 通信断復旧後 | 自動で再接続 | 状態維持 |

---

## 第10章　Phase3拡張想定

Phase3では、動的経路封鎖（`road_block`）検知に基づき、
`RUNNING→UPDATING_ROUTE` への自律遷移を許可する。  
`offset_hint` パラメータをroute_plannerに転送し、  
左右オフセットを考慮した回避ルート生成を実現予定。

---

## 第11章　設計上の留意事項

- `/active_route` のQoSは **TRANSIENT_LOCAL** とし、follower再起動時にも受信可能にする。
- `/manager_status` はVOLATILEで構わないが、周期送信によりGUIが最新状態を把握。
- 状態は4種類に固定し、内部フラグで細分化しても外部公開しない。
- `/report_stuck` 応答は同期完了後、即座にdecisionを返す。blocking時間は200ms以下を目標。
- route_plannerとの通信は非同期futureを用い、timeout管理を徹底。

---

## 第12章　まとめ

- Phase2では、滞留通報に対する3層判断ロジックを実装し、
  - **左右オフセット再計画**
  - **短距離スキップ**
  - **再計画またはHOLD**
  の順で段階的に判断。
- managerの状態は `IDLE / RUNNING / UPDATING_ROUTE / HOLDING` の4つに統一。
- Phase3の経路封鎖リルートを含め、設計変更なしで拡張対応が可能。
