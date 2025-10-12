# route_manager パッケージ README (phase1正式版)

## 概要
`route_manager` は、経路計画ノード（`route_planner`）から経路情報を取得し、システム全体に配信する **経路管理ノード** です。  
本READMEは **phase1正式版** に対応しており、他ノード（`route_planner`, `route_follower`）との統合動作を前提にしています。

phase1では、経路要求・配信・状態管理の骨格実装を完了しており、システム全体との連携動作が確認済みです。  
`/update_route` による経路更新処理は、**phase2以降で実装予定** です。

---

## 主な機能（phase1対応範囲）
- `/get_route` サービスによる経路初期取得  
- `/active_route` トピックによる経路情報配信  
- `/route_state` トピックによる経路状態通知  
- `/mission_info` トピックによる経路構成情報配信  
- `/follower_state` の購読による走行進捗同期  
- `/update_route` 呼出は**phase2以降の拡張対象**（本phaseでは骨格のみ実装）

---

## 起動方法

### 1. 単独起動
```
ros2 launch route_manager route_manager.launch.py start_label:=START goal_label:=GOAL
```

例：
```
ros2 launch route_manager route_manager.launch.py start_label:=A1 goal_label:=B5
```

- `start_label`：経路の始点ラベル(未指定の場合は先頭から開始)
- `goal_label`：経路の終点ラベル(未指定の場合は末尾まで継続)
- launch内では `OpaqueFunction` を用いて LaunchConfiguration を Node に展開します。

### 2. 起動結果確認
起動後、以下のトピックが生成されます：
```
/active_route
/route_state
/mission_info
```
状態遷移は `/route_state` トピックで監視できます。

---

## 外部インタフェース一覧

### トピック購読
| 名称 | 型 | 説明 |
|------|----|------|
| `/follower_state` | `FollowerState` | 経路追従ノードの進捗を購読（current_label使用） |

### トピック配信
| 名称 | 型 | 説明 |
|------|----|------|
| `/active_route` | `ActiveRoute` | 現在の経路情報（画像含む） |
| `/route_state` | `RouteState` | 経路状態（status, current_label, message） |
| `/mission_info` | `MissionInfo` | 経路構成情報（start, goal, checkpoints, description任意） |

### サービス呼出
| 名称 | 型 | 説明 |
|------|----|------|
| `/get_route` | `GetRoute.srv` | 経路の初期取得。planner発番のversionを採用。 |
| `/update_route` | `UpdateRoute.srv` | 経路更新。**phase2以降で本格対応予定**。本phaseでは呼出骨格のみ実装。 |

---

## 主なパラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `start_label` | str | "" | 出発ノードラベル(未指定の場合は先頭から開始) |
| `goal_label` | str | "" | 目的ノードラベル(未指定の場合は末尾まで継続) |
| `planner_get_service_name` | str | "/get_route" | 経路取得サービス名 |
| `planner_update_service_name` | str | "/update_route" | 経路更新サービス名 |
| `planner_timeout_sec` | float | 5.0 | サービス呼出タイムアウト |
| `planner_retry_count` | int | 3 | 再試行回数 |
| `publish_rate_hz` | float | 1.0 | 状態更新周期 |
| `auto_request_on_startup` | bool | True | 起動直後に経路要求を自動実行するか |

---

## 状態遷移
| 状態 | 概要 |
|------|------|
| UNSET | 初期状態 |
| REQUESTING | 経路要求中（内部状態） |
| ACTIVE | 経路有効 |
| UPDATING | 更新要求中（phase2以降で使用） |
| COMPLETED | 更新完了（phase2以降で使用） |
| ERROR | エラー発生時 |

---

## QoS設定
| トピック | Reliability | Durability | Depth |
|-----------|--------------|-------------|-------|
| `/active_route` | RELIABLE | TRANSIENT_LOCAL | 1 |
| `/route_state` | RELIABLE | VOLATILE | 1 |
| `/mission_info` | RELIABLE | TRANSIENT_LOCAL | 1 |

---

## 動作確認方法

1. **ノード起動**
   ```bash
   ros2 launch route_manager route_manager.launch.py start_label:=S goal_label:=G
   ```

2. **plannerノード起動**
   ```bash
   ros2 run route_planner route_planner
   ```

3. **フォロワノード起動**
   ```bash
   ros2 run route_follower route_follower
   ```

4. **状態確認**
   ```bash
   ros2 topic echo /route_state
   ```

5. **更新テスト（phase2以降対象）**
   ```bash
   ros2 service call /update_route route_interfaces/srv/UpdateRoute "{reason: 'reroute test'}"
   ```

---

## 内部構成要約
- `RouteManager` クラス内で全処理を統括。  
- Future＋Timerによる非ブロッキング構成。  
- `planner_retry_count` に基づく再試行制御。  
- `/active_route` 発行時に `stamp` を上書き。  
- エラー時は `/route_state` に `ERROR` 状態をPublish。

---

## テストとデバッグ
| テスト項目 | 期待結果 |
|-------------|-----------|
| 初期経路取得 | `/active_route` が1回Publishされる |
| フォロワ同期 | `/follower_state`更新によりcurrent_labelが追従する |
| planner無応答 | ERROR状態となり再試行後停止 |
| 更新要求（参考） | `/update_route` はphase2以降で有効化予定 |

---

## 将来拡張（phase2以降）
- `/update_route` 機能の本格実装（reroute対応）  
- 障害物検知との統合（`obstacle_monitor`連携）  
- mission再構成・GUI操作による経路切替  
- version整合性管理の自動化

---

## 保守上の注意
- QoS設定変更時はplanner・follower両方を見直すこと。  
- 状態更新はスレッド安全性を確保するため、`MutuallyExclusiveCallbackGroup` の利用を推奨。  
- 経路画像のencodingやデータ長の検証を省略しないこと。

---

