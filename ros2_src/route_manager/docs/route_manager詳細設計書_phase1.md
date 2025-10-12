# route_manager_詳細設計書_phase1正式版

## 第1章 概要
`route_manager`ノードは、経路計画ノード（`route_planner`）から経路情報を取得し、他ノードへ配信する中心的な経路管理ノードである。  
phase1においては、経路要求・更新要求の骨格実装を完成させ、実システムと連携して安定動作を確認済みである。

### 主な機能
- 経路の初期取得 (`/get_route` サービス呼出)
- 経路更新要求 (`/update_route` サービス呼出)
- `/active_route`（経路情報）および `/route_state`（状態） のPublish
- `/follower_state`購読による走行進捗の反映
- `/mission_info`の配信（起動時）
- 経路取得タイムアウト・再試行制御

---

## 第2章 責務とスコープ
### 責務
- 経路データの一元管理  
- 経路更新リクエストの仲介  
- 他ノードへの状態／経路配信

### スコープ（phase1）
- 経路要求・更新要求の呼出処理と結果配信
- タイマー駆動による状態監視
- `/follower_state`の購読と`/route_state`更新

### 除外（今後の拡張）
- 経路バージョン整合性チェックの強化（phase2予定）
- reroute判定・障害物対応（phase2～3予定）

---

## 第3章 フェーズ構成
| フェーズ | 内容 |
|-----------|------|
| phase1 | 経路取得・更新要求、状態配信の実装（完了） |
| phase2 | reroute機能、障害物検知との統合 |
| phase3 | mission再構成・GUI連携による経路切替 |

---

## 第4章 外部I/F仕様

### トピック購読
| 名称 | 型 | 説明 |
|------|----|------|
| `/follower_state` | `FollowerState` | フォロワからの進捗通知（current_label使用） |

### トピック配信
| 名称 | 型 | 説明 |
|------|----|------|
| `/active_route` | `ActiveRoute` | plannerで生成された経路情報を配信。画像・統計等を含む。 |
| `/route_state` | `RouteState` | 経路状態（status, current_label, messageなど）を配信。 |
| `/mission_info` | `MissionInfo` | mission構成情報（start, goal, checkpoints, description任意） |

### サービス呼出
| 名称 | 型 | 説明 |
|------|----|------|
| `/get_route` | `GetRoute.srv` | 経路初期取得。planner発番versionを採用。 |
| `/update_route` | `UpdateRoute.srv` | 経路更新。`pose`・`reason`はoptional。 |

---

## 第5章 状態機械

| 状態名 | 説明 | 遷移条件 |
|---------|------|----------|
| UNSET | 初期状態 | 起動直後 |
| REQUESTING | 経路取得要求中（内部状態） | `/get_route`送信直後 |
| ACTIVE | 正常経路保持 | 経路取得成功時 |
| UPDATING | 経路更新要求中 | `/update_route`呼出時 |
| COMPLETED | 更新完了 | 更新成功時 |
| ERROR | エラー状態 | タイムアウト・応答失敗時 |

---

## 第6章 動作シーケンス
1. ノード起動  
2. `auto_request_on_startup` が True の場合 `/get_route` 呼出  
3. planner応答受信 → `/active_route` と `/route_state` Publish  
4. `/follower_state`購読により current_label 更新  
5. route更新要求発生時 `/update_route` 呼出  
6. タイムアウト時は最大 N 回再試行  
7. 完了後 `/route_state` を `COMPLETED` に更新

---

## 第7章 エラー処理
| 発生条件 | 処理内容 |
|-----------|-----------|
| planner応答なし | タイムアウトログ出力後 ERROR状態へ遷移 |
| 経路画像不正（encoding/長さ） | 無効経路扱い、ERROR状態へ遷移 |
| Future実行例外 | ログ出力後、再試行（上限N回） |
| サービス呼出失敗 | ERROR状態、再試行または停止 |

---

## 第8章 パラメータ
| 名称 | 型 | 既定値 | 説明 |
|------|----|--------|------|
| `start_label` | str | "" | 出発ノードラベル |
| `goal_label` | str | "" | 目的ノードラベル |
| `checkpoint_labels` | list(str) | [] | 経由ノードラベル |
| `planner_get_service_name` | str | "/get_route" | plannerのGetRouteサービス名 |
| `planner_update_service_name` | str | "/update_route" | plannerのUpdateRouteサービス名 |
| `planner_timeout_sec` | float | 5.0 | サービス応答タイムアウト |
| `planner_retry_count` | int | 3 | 再試行回数 |
| `publish_rate_hz` | float | 1.0 | 状態更新周期 |
| `auto_request_on_startup` | bool | True | 起動時に自動経路要求を行うか |
| `log.route_summary` | bool | False | 経路要約ログ出力フラグ |

---

## 第9章 QoS設定
| トピック | Reliability | Durability | Depth |
|-----------|--------------|-------------|-------|
| `/active_route` | RELIABLE | TRANSIENT_LOCAL | 1 |
| `/route_state` | RELIABLE | VOLATILE | 1 |
| `/follower_state` | RELIABLE | VOLATILE | 1 |
| `/mission_info` | RELIABLE | TRANSIENT_LOCAL | 1 |

---

## 第10章 内部構成
### 主クラス
`RouteManager(Node)`  
- タイマー駆動 (`publish_rate_hz`)
- Futureベース非ブロッキング呼出
- `_handle_get_route_response()` / `_handle_update_route_response()` による結果処理

### 主関数
| 関数 | 概要 |
|------|------|
| `_call_get_route()` | 経路初期要求。Future登録＋タイマー監視。 |
| `_call_update_route()` | 経路更新要求。失敗時リトライ制御。 |
| `_publish_active_route()` | 画像・経路情報を検証後Publish。stamp更新。 |
| `_publish_route_state()` | 状態・messageをPublish。 |
| `_on_follower_state()` | current_label更新。 |
| `_validate_route()` | encoding/data長検証。 |

---

## 第11章 将来拡張方針
| フェーズ | 拡張内容 |
|-----------|----------|
| phase2 | reroute判定・障害物検知連携、mission再構成 |
| phase3 | GUI操作による手動経路切替・version同期管理 |

---

## 第12章 テスト観点
| 区分 | テスト項目 |
|------|-------------|
| 通常動作 | `/get_route` 応答で `/active_route` がPublishされること |
| 更新動作 | `/update_route` 要求で `/route_state` が `UPDATING→COMPLETED` に遷移すること |
| フォロワ同期 | `/follower_state`更新によりcurrent_labelが追従すること |
| タイムアウト | planner無応答時、ERROR状態となること |
| 再試行 | `planner_retry_count`回の再試行が行われること |

---

## 第13章 保守・拡張上の留意事項
- Future＋タイマー方式によりデッドロックを回避  
- QoS設定を変更する場合、他ノード側設定も同時に見直すこと  
- 経路version整合はplanner主導で行う  
- 状態更新はスレッド安全に留意（CallbackGroup利用を推奨）

---

## 第14章 まとめ
本書は、`route_manager`ノードのphase1実装に完全対応した正式版詳細設計書である。  
本書のみで同一機能の再実装が可能であり、`route_follower`正式版と整合する構成で統一されている。

---

この正式版はガイド準拠・phase1完結版です。  
次のステップとして、phase2設計書更新（reroute機能統合）に進む前に、この版を他ノード連携設計の基準として確定することを推奨します。
