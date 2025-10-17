# route_follower_phase2_検討記録（最終更新版・完全版）

## 0. 文書目的
本書は route_follower ノードの Phase2 設計に関する検討経緯・決定事項を体系的に整理したものである。  
従来の Phase1 実装から拡張された要素（滞留検知・障害物回避・L字サブゴール制御・状態骨格構造化など）を中心に、
開発上の論点・判断理由・今後の発展方針を含む。

---

## 1. Phase1 からの変更点

| 区分 | Phase1 | Phase2 |
|------|---------|--------|
| 障害物対処 | 無し（停止のみ） | 滞留検知を契機とした局所回避（Hint使用） |
| 状態管理 | 単純フロー | FollowerStateEnum による明示的状態遷移管理 |
| 回避動作 | 無 | L字2段階サブゴール（横→前進） |
| ログ出力 | 最小限 | state遷移/滞留/回避/警告を明示出力 |
| STOP解除 | manual_start共通 | signal_stop は sig_recog(GO) も可 |
| ソース構造 | 単一関数集中 | 状態骨格明示 + 関数分離（on_xxx） |
| 異常検知 | 無 | Pose未受信・index越え・route無設定をWARN出力 |

---

## 2. 開発背景と目的

つくばチャレンジ等の実環境走行を想定し、経路上の一時的障害や環境誤差に対して、
安全かつ単純な挙動で回避を行うことを目的とする。  
Phase2ではグローバル経路の再計算までは行わず、ローカルな「横ずれ＋前進」での回避を実装範囲とした。

---

## 3. 検討経緯の要約

### 3.1 滞留検知方式
- map座標系（/amcl_pose）基準で過去位置を比較。
- 直近2秒間での距離変化と速度を評価し、15秒連続で基準未満の場合に滞留成立。
- STOP（line_stop / signal_stop）中は検出対象外。

### 3.2 回避トリガ
- 滞留検知が唯一の回避トリガ。
- 滞留後に obstacle_monitor の Hint を評価し、front_blocked 多数決が True の場合のみ回避可。

### 3.3 Hint 情報の評価
| 項目 | 内容 |
|------|------|
| データ源 | obstacle_monitor /obstacle_avoidance_hint |
| キャッシュ長 | 5秒 |
| 判定 | front_blocked=True の比率 >= 0.8 |
| 左右開放度 | front_blocked=True サンプルの中央値 |
| 出力0値 | 障害物で閉塞時は0.0、障害物なしは0.75以上 |

### 3.4 回避方針
- 横方向オフセット = min(hint値, waypoint上限, 5.0m)
- 最小オフセット 0.35m を下限として確保。
- 前進距離 0.5m。
- 両方向可の場合は offset が小さい方を採用（同値は左優先）。

---

## 4. L字サブゴール方式

### 4.1 概要
回避時は2つのサブゴールを順に通過する：  
① 横シフト → ② 前進。  
いずれも base_link +Y/-Y 方向とその延長上に設定する。

### 4.2 サブゴール生成式
```
dx1 = -sin(yaw)*offset_y
dy1 =  cos(yaw)*offset_y
dx2 =  cos(yaw)*forward
dy2 =  sin(yaw)*forward
```
各点の姿勢は「直前点→当該点」の方向を yaw として設定。

### 4.3 判定
- サブゴール(1)・(2)とも到達後に次段発行。
- どちらの段階でも滞留検知が成立した場合は avoidance_failed として report_stuck。

---

## 5. WAITING_STOP状態の挙動

| Waypoint種別 | 解除条件 | 備考 |
|---------------|-----------|------|
| line_stop | manual_start=True | sig_recogは無効 |
| signal_stop | manual_start=True または sig_recog==1(GO) | 赤(2)や未定義値では解除しない |

---

## 6. /report_stuck 呼び出し仕様

| 項目 | 値 |
|------|----|
| サービス型 | route_msgs/srv/ReportStuck |
| リクエスト | route_version:int32, current_index:int32, current_wp_label:string, current_pose_map:Pose, reason:string, avoid_trial_count:uint32, last_hint_blocked:bool, last_applied_offset_m:float |
| レスポンス | decision:uint8(1=replan/2=skip/3=failed), waiting_deadline:Duration, offset_hint:float, note:string |
| 呼出方式 | 同期（timeout=30s） |
| decision処理 | replan/skip→WAITING_REROUTE, failed→ERROR |

---

## 7. ログ・モニタリング仕様

| 種別 | 出力内容 |
|------|-----------|
| INFO | 状態遷移, 滞留検知, 回避開始/完了, report_stuck応答 |
| WARN | Pose未受信, route未設定, waypoint index異常, timeout |
| ERROR | frame不整合, サービス異常 |

---

## 8. 状態管理構成（Phase2最終）

### 8.1 状態一覧

| 状態 | 内容 |
|------|------|
| IDLE | 経路待機（初期） |
| RUNNING | 経路追従・滞留監視 |
| WAITING_STOP | 停止状態、解除待ち |
| STAGNATION_DETECTED | 滞留検出後の一時状態 |
| AVOIDING | L字サブゴール実行中 |
| WAITING_REROUTE | report_stuck応答待機 |
| FINISHED | 全waypoint到達完了 |
| ERROR | 異常停止 |

### 8.2 タイマーコールバック構造
`_on_timer()` で状態骨格を保持し、各状態ごとに専用ハンドラ関数 `_on_xxx()` を呼び出す構造。  
この方式により状態遷移図との整合性が高く、フェーズ拡張にも対応容易。

---

## 9. 状態遷移要約フロー

```
IDLE
 ↓ /active_route受信
RUNNING
 ├─ waypoint到達 → WAITING_STOP(line/signal)
 │        └─ manual_start/sig_recog(GO) → RUNNING or FINISHED
 ├─ 滞留検知 → STAGNATION_DETECTED
 │        ├─ Hint不足 / 空間無 → report_stuck(unknown_stuck)
 │        └─ Hint有 / 回避可 → AVOIDING
 │                ├─ L字完了 → RUNNING
 │                └─ 再滞留 → report_stuck(avoidance_failed)
 ├─ report_stuck decision=replan/skip → WAITING_REROUTE
 │        └─ 新route受信 → RUNNING
 └─ timeout30s → ERROR
```

---

## 10. 異常時安全設計

- Pose未取得時は移動制御を停止し、目標Pose再送のみ実行。
- routeまたはindex不正時は処理を中断してWARN出力。
- いずれも自動再開は行わず、外部ノードによる再経路指示を待機。

---

## 11. Phase3 拡張方針

### 11.1 反対側リトライ
- L字回避失敗時、左右方向を反転して1回のみ再試行。
- `_avoid_queue` 生成部に反転処理を追加することで容易に実装可能。

### 11.2 経路再構築連携
- route_managerからのreplan結果を即時適用し、
  WAITING_REROUTE→RUNNING遷移を非同期で処理。

### 11.3 状態拡張例
- RETRYING, REPLANNING などを追加し、詳細ログやリトライ回数を可視化。

---

## 12. 結論
Phase2では、滞留を唯一のトリガとした堅牢な局所回避ロジックを確立し、
L字サブゴール制御・状態骨格構成によりフェーズ3以降の拡張基盤を整えた。  
本仕様に基づき、route_follower_phase2_final.py が正式実装版である。
