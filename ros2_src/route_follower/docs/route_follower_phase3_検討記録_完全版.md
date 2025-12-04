# route_follower_phase3_検討記録（最終更新版・完全版）

## 0. 文書目的
本書は route_follower ノードの Phase3 設計に関する検討経緯・決定事項を体系的に整理したものである。
従来の Phase1/Phase2 実装から拡張された要素（滞留検知・障害物回避・L字サブゴール制御・状態骨格構造化・reason_code拡張など）を中心に、
開発上の論点・判断理由・今後の発展方針を含む。

---

## 1. Phase1 からの変更点

| 区分 | Phase1 | Phase3 |
|------|---------|--------|
| 障害物対処 | 無し（停止のみ） | 滞留検知を契機とした局所回避（Hint使用） |
| 状態管理 | 単純フロー | FollowerStateEnum による明示的状態遷移管理 |
| 回避動作 | 無 | L字2段階サブゴール（横→前進） |
| ログ出力 | 最小限 | state遷移/滞留/回避/警告を明示出力 |
| STOP解除 | manual_start共通 | signal_stop は sig_recog(GO) も可 |
| ソース構造 | 単一関数集中 | 状態骨格明示 + 関数分離（on_xxx） |
| 異常検知 | 無 | Pose未受信・index越え・route無設定をWARN出力 |
| ReportStuck通報 | 理由文字列のみ | reason_code + reason_detail を追加し road_blocked を明示 |

---

## 2. 開発背景と目的

つくばチャレンジ等の実環境走行を想定し、経路上の一時的障害や環境誤差に対して、
安全かつ単純な挙動で回避を行うことを目的とする。  
Phase3でもグローバル経路の再計算までは行わず、ローカルな「横ずれ＋前進」での回避を実装範囲とした。

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
| 左右オフセット | front_blocked=True サンプルの中央値 |
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
| リクエスト | route_version:int32, current_index:int32, current_wp_label:string, current_pose_map:Pose, reason_code:uint8, reason_detail:string, avoid_trial_count:uint32, last_hint_blocked:bool, last_applied_offset_m:float |
| レスポンス | decision_code:uint8(1=replan/2=skip/3=failed), waiting_deadline:Duration, offset_hint:float, note:string |
| 呼出方式 | 同期（timeout=30s） |
| decision_code処理 | replan/skip→WAITING_REROUTE, failed→ERROR |

---

## 7. ログ・モニタリング仕様

| 種別 | 出力内容 |
|------|-----------|
| INFO | 状態遷移, 滞留検知, 回避開始/完了, report_stuck応答 |
| WARN | Pose未受信, route未設定, waypoint index異常, timeout |
| ERROR | frame不整合, サービス異常 |

---

## 8. 状態管理構成（Phase3最終）

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
 ├─ report_stuck decision_code=replan/skip → WAITING_REROUTE
 │        └─ 新route受信 → RUNNING
 └─ timeout30s → ERROR
```

---

## 10. 異常時安全設計

- Pose未取得時は移動制御を停止し、目標Pose再送のみ実行。
- routeまたはindex不正時は処理を中断してWARN出力。
- いずれも自動再開は行わず、外部ノードによる再経路指示を待機。

---

## 11. 信号認識ノード（ROS1）連携検討

- 目的: ROS2 側 route_follower から `recog_flag` を Publish し、`ros1_bridge` を介して ROS1 ノード `tc2023_signal_detector` の検出ループを起動できるようにする。

### 11.1 ROS1 側挙動の整理
- `recog_flag` が `1` を受信するまで検出処理を開始しない。`0` 等の場合は画像のデバッグ出力のみを継続する。【F:ros1_src/scripts/tc2023_signal_detector.py†L135-L173】
- `recog_flag==1` の間だけ YOLO 推論ループを回し、判定結果 `sig_recog` を Publish し続ける。`recog_flag` が 1 以外になるとループを抜け、再度フラグ待ちに戻る。【F:ros1_src/scripts/tc2023_signal_detector.py†L174-L222】

### 11.2 現行 route_follower での信号停止の扱い
- `sig_recog` は Subscribe 済みで、`signal_stop` ウェイポイント到達後の WAITING_STOP 解除条件として `sig_recog==1(GO)` を許可している（`manual_start` でも解除可）。【F:ros2_src/route_follower/route_follower/route_follower_node.py†L145-L154】【F:ros2_src/route_follower/route_follower/follower_core.py†L465-L492】
- ただし `recog_flag` 発行経路は未実装のため、ROS1 側の検出処理を起動できず `sig_recog` が届かないまま待ち続けるリスクがある。【F:ros2_src/route_follower/route_follower/follower_core.py†L518-L523】

### 11.3 実装方針（RouteFollowerNode 側の追加）
1. `std_msgs/Int32` 型 Publisher `recog_flag` を新設する。QoS は `sig_recog` と同じ RELIABLE/VOLATILE を使用し、`ros1_bridge` との整合を取る。
2. RouteFollowerNode 内に「現在のフラグ値」を保持する変数を追加し、値が変わったときのみ Publish してバンド幅を抑制する。
3. WAITING_STOP への遷移時に、対象ウェイポイントが `signal_stop=True` なら `recog_flag=1` を発行して検出を開始させる。line_stop の場合は 0 を維持する。
4. WAITING_STOP 解除（`sig_recog==1` もしくは `manual_start=True`）や RUNNING/FINISHED/ERROR への遷移時には `recog_flag=0` を送って検出ループを終了させる。初期化直後や route 未設定時も 0 を送っておくと安全。
5. 現在のウェイポイントの `signal_stop` 判定には `self.core.route` と `self.core.index` を参照する。状態遷移は `_handle_state_publish` 内で `output.state["status"]` を監視する形がシンプル。
6. トピック名は ROS1 側と揃えて `recog_flag` を維持し、bridge 側はデフォルトの動的ブリッジ（`ros1_bridge`）で `std_msgs/Int32` を透過させる。

### 11.4 ROS1 ノード側への影響と Publish 方針の調整
- 旧実装では検出ループ内で毎回 `rospy.wait_for_message('recog_flag', Int32, timeout=None)` を呼び出しており、RouteFollower 側が値変更時のみ `recog_flag` を Publish する実装だと 2 周目以降でメッセージ待ちにブロックする懸念があった。【F:ros1_src/scripts/tc2023_signal_detector.py†L15-L182】
- RouteFollower 側で周期送信を強いると「ROS1 ノードの推論周期まで ROS2 側が管理する」構造になり不自然で、`recog_flag` 駆動でしか検出を開始できないため `sig_recog` 受信タイミングも遅延する。
- 本件は ROS1 ノード側で「購読コールバックで `recog_flag` の最新値をキャッシュし、検出ループはキャッシュ値を参照する」形に改修し、値変化時の単発 Publish でもブロックせず動作させる。【F:ros1_src/scripts/tc2023_signal_detector.py†L15-L182】

### 11.5 実装状況（recog_flag 連携）
- RouteFollowerNode に `recog_flag` Publisher を追加し、WAITING_STOP で `signal_stop=True` のウェイポイント待機中のみ `1` をラッチ送信、それ以外の状態では `0` を送る。初期化時に `0` をラッチし、フラグ変化時のみ Publish することでブリッジ帯域を抑制する。【F:ros2_src/route_follower/route_follower/route_follower_node.py†L134-L200】【F:ros2_src/route_follower/route_follower/route_follower_node.py†L302-L444】
- `tc2023_signal_detector` は `recog_flag` サブスクライバで最新値をキャッシュし、待機ループ・推論ループはキャッシュ値を参照する構造へ改修済み。`recog_flag` が `0` に戻った時点で検出ループを抜け、単発 Publish にも追従する。【F:ros1_src/scripts/tc2023_signal_detector.py†L15-L182】

---

## 12. Phase3 拡張方針

### 12.1 反対側リトライ
- L字回避失敗時、左右方向を反転して1回のみ再試行。
- `_avoid_queue` 生成部に反転処理を追加することで容易に実装可能。

### 12.2 経路再構築連携
- route_managerからのreplan結果を即時適用し、
  WAITING_REROUTE→RUNNING遷移を非同期で処理。

### 12.3 状態拡張例
- RETRYING, REPLANNING などを追加し、詳細ログやリトライ回数を可視化。

---

## 13. 結論
Phase3でも滞留を唯一のトリガとした局所回避ロジックを維持しつつ、
L字サブゴール制御・状態骨格構成に加えて reason_code 拡張を実装し、
経路封鎖通報の明示化と route_manager との連携強化を実現した。
本仕様に基づき、`route_follower_node.py` が正式実装版である。
