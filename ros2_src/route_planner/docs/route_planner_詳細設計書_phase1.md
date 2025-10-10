
# route_planner_詳細設計書_phase1正式版

> 本書は、`route_planner.py`（phase1実装済み）および `graph_solver.py` に**完全整合**する正式版の詳細設計書である。  

---

## 第1章 概要

### 1.1 ノードの目的
`route_planner` は、静的に定義された**固定ブロック**と**可変ブロック**から構成される経路を、要求に応じて生成・返却する**ルート配布サーバ**である。  
- `/get_route` : 指定された開始ラベル・終了ラベル・チェックポイントをもとに**経路を新規生成**する。  
- `/update_route` : 現行経路に対して、**可変ブロックの封鎖**（エッジクローズ）等を反映し、**再探索**を行う。  
- `graph_solver.solve_variable_route()` を呼び出し、**PNG画像**を生成した上で `sensor_msgs/Image(bgr8)` に積み替えて返す。

### 1.2 phase1 での実装範囲
- サービス：`/get_route`, `/update_route` を実装済み（同期実行）
- 画像：`graph_solver` が生成した PNG を読み込み `bgr8` で返却
- 状態保持：現行経路・訪問履歴・封鎖エッジ集合・version をノード内に保持
- 並行性：`MutuallyExclusiveCallbackGroup` により**サービス実行は直列化**
- タイムアウト：**phase1では未実装**（呼び出しは同期完了まで待機）

---

## 第2章 責務とスコープ

### 2.1 責務
- **経路計画**：固定/可変ブロックを組み合わせた経路生成
- **再計画**：封鎖情報に応じた可変ブロックの再探索と経路再構成
- **画像提供**：探索結果の**地図重畳画像**（PNG起点→`bgr8`）を返却

### 2.2 スコープ（phase1）
- 入力：YAML 構成（blocks 定義）＋ CSV 群（nodes/edges/segment）
- 出力：`Route` メッセージ（waypoints, image, version など）
- 例外系：不正入力、固定ブロック封鎖、画像生成失敗時のフォールバック

### 2.3 非スコープ（将来フェーズ）
- 非同期並列化・サービスレート制御
- 自動リルート（外部イベント購読）
- 画像描画のサイズ/表レイアウトのノード内パラメトリック化

---

## 第3章 フェーズ構成

| フェーズ | 実装内容（本ノードに関する要点） |
|:--|:--|
| phase1 | `/get_route` と `/update_route` の同期サーバ。現行経路・履歴・封鎖集合の保持。graph_solver 連携で PNG→`bgr8`。 |
| phase2 | 外部イベント起点の再計画、manager 連携の強化、パラメータ拡張（予定）。 |
| phase3 | 統合可視化・高度最適化指標など（予定）。 |

---

## 第4章 外部I/F仕様

### 4.1 サービス
| 名称 | 型 | 概要 |
|:--|:--|:--|
| `/get_route` | `route_msgs/srv/GetRoute` | ブロック構成とラベル情報をもとに**経路生成**を行い、`Route` を返す。 |
| `/update_route` | `route_msgs/srv/UpdateRoute` | **現行経路**を前提に**可変ブロック**の再探索を行い、`Route` を返す。固定ブロック封鎖は**失敗**を返す。 |

> 備考：どちらも**同期**であり、完了まで呼び出し元は待機する。

### 4.2 メッセージ（要素の意味）
- `Route`（`route_msgs/msg/Route`）  
  - `waypoints: Waypoint[]` … 経路順のウェイポイント列  
    - `Waypoint.index: int` … **その Route 内の順序**（0..N-1）  
    - `Waypoint.label: string` … ノードの**地物識別**。**再訪**があり得る（indexと無関係）。  
    - `Waypoint.pose: geometry_msgs/Pose`（x, y, z, qx, qy, qz, qw）  
    - `Waypoint.reach_tolerance: float` … 到達判定距離  
  - `image: sensor_msgs/Image` … `bgr8` 形式で 1 枚返す（後述）  
  - `version: int` … ノード内で単調増加（phase1実装準拠）  
  - `meta: 任意の補助情報`（phase1は必要最小限のみ設定）

> **重要**：`label` は地物同一性、`index` は**同一Route内の通過順**を表す。再訪時は `label` が同じで `index` は異なる。

### 4.3 画像（`sensor_msgs/Image`）
- エンコーディング：**`bgr8` 固定**
- 生成方法：`graph_solver` が出力した PNG を**読み込み**、`bgr8` に変換して格納
- 失敗時：**ダミー画像**（テキストPNG）を生成して返却

---

## 第5章 パラメータ

| 名称 | 型 | 既定値 | 説明 |
|:--|:--|:--|:--|
| `config_yaml_path` | `string` | `""` | ルート構成 YAML（blocks 定義）への絶対/相対パス |
| `csv_base_dir` | `string` | `""` | CSV（nodes/edges/segment）基準ディレクトリ |

> **運用**：いずれも起動時に `declare_parameter()` で宣言。空値の場合はエラーログを出す。

---

## 第6章 QoS と並行性

- 本ノードは**サービスのみ**（phase1）で、トピックの publish/subscribe はしない。  
- コールバックグループ：`MutuallyExclusiveCallbackGroup`（**直列実行**）

---

## 第7章 内部状態（保持データ）

| 名称 | 型 | 初期値 | 概要/更新契機 |
|:--|:--|:--|:--|
| `blocks` | `List[Dict[str, Any]]` | `[]` | YAML からロードした**ブロック配列**。固定/可変を含む。 `/get_route` 受信時などに更新。 |
| `current_route` | `List[Dict[str, Any]]` | `[]` | 直近で配布した**経路のウェイポイント列**（辞書構造）。 `/get_route` 成功時に更新。 |
| `current_route_origins` | `List[str]` | `[]` | 各 waypoint の**由来（ブロック名/エッジなど）**を並走管理。 |
| `visited_checkpoints_hist` | `Set[str]` | `set()` | **訪問済みチェックポイント**の永続化。再訪回避に使用。 |
| `closed_edges` | `Set[Tuple[str,str]]` | `set()` | **封鎖中エッジ集合**（`(U,V)` 形式）。`/update_route` で追加。 |
| `route_version` | `int` | `0` | 応答 `Route.version` に設定する **単調増加**値。`/get_route` でリセット→加算。 |

---

## 第8章 入力ファイル仕様（YAML/CSV）

### 8.1 YAML（`config_yaml_path`）
- 取得順序：  
  1) ルート直下 `blocks`  /  2) `route_planner.ros__parameters.blocks` の**いずれか**を読み込む  
- `blocks[i]` は**固定**または**可変**。最小要素は以下：  
  - `type`: `"fixed"` または `"variable"`  
  - `nodes_csv`: ノードCSVへのパス（`csv_base_dir` からの相対または絶対）  
  - `edges_csv`: エッジCSVへのパス  
  - `segments`: セグメント（Waypoint CSV）群の定義（可変ブロックで使用）  
  - `start_label` / `goal_label` / `checkpoints`（必要に応じて記載）

> 実装は**存在/型チェック**とパス解決（相対→`csv_base_dir` 基準）を行う。

### 8.2 CSV
- `nodes.csv` : `id, lat, lon` または `id, x, y`（いずれも**数値**）  
- `edges.csv` : `source, target, segment_id, reversible`  
  - `reversible` は `0/1`, `true/false`, `yes/no` を受理  
- **Waypoint CSV（segment）**：`(x,y)` または `(lat,lon)` 形式のいずれかを許容

---

## 第9章 主要処理（関数仕様）

> 本章は**関数単位**に「目的／引数／戻り値／主要ロジック／例外」を示す。関数名・責務は実装に一致させる。

### 9.1 `load_config()`（ノード内メソッド）
- **目的**：`config_yaml_path` を読み取り、`blocks` を構築する。  
- **引数/戻り値**：なし／なし（副作用で `self.blocks` を更新）  
- **主要ロジック**：
  1. YAML を `safe_load()` で読込  
  2. `blocks` の探索：`route_planner.ros__parameters.blocks` → ルート直下 `blocks`  
  3. 各ブロックのファイルパスを `csv_base_dir` 基準に解決し `self.blocks` へ設定  
- **例外/エラー**：読込失敗時はエラーログを出し `self.blocks=[]` にする。

### 9.2 `slice_by_labels(wps, start_label, goal_label)`（モジュール関数）
- **目的**：ウェイポイント列から**開始/終了ラベルでサブ配列を抽出**する。  
- **引数**：`wps: List[Waypoint]`, `start_label: Optional[str]`, `goal_label: Optional[str]`  
- **戻り値**：`(sliced_list, start_offset: int)`  
- **仕様**：
  - `start_label` 未指定→**先頭**、`goal_label` 未指定→**末尾**を採用  
  - `start_idx > goal_idx` は**エラー**  
  - 戻り値の `start_offset` は**元配列に対する切り出し開始位置**

### 9.3 `indexing(wps)`（モジュール関数）
- **目的**：`Waypoint.index` を **0..N-1** で**再採番**する。  
- **引数/戻り値**：`wps: List[Waypoint]`／（戻り値なし、**副作用**で `index` を更新）

### 9.4 `adjust_orientations(wps)`（モジュール関数）
- **目的**：各 `Waypoint.pose.orientation` を**隣接点の方位**に基づいて補正する。  
- **引数/戻り値**：`wps: List[Waypoint]`／（副作用で更新）  
- **仕様**：`yaw_to_quaternion(yaw)` を用い、2点間の `atan2(dy, dx)` を yaw として反映。

### 9.5 `pack_route_msg(wps, image_png_path)`（モジュール関数）
- **目的**：`Route` メッセージを**構築**する。  
- **引数**：`wps: List[Waypoint]`, `image_png_path: Optional[str]`  
- **戻り値**：`Route`  
- **仕様**：PNG を読み込めた場合は `bgr8` に変換し `Image` を設定。失敗時は**テキストPNG**を生成して代入。

### 9.6 `solve_variable_route()`（外部：graph_solver.py）
- **目的**：無向グラフ上で `start`→`checkpoints群`→`goal` の最短経路（順序最適化）を解く。  
- **引数**：`nodes`, `edges`, `start`, `goal`, `checkpoints`  
- **戻り値**：`Dict[str, Any]`  
  - `edge_sequence: List[Dict]` … `segment_id` を含む区間列  
  - `node_sequence: List[str]` … ノード通過列  
  - `visit_order: List[str]` … 端点訪問順  
  - `stats: Dict` … JSON直列化可能な統計（行列は文字列キー化済み）  
  - `route_image_path: str` … 生成 PNG の実パス（`/tmp/route_solver_<pid>.png` など）

---

## 第10章 サービス処理フロー

### 10.1 `/get_route`
1. **設定読込**：`load_config()` で `blocks` を準備
2. **ブロック順の展開**：固定ブロックは**CSV の順**を採用。可変ブロックは `graph_solver` で探索
3. **チェックポイント統合**：YAML 定義＋リクエストを**和集合**化し、**当該ブロックに存在するもの**へ限定。履歴から**訪問済みは除外**
4. **ウェイポイント連結**：重複端点は `concat_with_dedup()` で**重複除去結合**（位置がほぼ等しい場合に 1 点化）
5. **ラベルスライス**：`slice_by_labels()` により start/goal 範囲で切出し
6. **採番/姿勢補正**：`indexing()` → `adjust_orientations()`
7. **画像**：可変ブロック探索結果の `route_image_path` を**優先**して読み込み、`bgr8` で格納
8. **状態更新**：`current_route`, `current_route_origins`, `visited_checkpoints_hist` を更新。`route_version` を**加算**
9. **応答**：`Route` を返却

### 10.2 `/update_route`
1. **前提確認**：`current_route` が空なら**失敗**（再探索の対象なし）
2. **封鎖適用**：リクエストの封鎖エッジ `(U,V)` を `closed_edges` に追加。**固定ブロックにかかる封鎖**は**失敗**
3. **境界接続**：現在位置 `current_pose` を**先頭**に置いた**仮想エッジ**（`current → prev → U`）で**可変ブロック**の探索入力を構成
4. **再探索**：閉塞を除いた **可変ブロック**のみ `graph_solver` で再計算
5. **連結/補正**：元ルートの非対象区間＋再探索区間を**連結**し、`indexing()` と `adjust_orientations()` を適用
6. **画像**：`route_image_path` を読み込み（不可時はテキストPNG）
7. **状態更新/応答**：`current_route` 等を更新し、`Route` を返却（`version` 加算）

---

## 第11章 エラー処理とログ

| 事象 | 動作（失敗応答 or フォールバック） |
|:--|:--|
| YAML 読込失敗 / blocks 不正 | エラーログ出力。`/get_route` は失敗応答。 |
| start/goal 未検出 | エラーログ出力。失敗応答。 |
| 固定ブロック上の封鎖 | `/update_route` は**失敗**（可変のみ対応） |
| 画像読込失敗 | **テキストPNG**を生成して `image` に格納 |
| 入力 CSV 異常 | エラーログ出力。失敗応答。 |

---

## 第12章 動作シーケンス（テキスト）

- **通常（/get_route）**：  
  `load_config → 各ブロック処理（固定=直結, 可変=solver） → 重複結合 → ラベルスライス → 採番 → 姿勢補正 → 画像読込 → 状態更新 → 応答`

- **再計画（/update_route）**：  
  `前提確認 → 封鎖適用（可変のみ） → 仮想エッジを含む再探索入力生成 → solver 再計算 → ルート連結 → 採番/補正 → 画像読込 → 状態更新 → 応答`

---

## 第13章 テスト観点（phase1）

| 観点 | 代表テスト |
|:--|:--|
| 経路生成 | 正常系：固定+可変ブロックを含む構成で `/get_route` が成功し、`index` が 0..N-1 連番である。 |
| 画像 | `route_image_path` による PNG 読込が成功し、`bgr8` で返却される。失敗時はテキストPNG。 |
| 再訪/ラベル | 同一ラベルの再訪時に `index` は別値になる。 |
| スライス | start/goal 未指定時のデフォルト（先頭/末尾）が適用される。 |
| 再計画 | 可変ブロック封鎖時に `/update_route` が成功し、固定ブロック封鎖では失敗する。 |
| 直列性 | 並列複数呼び出しを行っても排他により競合しない。 |

---

## 第14章 保守・拡張の留意事項

- `graph_solver` の**入力/出力I/F**（特に `route_image_path`）の**互換維持**  
- `Waypoint` の**index再採番**は**副作用関数**であり、戻り値を用いない前提で実装すること  
- データ互換：`edges.csv` の `reversible` は複数表記（0/1, true/false, yes/no）を許容  
- 画像ファイル名は `graph_solver` に依存（`/tmp/route_solver_<pid>.png` など）。同一プロセスは上書き、別プロセスは別名。

---

## 第15章 まとめ

本書は phase1 実装に**完全整合**し、API・データ定義・アルゴリズム・例外系・状態管理までを**関数粒度**で明記した。  
この記述のみで**同一機能を再実装可能**である。

