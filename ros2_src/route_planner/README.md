# route_planner パッケージ README (phase2正式版)

## 概要
`route_planner` は固定ブロックと可変ブロックを組み合わせてルートを生成し、
`route_manager` からの `/get_route`・`/update_route` サービス要求に応答する経路計画ノードです。Phase2 では
YAML 設定、CSV キャッシュ、グラフ探索を組み合わせて再計画に対応します。

## 主な機能
- `routes/config.yaml`（既定）で定義された `blocks` を連結し、`Route` メッセージと PNG 画像を生成。
- Waypoint CSV をキャッシュして `GetRoute` / `UpdateRoute` の双方で再利用。
- 可変ブロックでは `graph_solver.solve_variable_route()` を用いて最短経路探索と画像生成を実行。
- `/update_route` 要求時は滞留地点を基点に仮想エッジを構築し、封鎖エッジを除外して再探索。
- `Route.version` を管理し、初期要求で 1 に初期化、再計画成功ごとにインクリメント。

## 起動方法
1. `routes/` 配下に YAML・CSV を配置し、`config_yaml_path` と `csv_base_dir` が参照できる状態にする。
2. 依存ファイルを共有ディレクトリへ展開するため `colcon build` → `source install/setup.bash` を実行する。
3. 以下のコマンドでノードを起動し、サービスを提供する。
   ```bash
   ros2 run route_planner route_planner
   ```

> CSV をビルド後に変更した場合は再度 `colcon build` を実行してください。`package.xml` の install 設定により再配置されます。

## 外部インタフェース
### Service Server
| 名称 | 型 | 説明 |
|------|----|------|
| `/get_route` | `route_msgs/srv/GetRoute` | YAML 定義に基づきルートを生成して返却。成功時は `Route.version=1` に初期化。 |
| `/update_route` | `route_msgs/srv/UpdateRoute` | 滞留地点情報を受け取り、可変ブロックを再探索して部分ルートを返却。成功時は `Route.version` を加算。 |

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `config_yaml_path` | string | `routes/config.yaml` | ルート構成 YAML ファイルへのパス。`FindPackageShare` で解決。 |
| `csv_base_dir` | string | `routes` | CSV 探索基準ディレクトリ。空文字時は YAML の所在ディレクトリを使用。 |
| `map_image_path` | string | `""` | 可変ブロック描画用の地図画像パス。パッケージルートからの相対パスを指定可能。 |
| `map_worldfile_path` | string | `""` | 地図画像に対応するワールドファイルのパス。パッケージルートからの相対パスを指定可能。 |

## 状態管理・処理フロー
### YAML `blocks` の構成
`blocks` は固定 (`type: fixed`) と可変 (`type: variable`) のブロックを順に記述します。YAML または
`route_planner.ros__parameters.blocks` のどちらにも同じ形式で定義できます。

```yaml
blocks:
  - type: fixed
    name: "A_fixed"
    segment_id: "fixed/waypoints_A.csv"
  - type: variable
    name: "B_variable"
    nodes_file: "variable/nodes.csv"
    edges_file: "variable/edges.csv"
    start: "C100"
    goal: "C200"
    checkpoints: ["C150"]
```

#### 固定ブロックのキー
| キー | 型 | 説明 |
|------|----|------|
| `type` | str | `"fixed"` 固定ブロックを表す。 |
| `name` | str | ブロック識別名（ログ表示・履歴管理に利用）。 |
| `segment_id` | str | Waypoint CSV のパス（`csv_base_dir` からの相対）。 |

固定ブロックは CSV をそのまま連結します。封鎖再計画の対象外であり、閉塞が発生した場合は `UpdateRoute` が失敗応答となります。

#### 可変ブロックのキー
| キー | 型 | 説明 |
|------|----|------|
| `type` | str | `"variable"` 可変ブロックを表す。 |
| `name` | str | ブロック識別名。 |
| `nodes_file` | str | グラフノード定義 CSV。 |
| `edges_file` | str | エッジ定義 CSV。 |
| `start` | str | 可変区間の開始ノード ID。 |
| `goal` | str | 可変区間の終了ノード ID。 |
| `checkpoints` | list[str] | 必須経由ノード ID のリスト（任意）。 |

可変ブロックは `solve_variable_route()` で最短経路を算出し、`edge_sequence` に列挙されたセグメントを連結します。
CSV の Waypoint は `segment_id` 単位でキャッシュされ、進行方向に応じて反転されます。

### CSV 仕様
- Waypoint CSV: `label,latitude,longitude,x,y,z,q1,q2,q3,q4,right_is_open,left_is_open,line_is_stop,signal_is_stop,isnot_skipnum`
- グラフ CSV: `nodes.csv`（`id,lat,lon,...`）、`edges.csv`（`source,target,waypoint_list,reversible[,weight_factor]`）

`waypoint_list` には Waypoint CSV への相対パスを記述します。`reversible=0` の場合は片方向エッジとして扱われます。
`weight_factor` は任意の正の実数で、指定するとセグメント長に係数を掛けた重みでグラフ探索を行います。

### `/get_route` の処理
1. `blocks` を順番に処理し、固定ブロックは CSV を連結、可変ブロックは最短経路を探索する。
2. `start_label` / `goal_label` 指定時は該当ラベルでスライスし、Waypoint を再採番する。
3. 逆走区間やブロック境界では姿勢を再計算し、PNG が存在すれば `Route.route_image` に格納する。無い場合はテキスト PNG を生成する。
4. 現行ルート・チェックポイント履歴を保存し、`last_request_checkpoints` を更新する。

### `/update_route` の処理
1. `prev_index` / `current_index` が隣接し同一可変ブロックかを検証する。
2. 封鎖エッジ `{U,V}` をグラフビルダから除去し、ブロックが変わった場合は設定をリセットする。
3. 現在位置 `current_pose` から仮想エッジ (current→prev→U) の Waypoint 群を生成し、新ルート冒頭に連結する。
4. 封鎖済みエッジを除外し、未通過チェックポイントを考慮した再探索を実行する。
5. 再探索結果と後続ブロックを連結して姿勢補正・距離計算を行い、`Route.version` を加算する。
6. 生成した `Route` と由来情報を保存し、探索失敗時や固定ブロック閉塞時は `success=False` を返す。

## 動作確認手順
1. YAML と CSV を `routes/` に配置し、`colcon build` → `source install/setup.bash` を実行する。
2. `ros2 run route_planner route_planner` を起動し、`/get_route` `/update_route` サービスが利用可能であることを確認する。
3. 初期ルート生成を確認するには次を実行する。
   ```bash
   ros2 service call /get_route route_msgs/srv/GetRoute "{start_label: '', goal_label: '', checkpoint_labels: []}"
   ```
4. 滞留再計画を確認するには、`route_manager` などから `/update_route` を呼び出し、`prev_index` と `current_index` に隣接インデックスを指定する。

## デバッグのヒント
- エラー発生時はスタックトレースを出力するため、`GetRoute` / `UpdateRoute` 失敗時は YAML や CSV の記述ミスを確認する。
- 再探索で閉塞が多発する場合は `GetRoute` を再呼び出しし、`closed_edges` をリセットして最新ルートを取得する。
- 出力される PNG パスは `solve_variable_route` の戻り値 `route_image_path` に含まれる。ファイルが無い場合はダミー画像が生成される。

## 将来拡張メモ
- タイムアウト管理（`planner_timeout_sec` など）は `route_manager` が担当し、本ノードは計算とファイル入出力に特化している。
- 可変ブロック単位の統計情報や可視化 API の追加余地がある。
