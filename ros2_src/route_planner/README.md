# route_planner パッケージ README (phase1正式版・完全改訂版)

## 概要
`route_planner` は、固定ブロックと可変ブロックを組み合わせてルートを生成する経路計画ノードです。  
`graph_solver` を利用して最短経路を算出し、PNG画像を `bgr8` 形式で返します。

---

## ファイル配置と利用手順

1. **routesディレクトリ配下**にYAMLおよびCSVファイルを配置する。
2. YAML内のCSVパスは **routesからの相対パス** で記述する。
3. ファイルを配置・編集した場合は、以下のように必ず再ビルドを行う。

```bash
colcon build
source install/setup.bash
```

⚠️ **ビルド後の編集は反映されません。** 変更時は再度 `colcon build` を実行してください。

---

## YAML設定ファイル仕様

### 構造概要
YAMLのトップレベルには `blocks` 配列を定義します。  
各要素が1つのブロック（固定または可変）を表し、順に結合して最終ルートを構成します。

```yaml
blocks:
  - { type: fixed, name: 20250920, segment_id: "fixed/waypoint.csv" }
  - { type: variable, name: V1,
      nodes_file: "variable/nodes.csv",
      edges_file: "variable/edges.csv",
      start: "C100", goal: "C102",
      checkpoints: ["C101"] }
```

---

## 固定ブロック (type: fixed)

| キー | 型 | 説明 |
|------|----|------|
| `type` | str | `"fixed"` 固定ブロックを示す |
| `name` | str | ブロック識別名 |
| `segment_id` | str | Waypoint列を定義したCSVファイル（routesからの相対パス） |

**動作仕様**  
- 指定されたCSVをそのままWaypointsとして使用します。  
- 再探索対象外であり、`/update_route` では変更されません。

**例：**
```yaml
- { type: fixed, name: 20250920, segment_id: "fixed/waypoint.csv" }
```

---

## 可変ブロック (type: variable)

| キー | 型 | 説明 |
|------|----|------|
| `type` | str | `"variable"` 可変ブロックを示す |
| `name` | str | ブロック識別名 |
| `nodes_file` | str | グラフノード定義CSV（routesからの相対パス） |
| `edges_file` | str | エッジ定義CSV（routesからの相対パス） |
| `start` | str | ノードID。開始ノードを指定 |
| `goal` | str | ノードID。終了ノードを指定 |
| `checkpoints` | list[str] | 経由ノードIDのリスト（1件以上推奨） |

**動作仕様**  
- `graph_solver` により最短経路を算出します。  
- `/update_route` 呼び出し時には封鎖エッジを除いて再探索します。  
- 固定ブロックに封鎖を指定するとエラーになります。

**例：**
```yaml
- { type: variable, name: V1,
    nodes_file: "variable/nodes.csv",
    edges_file: "variable/edges.csv",
    start: "C100", goal: "C102",
    checkpoints: ["C101"] }
```

---

## CSVファイル仕様

### nodes.csv
| カラム | 型 | 説明 |
|--------|----|------|
| id | str | ノード識別子（ユニーク） |
| lat / lon | float | ノード座標。緯度経度またはx/y座標 |

### edges.csv
| カラム | 型 | 説明 |
|--------|----|------|
| source | str | 出発ノードID |
| target | str | 到着ノードID |
| waypoint_list | str | 該当区間のWaypoint CSVへの相対パス |
| reversible | int | 双方向可否（0または1） |

### waypoint.csv
| カラム | 型 | 説明 |
|--------|----|------|
| x | float | X座標 |
| y | float | Y座標 |
| label | str | オプション。ノードラベルを指定可 |

---

## graph_solverとの関係

- `route_planner` は可変ブロックの内容を `graph_solver.solve_variable_route()` に渡します。
- `graph_solver` は `/tmp/route_solver_<pid>.png` に画像を生成します。
- 生成されたPNGを `bgr8` に変換して `Route.image` に格納します。
- 画像読込に失敗した場合はダミー画像を生成して返します。

---

## ルート構成例

```
routes/
├─ sample_route.yaml
├─ fixed/
│  └─ waypoint.csv
└─ variable/
   ├─ nodes.csv
   ├─ edges.csv
   └─ segments/
      ├─ seg_A.csv
      └─ seg_B.csv
```

複数のブロックを定義することで、固定区間＋迂回区間などを柔軟に構成できます。

---

## サービス一覧

| 名称 | 型 | 概要 |
|------|----|------|
| `/get_route` | `GetRoute.srv` | 経路生成を行い、Routeメッセージを返す |
| `/update_route` | `UpdateRoute.srv` | 封鎖エッジを考慮した再探索を行う |

---

## パラメータ一覧

| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `config_yaml_path` | str | "" | 経路構成YAMLファイル（routes以下） |
| `csv_base_dir` | str | "routes" | CSV格納ディレクトリ |
| `planner_timeout_sec` | float | 5.0 | 呼出タイムアウト（phase1未使用） |

---

## 注意事項

- YAMLおよびCSVは **routes配下に格納してからビルド** すること。  
- **ビルド後の編集は反映されない**ため、変更時は再ビルドが必要です。  
- 固定ブロック封鎖はエラーとなります。  
- `edges.csv` の `reversible` は整数（0/1）のみ使用可能です。  

---

## 将来拡張（phase2以降）

- mission_plannerとの自動リルート連携  
- 複数可変ブロックの動的切替  
- 経路画像および統計情報の可視化拡張
