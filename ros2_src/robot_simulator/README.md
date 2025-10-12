# robot_simulator パッケージ README（phase1）

## 概要
`robot_simulator` は、`/active_target`（`geometry_msgs/PoseStamped`）を追従する**簡易ロボット運動シミュレータ**です。既定では **5 km/h の等速直線**で移動し、**100 ms 周期**で自己位置を `/amcl_pose`（`PoseStamped`）として配信します。初回は「**target の向きの反対側へ 5 m**」離れた位置から開始し、**極小解停止**（距離≤1 m かつ次ステップで距離が増える場合に停止）を備えます。必要に応じて `map→base_link` の `/tf` も配信できます。

---

## 主な機能
- `/active_target` を購読し、ターゲット方向へ**等速直線運動**。初回受信時に**背後5 m**で初期化し、姿勢は常にターゲット方向を向く。
- **極小解停止**：target が不変かつ距離≤`stop_radius_m`（既定1 m）で、次の一歩で遠ざかる場合は停止。ターゲットが更新されれば即再追従。
- **100 ms 周期**で `/amcl_pose` を配信。任意で `/tf`（`map→base_link`）を配信可能。
- **QoS**：`/active_target` は **RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)** を採用（最新1件を確実に取得）。

---

## 外部I/F

### 購読トピック
| 名称 | 型 | 説明 | QoS |
|---|---|---|---|
| `/active_target` | `geometry_msgs/PoseStamped` | 追従する目標Pose。`header.frame_id="map"` を前提（不一致は警告のみで処理継続）。 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1) |

### 配信トピック
| 名称 | 型 | 説明 | QoS |
|---|---|---|---|
| `/amcl_pose` | `geometry_msgs/PoseStamped` | シミュレータの自己位置。100 ms 周期で発行。`header.frame_id="map"`。 | RELIABLE / VOLATILE / KEEP_LAST(1) |
| （任意）`/tf` | `tf2_msgs/TFMessage` | `map→base_link` の Transform。`publish_tf=true` のときのみ配信。 | RELIABLE / VOLATILE / KEEP_LAST(10) |

> 実装は上記仕様に沿っており、frame 不一致時は警告ログのみで処理継続します（座標変換は行いません）。

---

## パラメータ
| 名称 | 型 | 既定値 | 説明 |
|---|---|---:|---|
| `speed_kmph` | double | 5.0 | 巡航速度 [km/h]（内部で m/s に換算） |
| `timer_period_ms` | int | 100 | 更新・配信周期 [ms] |
| `frame_id` | string | `map` | 出力フレーム |
| `child_frame_id` | string | `base_link` | `/tf`の child frame |
| `publish_tf` | bool | false | `/tf` を配信するか |
| `init_offset_m` | double | 5.0 | 初期位置を target の**反対側**に下げる距離 |
| `stop_radius_m` | double | 1.0 | 極小解停止の距離半径 |
| `pose_cov_diag` | double[6] | `[0.02,0.02,0.04,0,0,(2°)^2]` | x,y,z,roll,pitch,yaw の**分散** |
| `noise_pos_std_m` | double | 0.0 | 位置ノイズ標準偏差 [m]（出力の見かけだけに付与） |
| `noise_yaw_std_deg` | double | 0.0 | ヨー角ノイズ標準偏差 [deg]（同上） |
| `log_debug` | bool | false | デバッグログ出力の有効化 |

---

## QoS 設定（要点）
- `/active_target`：**RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)**
- `/amcl_pose`：RELIABLE / VOLATILE / KEEP_LAST(1)
- `/tf`：RELIABLE / VOLATILE / KEEP_LAST(10)

---

## 起動方法

### 1) 単体起動（launch）
```bash
ros2 launch robot_simulator robot_simulator.launch.py
```
代表的な引数を変更する例：
```bash
ros2 launch robot_simulator robot_simulator.launch.py   speed_kmph:=5.0 timer_period_ms:=100 frame_id:=map   publish_tf:=false init_offset_m:=5.0 stop_radius_m:=1.0   noise_pos_std_m:=0.0 noise_yaw_std_deg:=0.0 log_debug:=false
```

### 2) 実行ファイル直起動（デバッグ用途）
```bash
ros2 run robot_simulator robot_simulator
```

---

## 動作確認手順（例）

1. **シミュレータ起動**
   ```bash
   ros2 launch robot_simulator robot_simulator.launch.py
   ```

2. **target の投入**（例：固定点を 1 件パブリッシュ）
   ```bash
   ros2 topic pub /active_target geometry_msgs/PoseStamped    "{header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 0.0},    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -1
   ```

3. **自己位置の監視**
   ```bash
   ros2 topic echo /amcl_pose
   ```

4. **/tf の配信確認**（`publish_tf:=true` で起動した場合）
   ```bash
   ros2 topic echo /tf
   ```

---

## 仕様ノート・挙動の要点
- **常に target 方向を向く**：停止中でもヨー角は target 方向を向くよう更新。
- **極小解停止**：`距離≤stop_radius_m` かつ「次の一歩で距離が増える」場合に停止。target が更新されれば再び追従。
- **frame の扱い**：`/active_target.header.frame_id` が想定と異なる場合は**警告**し、座標変換は行わない。
- **ノイズ付与**：ノイズは**出力にのみ**付与され、内部状態はノイズなしで積分。

---

## 既知の注意点・トラブルシュート
- **`/amcl_pose` が出ない**：`/active_target` 未受信の間は配信しません。
- **速度や周期の不正**：`speed_kmph ≤ 0` や `timer_period_ms ≤ 0` は**起動時に例外**。
- **TF 競合**：実機の TF と競合しうるため、`publish_tf` は既定で **false**。

---

## 他ノードとの関係（試験系での想定）
- `route_follower` の `/active_target` 出力に追従し、`/amcl_pose` を供給する**試験用ノード**。

---

## ライセンス・作者
- 本 README は添付の詳細設計・実装・launch に準拠して作成されています。
