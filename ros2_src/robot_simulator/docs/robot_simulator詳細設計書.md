# robot_simulator ノード 詳細設計書

（/active_target 追従・5km/h 等速直線・100ms周期・極小解停止・/amcl_pose 出力）

本書は、2ノード構成（`test_controller`＋`robot_simulator`）における **robot_simulator** の唯一の実装基準です。
`route_follower` が発行する `/active_target` に対して、簡易ロボット運動を再現し、`/amcl_pose` を配信します。

---

## 0. 目的・責務・非責務

* **目的**

  * `/active_target` へ**等速直線運動（5km/h）で追従**するロボットをシミュレートし、`/amcl_pose` を **100ms周期**で配信。
  * **初期位置**は「最初に受け取った active_target の**進行方向と反対（180°）側**へ 5m 離れた点」。
  * **極小解停止**：active_target が不変、かつ距離 ≤1m、次ステップで距離が増えるなら停止。

* **責務**

  * `/active_target` 購読、運動モデル計算、`/amcl_pose`（必要なら `/tf`）配信。
  * 単純な Pose 共分散の付与（定数 or パラメータ）。

* **非責務**

  * 経路生成や追従制御の判断。
  * センサや障害物回避の再現。
  * follower/manager/planner のモック機能（test_controller側の責務）。

---

## 1. I/F 仕様

### 1.1 Subscribe

| Topic            | Type                        | QoS                                           | 意味                                           |
| ---------------- | --------------------------- | --------------------------------------------- | -------------------------------------------- |
| `/active_target` | `geometry_msgs/PoseStamped` | **RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)** | 追従すべき目標Pose。`header.frame_id` は `"map"` を前提。 |

> 受信時に**最新1件**のみ保持。frame が `"map"` 以外の場合は警告ログを出し、**処理は継続**（座標変換は行わない）。

### 1.2 Publish

| Topic        | Type                                      | QoS                                    | 意味                                       |
| ------------ | ----------------------------------------- | -------------------------------------- | ---------------------------------------- |
| `/amcl_pose` | `geometry_msgs/PoseStamped` | **RELIABLE / VOLATILE / KEEP_LAST(1)** | 100ms周期で配信する自己位置。`header.frame_id="map"` |
| （任意）`/tf`    | `tf2_msgs/TFMessage`                      | RELIABLE / VOLATILE / KEEP_LAST(10)    | `map->base_link`（必要時のみ。既定は無効）            |

---

## 2. パラメータ

| 名称                  | 型         | 既定値                             | 説明                                |
| ------------------- | --------- | ------------------------------- | --------------------------------- |
| `speed_kmph`        | double    | `5.0`                           | 巡航速度 [km/h]。内部で m/s に換算（=1.3889）  |
| `timer_period_ms`   | int       | `100`                           | 配信周期（運動更新周期）                      |
| `frame_id`          | string    | `"map"`                         | 出力の座標系                            |
| `child_frame_id`    | string    | `"base_link"`                   | `/tf` 出力時の子フレーム                   |
| `publish_tf`        | bool      | `false`                         | `/tf` を出力するか                      |
| `init_offset_m`     | double    | `5.0`                           | 初期位置の**逆方向**オフセット[m]              |
| `stop_radius_m`     | double    | `1.0`                           | 極小解停止を判定する距離[m]                   |
| `pose_cov_diag`     | double[6] | `[0.02,0.02,0.04, 0,0, (2°)^2]` | x,y,z, r,p,yaw の分散（z,r,p は0固定でも可） |
| `noise_pos_std_m`   | double    | `0.0`                           | 位置ノイズ（ガウス）標準偏差[m]（既定OFF）          |
| `noise_yaw_std_deg` | double    | `0.0`                           | yaw ノイズ標準偏差[deg]（既定OFF）           |
| `log_debug`         | bool      | `false`                         | デバッグ出力ON                          |

---

## 3. 運動モデル仕様

### 3.1 初期位置と姿勢

* **初回**に `/active_target` を受信した時：

  1. target の**向き**を quaternion → yaw に変換（`yaw_t`）。
  2. target の**進行方向ベクトル** `v_t = (cos(yaw_t), sin(yaw_t))` を求める。
  3. **初期位置** `P0 = T.position - init_offset_m * v_t`。
  4. **初期姿勢**は「**targetへ向く**」：`yaw_0 = atan2(T.y - P0.y, T.x - P0.x)`。

> つまり、ターゲットの「向き」から 180° 反対へ 5m 下がった背後から、**ターゲット方向を向いて**スタート。

### 3.2 等速直線運動

* 速度 `v = speed_kmph * 1000/3600 [m/s]`。
* 更新周期 `dt = timer_period_ms / 1000`。
* 現在位置 `P=(x,y)`、目標 `T=(x_t,y_t)`、距離 `d = ||T-P||`。
* **単位方向** `u = (T-P)/d`（`d>0` のとき）。
* **候補更新**：`P' = P + v*dt * u`。
* **姿勢**：`yaw = atan2(u_y, u_x)`（停止時も **常に target を向く**）。

### 3.3 極小解停止

* 判定条件（**targetが不変**かつ …）：

  * `d <= stop_radius_m` **かつ** `||T - P'|| > d`（次の一歩で遠ざかる）。
* 条件成立時：**停止**（`P'=P`、yaw は target 方向を維持）。

### 3.4 target 変更時の挙動

* `/active_target` が更新されたら即座に `u` を再計算。
* 停止中でも target が変われば追従を再開。

### 3.5 ノイズ（任意）

* `noise_pos_std_m > 0` なら、各更新で `N(0,σ)` を x,y に加算。
* `noise_yaw_std_deg > 0` なら、yaw に `N(0,σ_deg)` を加算。
* ノイズは `/amcl_pose` へは反映するが**内部の運動状態には加算しない**（見かけ上の揺れ）。

---

## 4. メッセージ詳細

### 4.1 `/amcl_pose`（PoseStamped）

* `header.frame_id = frame_id`（既定 `"map"`）
* `pose.position = (x, y, 0)`
* `pose.orientation = yaw → quaternion`

### 4.2 `/tf`（任意）

* `TransformStamped`

  * `header.frame_id = frame_id`
  * `child_frame_id = child_frame_id`（`"base_link"`）
  * `translation = (x,y,0)`、`rotation = yaw→quat`
* 既定 `publish_tf=false`（重複TFの衝突回避のため）。

---

## 5. 例外・境界条件

* **active_target 未受信**：`/amcl_pose` を出さない（ログ INFO）。
* **active_target の frame 不一致**：警告ログを出し、そのまま数値を使用（座標変換はしない）。
* **距離 `d=0`**：`u` を (0,0) とし、**停止**（yaw は前回値維持）。
* **速度0**（不正設定）：エラーで起動拒否。
* **タイマ遅延**：処理が遅延しても、1サイクル内で追加ループは行わず次周期へ。

---

## 6. 内部構成（クラス）

```
robot_simulator/
 ├─ robot_sim_node.py       # RobotSimulatorNode(rclpy.Node)
 └─ kinematics.py           # yaw<->quat, 距離計算などのユーティリティ
```

### 6.1 `RobotSimulatorNode`

* **購読**：`/active_target`（最新を保持、更新時刻を記録）。
* **タイマ**：`timer_period_ms` で運動更新→ `/amcl_pose` publish。
* **主要フィールド**：

  * `self.have_target`（初回受信済みか）
  * `self.P = (x,y)` 現在位置
  * `self.yaw` 現在姿勢
  * `self.last_target`（PoseStamped）
  * `self.v`（m/s）
* **メソッド**：

  * `_on_target(msg)`：target 更新
  * `_init_from_target()`：初期位置計算
  * `_step(dt)`：等速一歩進む（極小解停止含む）
  * `_pub_amcl_pose()`、`_pub_tf_if_enabled()`

---

## 7. 擬似コード（要点）

```python
class RobotSimulatorNode(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        # params
        self.v = self.get_parameter_or('speed_kmph', 5.0).get_parameter_value().double_value * 1000.0/3600.0
        self.dt = self.get_parameter_or('timer_period_ms', 100).integer_value / 1000.0
        self.stop_r = self.get_parameter_or('stop_radius_m', 1.0).double_value
        self.init_offset = self.get_parameter_or('init_offset_m', 5.0).double_value
        self.frame_id = self.get_parameter_or('frame_id', 'map').string_value
        self.publish_tf = self.get_parameter_or('publish_tf', False).bool_value

        self.sub = self.create_subscription(PoseStamped, '/active_target', self._on_target,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_pose = self.create_publisher(PoseStamped, '/amcl_pose', 10)

        self.have_target = False
        self.P = None; self.yaw = 0.0
        self.timer = self.create_timer(self.dt, self._on_timer)

    def _on_target(self, msg: PoseStamped):
        self.last_target = msg
        if not self.have_target:
            # 初期化
            yaw_t = quat_to_yaw(msg.pose.orientation)
            vtx, vty = math.cos(yaw_t), math.sin(yaw_t)
            px = msg.pose.position.x - self.init_offset * vtx
            py = msg.pose.position.y - self.init_offset * vty
            self.P = np.array([px, py], dtype=float)
            self.yaw = math.atan2(msg.pose.position.y - py, msg.pose.position.x - px)
            self.have_target = True

    def _on_timer(self):
        if not self.have_target:
            return
        T = np.array([self.last_target.pose.position.x, self.last_target.pose.position.y], dtype=float)
        dvec = T - self.P
        d = np.linalg.norm(dvec)
        if d > 1e-6:
            u = dvec / d
        else:
            u = np.array([0.0, 0.0])

        step = self.v * self.dt
        P_next = self.P + step * u

        # 極小解停止
        if d <= self.stop_r and np.linalg.norm(T - P_next) > d:
            P_next = self.P  # stay

        self.P = P_next
        if np.linalg.norm(u) > 0:
            self.yaw = math.atan2(u[1], u[0])

        self._pub_amcl_pose()

    def _pub_amcl_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = float(self.P[0])
        msg.pose.pose.position.y = float(self.P[1])
        msg.pose.pose.orientation = yaw_to_quat(self.yaw)
        # covariance 填め込み（省略）
        self.pub_pose.publish(msg)
```

---

## 8. タイミング・性能

* **配信周期**：10Hz（可変）。
* **遅延目標**：timerコールバック→publish まで < 5ms（デスクトップ環境想定）。
* **CPU負荷**：軽微（単純演算）。
* **スレッド**：rclpy のコールバックスレッドのみ。追加スレッド不要。

---

## 9. テスト項目（最小）

1. **初期化**：最初の active_target 受信で背後 5m に初期化される。
2. **直線追従**：等速で target に近づく（距離の単調減少）。
3. **極小解停止**：距離 ≤1m かつ次ステップで増加 → 停止。
4. **target 更新**：目標が変われば即座に方向転換。
5. **QoS**：/active_target が TL の時に最新1件を確実に取得。
6. **frame**：frame_id が `"map"` 出力で固定される。
7. **ノイズ**：パラメータONで出力に揺らぎが乗る（内部状態は安定）。
8. **/tf**：publish_tf=true で `map->base_link` が配信される。

---

## 10. launch 例

```python
# robot_simulator.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_system',
            executable='robot_simulator',
            name='robot_simulator',
            output='screen',
            parameters=[{
                'speed_kmph': 5.0,
                'timer_period_ms': 100,
                'frame_id': 'map',
                'publish_tf': False
            }]
        )
    ])
```

> `test_controller` からも LaunchManager 経由で同等パラメータを注入可能。

---

## 11. 運用メモ（test_controller との連携）

* follower 単体試験：

  * test_controller（managerモックONで `/active_route` 配信）
  * follower（試験対象）
  * **robot_simulator（本ノード）**
    → follower の `/active_target` に合わせて `/amcl_pose` を発行し続ける。

* 全系監視：

  * manager/planner/follower 実体 + robot_simulator
  * test_controller は **監視ダッシュボード**として起動（イベント発行可）。

---

## 12. 既知の注意点

* フレーム変換は実装しない（`/active_target` は `"map"` 前提）。
* `/tf` は既定OFF（実システムのTFと競合しうるため）。
* 速度や停止半径は試験用にパラメータ化しており、ケースに応じて調整可能。


