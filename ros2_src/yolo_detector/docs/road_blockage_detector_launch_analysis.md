# 日本語文書
# road_blockage_detector launch 時の挙動差分調査メモ

## 1. 事象の概要
- road_blockage_detector は自己位置を取得できないと判定処理をスキップする実装のため、起動直後に `/amcl_pose` が未受信だと仮封鎖解除や多重検知抑止が動かずに見えることがあった。
- 過去には TF とシミュレーション時間のずれが主因だったが、自己位置源を `/amcl_pose` のみに絞り、launch から `use_sim_time` を外したことで時間軸の不整合を排除した。

## 2. 確認事項
### 2.1 自己位置取得の前提
- ノードは Detection2DArray のヘッダー時刻をもとに `/amcl_pose` のキャッシュを参照し、取得できない場合は警告を出して処理を終了する。検知時刻と `/amcl_pose` のヘッダーに 3 秒以上の差がある場合にも警告を発行する。【F:yolo_detector/yolo_detector/road_blockage_detector_node.py†L97-L125】【F:yolo_detector/yolo_detector/road_blockage_detector_node.py†L152-L176】
- `/amcl_pose` の取得と時刻管理に一本化したため、TF ルックアップ失敗が原因で処理が止まるケースはなくなった。

### 2.2 launch の時間設定
- `use_sim_time` の引数を `road_blockage_detector.launch.py` と統合版 launch から削除し、ノード側でも同パラメータを参照しないようにした。これにより `/clock` 有無に関わらず `/amcl_pose` に合わせた時間軸で動作する。【F:yolo_detector/launch/road_blockage_detector.launch.py†L1-L23】【F:yolo_detector/launch/yolo_with_road_blockage.launch.py†L1-L46】【F:yolo_detector/launch/yolo_ncnn_with_road_blockage.launch.py†L1-L49】

## 3. 対応方針と修正内容
- 自己位置は `/amcl_pose` のみを使用し、Detection と `/amcl_pose` のヘッダー時刻差が 3 秒以上なら警告ログを出す。
- launch ファイルでは `use_sim_time` を受け付けず、パラメータファイルの指定だけで起動する構成とする。
