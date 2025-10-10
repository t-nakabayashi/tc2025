# tc2025

## シンボリックリンクの貼り方

### ROS1 (catkin)

1. 既存のリンクやディレクトリがあれば削除します。
   ```bash
   rm -rf ~/catkin_ws/src/tc2025
   ```
2. このリポジトリの `ros1_src` を `~/catkin_ws/src/tc2025` へリンクします。
   ```bash
   ln -s /home/(user name)/ros/tc2025/ros1_src ~/catkin_ws/src/tc2025
   ```

### ROS2 (colcon)

1. 既存のリンクやディレクトリがあれば削除します。（※削除して問題ないかよく確認してから実施してください）
   ```bash
   rm -rf ~/ros2_ws/src
   ```
2. このリポジトリの `ros2_src` を `~/ros2_ws/src` へリンクします。
   ```bash
   ln -s /home/(user name)/ros/tc2025/ros2_src ~/ros2_ws/src
   ```

## 使い方

### ROS1 パッケージ

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tc2025 <launchファイル名>
```

### ROS2 パッケージ

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run <パッケージ名> <ノード名>
```

パッケージや各種設定ファイル、スクリプトは今後 `tc2025` / `tc2025_ros2` 配下に追加していきます。
