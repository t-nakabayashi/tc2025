# 週末組2024 README
このリポジトリは、週末組2024のつくばチャレンジリポジトリです。

## 環境設定

### 依存パッケージ
このパッケージはROS1 noetic, python3で動作し、以下のROSパッケージを必要とします。
パッケージのインストール方法を各リポジトリを参考にしてください。

~~~
tf
ypspur_ros
urg_node
livox_ros_driver2
velodyne_pointcloud
fast_lio
map_server
mcl_3dl
trajectory_tracker
move_base
~~~

### 依存パッケージの導入方法
#### gpt接続に関わるツールのインストール

~~~
pip install python-dotenv openai requests
~~~

/scripts/.env.sampleを/scripts/.envにリネームし、各自で取得したAPIキーを入れる

#### ypspurのインストール
https://github.com/openspur/yp-spur/blob/master/doc/README.ja.md

#### ypspur rosのインストール
https://github.com/openspur/ypspur_ros


#### urg nodeのインストール
~~~
sudo apt install ros-noetic-urg-node
~~~

#### install livox SDK and livox ros driver2

~~~
# install livox SDK2
cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install

# install livox ros driver2
cd ~/(your workspace)/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git 
cd ../
catkin_make
~~~


#### insall velodyne and move base
~~~
sudo apt install ros-noetic-velodyne
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-map-server

~~~


#### install tf2 and mcl-3dl 
~~~
sudo apt install ros-noetic-geometry2
cd ~/(your workspace)/src
git clone https://github.com/at-wat/mcl_3dl.git
git clone https://github.com/at-wat/mcl_3dl_msgs.git
~~~


#### install livox ros driver1 and fastlio

catkin_makeが失敗する場合は何回かcatkin_makeを繰り返す

~~~
# install livox ros direver 1
cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install

cd ~/(your workspace)/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git 
cd ../
catkin_make

#FAST-LIOの導入
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
~~~




### 起動方法
各launchファイルはそれぞれ異なる機能を担当しています。

用途に応じて適切なファイルを選択し、以下のコマンドで起動してください。

####  1 ロボット・センサの起動 
説明: ロボット、センサの起動を行います。各センサーフレームの静的変換を設定し、基本的な通信設定やポート、TFブロードキャストを行います。
基本的にrosbagはこのlaunchファイルを実行して記録されているため、rosbag playを用いる場合には本プログラムの起動は不要です。

##### 1_init_robot_tc2024.launch
説明: ロボット、センサの起動を行います。各センサーフレームの静的変換を設定し、基本的な通信設定やポート、TFブロードキャストを行います。
##### 主なノード:
tf_base2mid360, tf_base2laserなどのtf変換ノード


ypspur_ros: ポートとパラメータファイルを指定し、通信を設定


urg_node: レーザーセンサの設定およびリマップ


usb_cam: カメラの設定および起動

### 2 地図・ウェイポイントの自動生成
地図、ウェイポイントを自動生成します。2-1,2-2,2-3と順に起動します。


waypoint_file等は環境に合わせて変更してください。FASTLIOのパッケージがFASTLIO/PCD直下にポイントクラウドデータを保存するので、デフォルトはおももにFASTLIO/PCD直下のファイルが設定されています。




#### 2-1_mapping_fastlio.launch
説明: Fast LIOを利用して、ロボットの周囲をマッピングします。事前に設定されたパラメータファイルやWaypointsを使用します。
##### 主なノード:
fastlio_mapping: マッピングのためのLiDARデータ処理


rviz: ビジュアライゼーションのための設定


amcl_pos_reader_lio: AMCL位置のデータ取得


#### 2-2_pcd_cleaning.launch
説明: 3Dマップデータのクリーニングを行います。指定したPCDファイルを読み込み、ノイズの削減や無駄な点群の削除を実施します。
##### 主なノード:
del_point: PCDファイルのノイズ削減およびクリーニング処理


#### 2-3_pcd_to_2dmap.launch
説明: クリーニング後の3D点群を2Dマップ画像に変換します。2Dナビゲーションやローカリゼーションに利用するマップデータを生成します。
##### 主なノード:
pointcloud_to_map: 点群データを2D画像に変換

### 3 自己位置推定
Lidarデータをもとに自己位置推定を行います。


waypoint_file等は環境に合わせて変更してください。また、pcd_fileとして、$(find fast_lio)/PCD/scans_light_noground.pcdが指定されていますが、通常は2-2で生成されるscans_light.pcdを指定してください。cloudcompareのSCFフィルタを適用することで地面成分を除去することで自己位置推定の失敗確率が下がるため、自己位置推定に失敗するようであれば地面成分の除去を実施し、nogroundファイルを作成することを推奨します。

rosbagの情報をもとに自己位置推定をする場合は、以下のパラメータを設定の上、rosbag play XXX.bag --clockとしてください

~~~
rosparam set /use_sim_time true
~~~

#### 3_locarization.launch
説明: ローカリゼーション処理の設定。MCL 3DLやAMCLを利用し、地図上でロボットの位置を追跡します。

##### 主なノード:
mcl_3dl: MCL 3DLアルゴリズムを使用したローカリゼーション


move_base: ナビゲーションパラメータとコストマップを読み込む


clear_localmap: ローカルコストマップの定期クリア


marge_lidar_data: LiDARデータ統合スクリプト

### 4 動作計画
自己位置推定結果とウェイポイントをもとに動作計画を実施します。


##### 4_run.launch
説明: Waypointマネージャーとナビゲーションノードを起動し、ウェイポイントに従った自動運転を開始します。
##### 主なノード:
waypoint_manager: ウェイポイントの管理と経路追従のための初期化


navigator: 配信されたウェイポイントに従い移動するナビゲーション制御

## ライセンス
このプロジェクトはMIT Licenseのもとで公開されています。