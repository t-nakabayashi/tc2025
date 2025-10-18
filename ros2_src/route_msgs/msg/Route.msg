# 経路全体（配信の中核メッセージ）
# - header.frame_id は "map" を必須とする。
# - version は route_manager が配信するルートの世代管理用。
# - route_image は経路可視化の埋込画像（encoding: "bgr8" 推奨）。
# - waypoints は index 順で格納する（0..N-1）。

std_msgs/Header header
int32 version
float32 total_distance
sensor_msgs/Image route_image
route_msgs/Waypoint[] waypoints
int32 start_index
string start_wp_label