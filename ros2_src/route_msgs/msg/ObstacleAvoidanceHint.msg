# route_msgs/msg/ObstacleAvoidanceHint.msg
# obstacle_monitor が出力する障害物検知ヒント

std_msgs/Header header

bool front_blocked
float32 left_is_open
float32 right_is_open
