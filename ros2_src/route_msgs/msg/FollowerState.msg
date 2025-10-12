# 走行ノードの状態
# - status は下記の文字列のみを許容する。
#   "IDLE", "RUNNING", "WAITING_STOP", "FINISHED",
#   "AVOIDING_OBSTACLE", "WAITING_REPLAN", "REROUTING", "ERROR"

std_msgs/Header header
int32 route_version
int32 current_index
string current_waypoint_label

string status
geometry_msgs/Pose current_pose
float32 distance_to_target

string next_waypoint_label

