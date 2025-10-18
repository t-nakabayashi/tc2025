# route_msgs/msg/FollowerState.msg
# route_follower ノードが現在の追従状態を通知する。
# Phase2にて拡張された最終仕様。

std_msgs/Header header

int32 route_version
string state
int32 current_index

geometry_msgs/Pose current_pose
float32 distance_to_target

int32 avoidance_attempt_count
string last_stagnation_reason

bool front_blocked_majority
float32 hint_left_open_m_median
float32 hint_right_open_m_median

string current_waypoint_label
string next_waypoint_label
