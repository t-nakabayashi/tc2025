# route_manager 観点のルート状態の概要（監視・UI向け）

std_msgs/Header header
int32 route_version
int32 total_waypoints
int32 current_index
string current_label
string status    # "UNSET","ACTIVE","UPDATING","COMPLETED","ERROR"
string message



