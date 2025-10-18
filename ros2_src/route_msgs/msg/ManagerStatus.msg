# route_manager が外部へ公開する状態情報（Phase2仕様）

std_msgs/Header header
string state        # "idle", "running", "updating_route", "holding"
string decision     # "none", "replan", "skip", "failed"
string last_cause   # "stagnation", "no_hint", "no_space", "avoidance_failed", ...
uint32 route_version
