# Waypoint（経路中の1点の「実体」）
# - index: 当該 Route における連番（0起算）。同一labelでも再訪で複数エントリが存在し得る。
# - label: 地図上の地点識別子（文字列）。同一点を再訪しても同じlabelを用いる。
# - pose: 位置・姿勢（geometry_msgs/Pose）。yaw 専用フィールドは用いない。
# - right_open/left_open: 路側開放度など、走行判断に寄与するスカラー（拡張属性）。
# - line_stop/signal_stop/not_skip: 停止・スキップに関する制御フラグ。

int32 index
string label
geometry_msgs/Pose pose

float32 right_open
float32 left_open

bool line_stop
bool signal_stop
bool not_skip

