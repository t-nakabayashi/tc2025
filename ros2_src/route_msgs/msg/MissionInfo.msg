# ミッションのメタ情報（静的・準静的設定を通知）
# - 例：スタート/ゴール/必須通過群のラベル列、運用パラメータ概要など

std_msgs/Header header
string start_label
string goal_label
string[] checkpoint_labels
string description

