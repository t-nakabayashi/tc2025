from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch引数の宣言 ---
    arrival_threshold_arg = DeclareLaunchArgument(
        "arrival_threshold", default_value="0.6", description="[m] 到達判定閾値"
    )
    control_rate_arg = DeclareLaunchArgument(
        "control_rate_hz", default_value="20.0", description="[Hz] 制御周期"
    )
    resend_interval_arg = DeclareLaunchArgument(
        "resend_interval_sec", default_value="1.0", description="[s] 再送周期"
    )
    start_immediately_arg = DeclareLaunchArgument(
        "start_immediately", default_value="true", description="受信直後に自動開始するか"
    )
    target_frame_arg = DeclareLaunchArgument(
        "target_frame", default_value="map", description="全Poseのframe_id"
    )
    node_name_arg = DeclareLaunchArgument(
        "node_name", default_value="route_follower", description="ノード名"
    )

    # --- トピック名をlaunch引数として可変に ---
    active_route_topic_arg = DeclareLaunchArgument(
        "active_route_topic", default_value="/active_route", description="経路入力トピック名"
    )
    current_pose_topic_arg = DeclareLaunchArgument(
        "current_pose_topic", default_value="/amcl_pose", description="現在位置Pose購読トピック名"
    )
    active_target_topic_arg = DeclareLaunchArgument(
        "active_target_topic", default_value="/active_target", description="出力ターゲットPoseトピック名"
    )

    # --- launch引数の読み込み ---
    arrival_threshold = LaunchConfiguration("arrival_threshold")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    resend_interval_sec = LaunchConfiguration("resend_interval_sec")
    start_immediately = LaunchConfiguration("start_immediately")
    target_frame = LaunchConfiguration("target_frame")
    node_name = LaunchConfiguration("node_name")

    active_route_topic = LaunchConfiguration("active_route_topic")
    current_pose_topic = LaunchConfiguration("current_pose_topic")
    active_target_topic = LaunchConfiguration("active_target_topic")

    # --- ノード定義 ---
    route_follower_node = Node(
        package="route_follower",
        executable="route_follower",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "arrival_threshold": arrival_threshold,
            "control_rate_hz": control_rate_hz,
            "resend_interval_sec": resend_interval_sec,
            "start_immediately": start_immediately,
            "target_frame": target_frame,
        }],
        remappings=[
            ("/active_route", active_route_topic),
            ("/amcl_pose", current_pose_topic),
            ("/active_target", active_target_topic),
        ],
    )

    return LaunchDescription([
        arrival_threshold_arg,
        control_rate_arg,
        resend_interval_arg,
        start_immediately_arg,
        target_frame_arg,
        node_name_arg,
        active_route_topic_arg,
        current_pose_topic_arg,
        active_target_topic_arg,
        route_follower_node
    ])

