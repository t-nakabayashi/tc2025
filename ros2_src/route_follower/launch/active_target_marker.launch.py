"""/active_target Marker を起動するLaunchファイル."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """active_target_markerノードを起動するLaunchDescriptionを生成する."""

    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="active_target_marker",
        description="起動するノード名",
    )
    active_target_topic_arg = DeclareLaunchArgument(
        "active_target_topic",
        default_value="/active_target",
        description="入力PoseStampedトピック名",
    )
    marker_topic_arg = DeclareLaunchArgument(
        "marker_topic",
        default_value="/active_target/marker",
        description="出力Markerトピック名",
    )

    node_name = LaunchConfiguration("node_name")
    active_target_topic = LaunchConfiguration("active_target_topic")
    marker_topic = LaunchConfiguration("marker_topic")

    marker_node = Node(
        package="route_follower",
        executable="active_target_marker",
        name=node_name,
        output="screen",
        emulate_tty=True,
        remappings=[
            ("active_target", active_target_topic),
            ("active_target/marker", marker_topic),
        ],
    )

    return LaunchDescription([
        node_name_arg,
        active_target_topic_arg,
        marker_topic_arg,
        marker_node,
    ])
