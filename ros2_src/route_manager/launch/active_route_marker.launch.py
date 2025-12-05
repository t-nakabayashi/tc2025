"""/active_route MarkerArray 配信ノードを起動するLaunchファイル."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """active_route_markerノードを起動するLaunchDescriptionを生成する."""

    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="active_route_marker",
        description="起動するノード名",
    )
    active_route_topic_arg = DeclareLaunchArgument(
        "active_route_topic",
        default_value="/active_route",
        description="入力Routeトピック名",
    )
    marker_topic_arg = DeclareLaunchArgument(
        "marker_topic",
        default_value="/active_route/markers",
        description="出力MarkerArrayトピック名",
    )

    node_name = LaunchConfiguration("node_name")
    active_route_topic = LaunchConfiguration("active_route_topic")
    marker_topic = LaunchConfiguration("marker_topic")

    marker_node = Node(
        package="route_manager",
        executable="active_route_marker",
        name=node_name,
        output="screen",
        emulate_tty=True,
        remappings=[
            ("active_route", active_route_topic),
            ("active_route/markers", marker_topic),
        ],
    )

    return LaunchDescription([
        node_name_arg,
        active_route_topic_arg,
        marker_topic_arg,
        marker_node,
    ])
