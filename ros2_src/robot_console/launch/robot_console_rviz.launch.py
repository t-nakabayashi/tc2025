"""robot_console 用のRVizビューを起動するLaunchファイル."""

from __future__ import annotations

from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext, *args, **kwargs) -> List[Node]:
    """RViz2ノードを生成する内部関数."""

    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="robot_console_rviz",
            output="screen",
            arguments=["-d", rviz_config],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """RViz2を robot_console 推奨設定で起動する."""

    default_config_path = Path(
        get_package_share_directory("robot_console")
    ) / "rviz" / "robot_console_view.rviz"

    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=str(default_config_path),
        description="RViz設定ファイルパス",
    )

    return LaunchDescription([rviz_arg, OpaqueFunction(function=_launch_setup)])
