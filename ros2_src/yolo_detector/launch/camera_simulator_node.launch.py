from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    frame_image_path_arg = DeclareLaunchArgument(
        'frame_image_path', default_value='', description='配信する静止画のパス'
    )
    frame_width_arg = DeclareLaunchArgument(
        'frame_width', default_value='-1', description='リサイズ後の横幅（負値で無効）'
    )
    frame_height_arg = DeclareLaunchArgument(
        'frame_height', default_value='-1', description='リサイズ後の縦幅（負値で無効）'
    )
    frame_ratio_arg = DeclareLaunchArgument(
        'frame_ratio', default_value='10.0', description='配信レート[Hz]'
    )

    camera_simulator_node = Node(
        package='yolo_detector',
        executable='camera_simulator_node',
        name='camera_simulator_node',
        output='screen',
        parameters=[
            {
                'frame_image_path': LaunchConfiguration('frame_image_path'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'frame_ratio': LaunchConfiguration('frame_ratio'),
            }
        ],
    )

    return LaunchDescription(
        [
            frame_image_path_arg,
            frame_width_arg,
            frame_height_arg,
            frame_ratio_arg,
            camera_simulator_node,
        ]
    )
