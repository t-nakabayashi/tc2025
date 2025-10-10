from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """LaunchConfigurationを評価してノードに渡す。"""
    start_label = LaunchConfiguration('start_label').perform(context)
    goal_label = LaunchConfiguration('goal_label').perform(context)

    node = Node(
        package='route_manager',
        executable='route_manager',
        name='route_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'start_label': str(start_label),
            'goal_label': str(goal_label),
        }]
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_label', default_value='', description='始点ラベル'),
        DeclareLaunchArgument('goal_label', default_value='', description='終点ラベル'),
        OpaqueFunction(function=launch_setup),
    ])
