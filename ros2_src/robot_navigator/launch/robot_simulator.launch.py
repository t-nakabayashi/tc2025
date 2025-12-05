from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/ypspur_ros/odom',
        description='シミュレータが配信するオドメトリトピック',
    )

    odom_topic = LaunchConfiguration('odom_topic')

    simulator_node = Node(
        package='robot_navigator',
        executable='robot_simulator',
        name='robot_simulator',
        output='screen',
        parameters=[
            {
                'cycle_hz': 10.0,
                'publish_tf': True,
                'pose_noise_std_m': 0.0,
                'yaw_noise_std_deg': 0.0,
                'enable_glitch_trigger': True,
                'glitch_trigger_topic': '/amcl_glitch_trigger',
                'glitch_cooldown_sec': 5.0,
                'glitch_wait_after_stop_sec': 5.0,
                'glitch_radius_m': 1.0,
                'glitch_yaw_std_deg': 5.0,
                'glitch_cov_floor_m2': 0.25,
                'glitch_yaw_cov_floor_deg2': 25.0,
                'glitch_linear_stop_threshold': 0.02,
                'glitch_angular_stop_threshold': 0.02,
            }
        ],
        remappings=[
            ('odom', odom_topic),
            ('/odom', odom_topic),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        simulator_node,
    ])
