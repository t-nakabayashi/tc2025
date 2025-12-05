from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('robot_navigator')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])
    navigator_launch = PathJoinSubstitution([pkg_share, 'launch', 'robot_navigator.launch.py'])

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='robot_navigator ノードの既定パラメータファイル',
    )
    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='robot_navigator', description='robot_navigator のノード名'
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan', description='LaserScan 入力トピック'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/ypspur_ros/odom',
        description='オドメトリ入力トピック',
    )
    amcl_pose_topic_arg = DeclareLaunchArgument(
        'amcl_pose_topic', default_value='/amcl_pose', description='現在姿勢入力トピック'
    )
    active_target_topic_arg = DeclareLaunchArgument(
        'active_target_topic',
        default_value='/active_target',
        description='PoseStamped 目標入力トピック',
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel', description='Twist 出力トピック'
    )
    marker_topic_arg = DeclareLaunchArgument(
        'marker_topic',
        default_value='/direction_marker',
        description='可視化 Marker 出力トピック',
    )
    obstacle_hint_topic_arg = DeclareLaunchArgument(
        'obstacle_hint_topic',
        default_value='/obstacle_avoidance_hint',
        description='障害物ヒント入力トピック',
    )

    param_file = LaunchConfiguration('param_file')
    node_name = LaunchConfiguration('node_name')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    amcl_pose_topic = LaunchConfiguration('amcl_pose_topic')
    active_target_topic = LaunchConfiguration('active_target_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    marker_topic = LaunchConfiguration('marker_topic')
    obstacle_hint_topic = LaunchConfiguration('obstacle_hint_topic')

    navigator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigator_launch),
        launch_arguments={
            'param_file': param_file,
            'node_name': node_name,
            'scan_topic': scan_topic,
            'odom_topic': odom_topic,
            'amcl_pose_topic': amcl_pose_topic,
            'active_target_topic': active_target_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'marker_topic': marker_topic,
            'obstacle_hint_topic': obstacle_hint_topic,
        }.items(),
    )

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
                'glitch_radius_min_m': 2.0,
                'glitch_radius_max_m': 3.0,
                'glitch_yaw_min_deg': 60.0,
                'glitch_yaw_max_deg': 90.0,
                'glitch_cov_floor_m2': 0.25,
                'glitch_yaw_cov_floor_deg2': 25.0,
                'glitch_linear_stop_threshold': 0.02,
                'glitch_angular_stop_threshold': 0.02,
            }
        ],
        remappings=[
            ('/odom', odom_topic),
        ],
    )

    return LaunchDescription(
        [
            param_file_arg,
            node_name_arg,
            scan_topic_arg,
            odom_topic_arg,
            amcl_pose_topic_arg,
            active_target_topic_arg,
            cmd_vel_topic_arg,
            marker_topic_arg,
            obstacle_hint_topic_arg,
            navigator_include,
            simulator_node,
        ]
    )
