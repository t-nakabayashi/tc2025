from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('route_follower')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='route_followerノードの既定パラメータファイル',
    )
    arrival_threshold_arg = DeclareLaunchArgument(
        'arrival_threshold', default_value='0.6', description='[m] 到達判定閾値'
    )
    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz', default_value='20.0', description='[Hz] 制御周期'
    )
    resend_interval_arg = DeclareLaunchArgument(
        'resend_interval_sec', default_value='1.0', description='[s] 再送周期'
    )
    start_immediately_arg = DeclareLaunchArgument(
        'start_immediately', default_value='true', description='受信直後に自動開始するか'
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='map', description='全Poseのframe_id'
    )
    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='route_follower', description='ノード名'
    )

    # --- 通信系のリマップ（デフォルト名をlaunch側で提供） ---
    active_route_topic_arg = DeclareLaunchArgument(
        'active_route_topic', default_value='/active_route', description='経路入力トピック名'
    )
    amcl_pose_topic_arg = DeclareLaunchArgument(
        'amcl_pose_topic', default_value='/amcl_pose', description='現在位置Poseトピック名'
    )
    obstacle_hint_topic_arg = DeclareLaunchArgument(
        'obstacle_hint_topic',
        default_value='/obstacle_avoidance_hint',
        description='障害物ヒント入力トピック名',
    )
    manual_start_topic_arg = DeclareLaunchArgument(
        'manual_start_topic', default_value='/manual_start', description='手動スタートトピック名'
    )
    signal_recognition_topic_arg = DeclareLaunchArgument(
        'signal_recognition_topic', default_value='/sig_recog', description='信号判定入力トピック名'
    )
    recog_flag_topic_arg = DeclareLaunchArgument(
        'recog_flag_topic', default_value='/recog_flag', description='信号検出開始フラグ出力トピック名'
    )
    active_target_topic_arg = DeclareLaunchArgument(
        'active_target_topic', default_value='/active_target', description='ターゲットPose出力トピック名'
    )
    follower_state_topic_arg = DeclareLaunchArgument(
        'follower_state_topic', default_value='/follower_state', description='状態出力トピック名'
    )
    report_stuck_service_arg = DeclareLaunchArgument(
        'report_stuck_service', default_value='/report_stuck', description='ReportStuckサービス名'
    )

    arrival_threshold = LaunchConfiguration('arrival_threshold')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    resend_interval_sec = LaunchConfiguration('resend_interval_sec')
    start_immediately = LaunchConfiguration('start_immediately')
    target_frame = LaunchConfiguration('target_frame')
    node_name = LaunchConfiguration('node_name')
    param_file = LaunchConfiguration('param_file')

    active_route_topic = LaunchConfiguration('active_route_topic')
    amcl_pose_topic = LaunchConfiguration('amcl_pose_topic')
    obstacle_hint_topic = LaunchConfiguration('obstacle_hint_topic')
    manual_start_topic = LaunchConfiguration('manual_start_topic')
    signal_recognition_topic = LaunchConfiguration('signal_recognition_topic')
    recog_flag_topic = LaunchConfiguration('recog_flag_topic')
    active_target_topic = LaunchConfiguration('active_target_topic')
    follower_state_topic = LaunchConfiguration('follower_state_topic')
    report_stuck_service = LaunchConfiguration('report_stuck_service')

    route_follower_node = Node(
        package='route_follower',
        executable='route_follower',
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[
            param_file,
            {
                'arrival_threshold': arrival_threshold,
                'control_rate_hz': control_rate_hz,
                'resend_interval_sec': resend_interval_sec,
                'start_immediately': start_immediately,
                'target_frame': target_frame,
            },
        ],
        remappings=[
            ('active_route', active_route_topic),
            ('amcl_pose', amcl_pose_topic),
            ('obstacle_avoidance_hint', obstacle_hint_topic),
            ('manual_start', manual_start_topic),
            ('sig_recog', signal_recognition_topic),
            ('recog_flag', recog_flag_topic),
            ('active_target', active_target_topic),
            ('follower_state', follower_state_topic),
            ('report_stuck', report_stuck_service),
        ],
    )

    return LaunchDescription([
        param_file_arg,
        arrival_threshold_arg,
        control_rate_arg,
        resend_interval_arg,
        start_immediately_arg,
        target_frame_arg,
        node_name_arg,
        active_route_topic_arg,
        amcl_pose_topic_arg,
        obstacle_hint_topic_arg,
        manual_start_topic_arg,
        signal_recognition_topic_arg,
        recog_flag_topic_arg,
        active_target_topic_arg,
        follower_state_topic_arg,
        report_stuck_service_arg,
        route_follower_node,
    ])
