from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- パラメータ引数宣言 ---
    speed_kmph_arg = DeclareLaunchArgument(
        'speed_kmph', default_value='5.0', description='ロボット速度 [km/h]'
    )
    timer_period_ms_arg = DeclareLaunchArgument(
        'timer_period_ms', default_value='100', description='タイマー周期 [ms]'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='map', description='出力 frame_id'
    )
    child_frame_id_arg = DeclareLaunchArgument(
        'child_frame_id', default_value='base_link', description='/tf の child frame_id'
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='false', description='/tf を配信するか'
    )
    init_offset_m_arg = DeclareLaunchArgument(
        'init_offset_m', default_value='5.0', description='初期オフセット距離 [m]'
    )
    stop_radius_m_arg = DeclareLaunchArgument(
        'stop_radius_m', default_value='1.0', description='極小解停止の半径 [m]'
    )
    noise_pos_std_m_arg = DeclareLaunchArgument(
        'noise_pos_std_m', default_value='0.0', description='位置ノイズ σ[m]'
    )
    noise_yaw_std_deg_arg = DeclareLaunchArgument(
        'noise_yaw_std_deg', default_value='0.0', description='ヨー角ノイズ σ[deg]'
    )
    log_debug_arg = DeclareLaunchArgument(
        'log_debug', default_value='false', description='デバッグログ出力を有効にするか'
    )

    # --- LaunchConfiguration取得 ---
    speed_kmph = LaunchConfiguration('speed_kmph')
    timer_period_ms = LaunchConfiguration('timer_period_ms')
    frame_id = LaunchConfiguration('frame_id')
    child_frame_id = LaunchConfiguration('child_frame_id')
    publish_tf = LaunchConfiguration('publish_tf')
    init_offset_m = LaunchConfiguration('init_offset_m')
    stop_radius_m = LaunchConfiguration('stop_radius_m')
    noise_pos_std_m = LaunchConfiguration('noise_pos_std_m')
    noise_yaw_std_deg = LaunchConfiguration('noise_yaw_std_deg')
    log_debug = LaunchConfiguration('log_debug')

    # --- ノード定義 ---
    robot_sim_node = Node(
        package='robot_simulator',
        executable='robot_simulator',
        name='robot_simulator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'speed_kmph': speed_kmph,
            'timer_period_ms': timer_period_ms,
            'frame_id': frame_id,
            'child_frame_id': child_frame_id,
            'publish_tf': publish_tf,
            'init_offset_m': init_offset_m,
            'stop_radius_m': stop_radius_m,
            'noise_pos_std_m': noise_pos_std_m,
            'noise_yaw_std_deg': noise_yaw_std_deg,
            'log_debug': log_debug,
        }]
    )

    return LaunchDescription([
        speed_kmph_arg,
        timer_period_ms_arg,
        frame_id_arg,
        child_frame_id_arg,
        publish_tf_arg,
        init_offset_m_arg,
        stop_radius_m_arg,
        noise_pos_std_m_arg,
        noise_yaw_std_deg_arg,
        log_debug_arg,
        robot_sim_node
    ])
