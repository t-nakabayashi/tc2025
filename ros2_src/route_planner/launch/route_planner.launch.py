from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- パッケージ共有ディレクトリを取得 ---
    pkg_share = FindPackageShare('route_planner')
    default_config = PathJoinSubstitution([pkg_share, 'routes', 'config.yaml'])
    default_csv_dir = PathJoinSubstitution([pkg_share, 'routes'])

    # --- launch引数の宣言 ---
    config_yaml_arg = DeclareLaunchArgument(
        'config_yaml_path',
        default_value=default_config,
        description='ルート構成YAMLファイルのパス (デフォルト: <pkg_share>/routes/config.yaml)'
    )
    csv_base_dir_arg = DeclareLaunchArgument(
        'csv_base_dir',
        default_value=default_csv_dir,
        description='CSVファイルの基準ディレクトリ (デフォルト: <pkg_share>/csv)'
    )
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='route_planner',
        description='ノード名'
    )
    get_service_arg = DeclareLaunchArgument(
        'get_service_name',
        default_value='GetRoute',
        description='GetRouteサービス名'
    )
    update_service_arg = DeclareLaunchArgument(
        'update_service_name',
        default_value='UpdateRoute',
        description='UpdateRouteサービス名'
    )

    # --- launch引数の取得 ---
    config_yaml_path = LaunchConfiguration('config_yaml_path')
    csv_base_dir = LaunchConfiguration('csv_base_dir')
    node_name = LaunchConfiguration('node_name')
    get_service_name = LaunchConfiguration('get_service_name')
    update_service_name = LaunchConfiguration('update_service_name')

    # --- Node定義 ---
    route_planner_node = Node(
        package='route_planner',
        executable='route_planner',
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_yaml_path': config_yaml_path,
            'csv_base_dir': csv_base_dir,
        }],
        remappings=[
            ('GetRoute', get_service_name),
            ('UpdateRoute', update_service_name),
        ]
    )

    return LaunchDescription([
        config_yaml_arg,
        csv_base_dir_arg,
        node_name_arg,
        get_service_arg,
        update_service_arg,
        route_planner_node
    ])
