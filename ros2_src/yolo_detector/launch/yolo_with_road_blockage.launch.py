from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('yolo_detector')
    default_route_param = PathJoinSubstitution([pkg_share, 'params', 'road_blockage_detector.yaml'])

    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/usb_cam/image_raw', description='購読する画像トピック'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='', description='使用するPyTorchモデルのパス'
    )
    detection_interval_arg = DeclareLaunchArgument(
        'detection_interval',
        default_value='0.5',
        description='推論を行うインターバル（秒）',
    )
    image_size_arg = DeclareLaunchArgument(
        'image_size', default_value='320', description='推論時の入力画像サイズ'
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5', description='検出の信頼度閾値'
    )
    route_param_file_arg = DeclareLaunchArgument(
        'route_param_file',
        default_value=default_route_param,
        description='road_blockage_detectorのパラメータファイル',
    )

    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'detection_interval': LaunchConfiguration('detection_interval'),
                'image_size': LaunchConfiguration('image_size'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            }
        ],
    )

    road_blockage_node = Node(
        package='yolo_detector',
        executable='road_blockage_detector',
        name='road_blockage_detector',
        output='screen',
        parameters=[LaunchConfiguration('route_param_file')],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            model_path_arg,
            detection_interval_arg,
            image_size_arg,
            confidence_threshold_arg,
            route_param_file_arg,
            yolo_node,
            road_blockage_node,
        ]
    )
