# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_monitor',
            executable='obstacle_monitor',
            name='obstacle_monitor',
            output='screen',
            parameters=[{
                'pub_rate_hz': 10.0,
                'front_half_deg': 12.5,
                'stop_dist_m': 1.5,
                'robot_width_m': 0.45,
                'safety_margin_m': 0.10,
                'enable_viewer': True,
                'viewer_image_size': 500,
                'viewer_scale_px_per_m': 50.0,
            }]
        )
    ])
