#!/usr/bin/env python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    default_param_file = os.path.join(share_dir, 'params', 'ydlidar.yaml')

    return LaunchDescription([
        # Allow user to override the params file from CLI
        DeclareLaunchArgument(
            name='params_file',
            default_value=default_param_file,
            description='Full path to the ROS2 parameters file to use.'
        ),

        # LIDAR lifecycle node
        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),

        # Static TF publisher for laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
            output='screen'
        )
    ])
