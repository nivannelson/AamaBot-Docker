from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=['/root/aammabot_ws/src/AamaBot/ydlidar_ros2_driver/params/X2.yaml'],
        output='screen',
        arguments=['--ros-args', '--log-level', 'error'],
    )

    tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        output='screen',
    )

    motor_node = Node(
        package='motor_control',
        executable='twist_motor_subscriber',
        name='twist_motor_subscriber_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rf2o_laser_odometry'),
                'launch',
                'rf2o_laser_odometry.launch.py'
            )
        )
    )

    return LaunchDescription([
        # Start LIDAR and TF first
        ydlidar_node,
        tf_pub,
        motor_node,

        # Start RF2O with enough delay (e.g., 5s)
        TimerAction(
            period=5.0,
            actions=[rf2o_launch]
        )
    ])
