import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the Cartographer config files
    cartographer_config_dir = os.path.join(
        get_package_share_directory('robot_cartographer_config'), 'config'
    )
    configuration_basename = LaunchConfiguration('configuration_basename')

    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='aamabot3_lds_2d.lua', # Or whatever you named your main config
        description='Name of lua file for Cartographer configuration.'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('robot_cartographer_config'),
        'config',
        'cartographer.rviz'  # Make sure this file exists in 'rviz/' directory
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}], # Set to False if not in simulation
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('scan', '/scan') # Ensure this matches your YDLidar's topic
        ]
    )

    # Optional: Occupancy grid node to visualize the map
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}], # Set to False if not in simulation
        arguments=[
            '-resolution', '0.05', # Map resolution
            '-publish_period_sec', '1.0' # How often to publish the map
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]  # match with rest of the system
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link']
    )


    return LaunchDescription([
        declare_configuration_basename_cmd,
        cartographer_node,
        occupancy_grid_node,
        static_tf_publisher,
        rviz_node,
    ])