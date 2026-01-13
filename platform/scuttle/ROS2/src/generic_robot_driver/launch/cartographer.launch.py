import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('generic_robot_driver')
    
    # path to default configuration files provided by cartographer_ros
    cartographer_prefix = get_package_share_directory('cartographer_ros')
    
    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    cartographer_config_basename = 'cartographer.lua'

    # 1. Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Search in the default folder first for map_builder.lua, etc.
            '-configuration_directory', os.path.join(cartographer_prefix, 'configuration_files'),
            # Then search in your robot folder for cartographer.lua
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename
        ],
        remappings=[
             # Ensure your lidar topic is correct. YDLidar usually publishes to /scan
             ('scan', 'scan')
        ]
    )

    # 2. Occupancy Grid Node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05},
            {'publish_period_sec': 1.0}
        ]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])