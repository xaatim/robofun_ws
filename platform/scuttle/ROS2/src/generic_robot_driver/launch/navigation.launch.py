import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Get Package Paths
    pkg_generic_driver = get_package_share_directory('generic_robot_driver')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 2. Define Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # 3. Declare Arguments (This makes them flexible)
    
    # Argument 1: Simulation Time (Default: False for real robot)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Argument 2: Map File
    # Set a default path, but allow overriding with map:=/new/path.yaml
    default_map_path = os.path.join(pkg_generic_driver, 'maps', 'my_map4.yaml')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load')

    # Argument 3: Params File
    default_params_path = os.path.join(pkg_generic_driver, 'config', 'nav2_scuttle_devkit_params.yaml')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # 4. Include the Nav2 Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        nav2_launch
    ])