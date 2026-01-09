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

    # 2. Define Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Path to your saved map (Change 'my_new_map.yaml' to your actual map name if different)
    map_yaml_file = LaunchConfiguration('map', 
        default=os.path.join(pkg_generic_driver, 'maps', 'my_new_map.yaml'))
        
    # Path to the params file you uploaded
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_generic_driver, 'config', 'nav2_scuttle_devkit_params.yaml'))

    # 3. Include the Standard Nav2 Launch
    # This starts: AMCL, Planner (NavFn), Controller (DWB), Map Server, BT Navigator, etc.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true'  # Automatically transitions nodes to "Active" state
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])