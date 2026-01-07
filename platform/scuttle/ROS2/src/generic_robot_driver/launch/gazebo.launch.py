import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('generic_robot_driver')

    # 1. Process the URDF
    xacro_file = os.path.join(pkg_path, 'urdf', 'scuttle_ydlidar.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {
        'robot_description': robot_description_config.toxml()}  # type: ignore
    world_file_path = os.path.join(pkg_path, 'worlds', 'shapes.world')
    # 2. Launch Gazebo (server + client)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    # 3. Spawn Entity (The Robot)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'scuttle'],
        output='screen'
    )

    # 4. Robot State Publisher (Necessary for TFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
