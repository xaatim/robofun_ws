import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Path to your package
    pkg_path = get_package_share_directory('generic_robot_driver')

    # Path to xacro/urdf
    xacro_file = os.path.join(pkg_path, 'urdf', 'scuttle_ydlidar.xacro')

    # Convert xacro to robot_description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}  # type: ignore

    # use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Joint State Publisher (publishes /joint_states)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',     # GUI version like your last working one
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot State Publisher (publishes /tf and /robot_description)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # RViz
    rviz_config = os.path.join(pkg_path, 'rviz', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
