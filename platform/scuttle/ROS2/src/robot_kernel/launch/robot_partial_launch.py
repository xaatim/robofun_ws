from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='log',
        ),

        Node(
            package='robot_control',
            executable='mode_manager',
            name='mode_manager',
            output='log',
        ),

        Node(
            package='robot_control',
            executable='manual_control',
            name='manual_control',
            output='log',
        ),

        Node(
            package='robot_control',
            executable='autonomous_control',
            name='autonomous_control',
            output='log',
        ),

        Node(
            package='robot_control',
            executable='motor_driver',
            name='motor_driver',
            output='log',
        ),

          Node(
              package='robot_vision',
              executable='camera_pub',
              name='camera_pub',
              output='log',
          ),

                Node(
            package='robot_kernel',
            executable='socketio_node',
            name='socketio_node',
            output='screen',
        ),
    ])
