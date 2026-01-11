import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Paths and Configurations ---
    package_path = get_package_share_path('generic_robot_driver')
    urdf_path = package_path / 'urdf/scuttle_ydlidar.xacro'
    ydlidar_config = os.path.join(package_path, 'config/ydlidar.yaml')
    
    # Joystick Config from robot_teleop.launch.py
    JOY_CONFIG = {
        'dev': '/dev/input/js0',
        'deadzone': 0.3,
        'autorepeat_rate': 20.0
    }

    TELEOP_CONFIG = {
        'axis_linear': {'x': 1},
        'scale_linear': {'x': 0.15},
        'scale_linear_turbo': {'x': 1.0},
        'axis_angular': {'yaw': 0},
        'scale_angular': {'yaw': 0.75},
        'enable_button': 2,
        'enable_turbo_button': 5
    }

    # --- Nodes from Bringup ---
    
    # 1. Robot State Publisher (TF & URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', str(urdf_path)]), value_type=str)}]
    )

    # 2. Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # 3. Base Driver (Motor Control)
    robot_driver = Node(
        package='generic_robot_driver',
        executable='generic_robot_driver',
        output='screen',
        parameters=[{
            'enable_tf_pub': True,
            'enable_jointstate_pub': True,
            'motor_type': 'hw231' # Default for devkit
        }],
        remappings=[('odom', 'odom/amr')]
    )

    # 4. YDLIDAR Driver
    ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[ydlidar_config]
    )

    # 5. RealSense Camera
    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='scuttle_camera_node',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'align_depth.enable': True,
            'initial_reset': True
        }]
    )

    # --- Nodes from Teleop ---

    # 6. Joy Node (Reads Controller)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[JOY_CONFIG]
    )

    # 7. Teleop Twist Joy (Converts Joy to cmd_vel)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        parameters=[TELEOP_CONFIG]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        robot_driver,
        ydlidar,
        camera,
        joy_node,
        teleop_node
    ])