import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

env_namespace = os.getenv('ROBOT_NAMESPACE')
if env_namespace == None:
    print(f"[SCUTTLE_TELEOP] - Namespace settings not available. Running node without namespace")
    ROBOT_NAMESPACE = "/"
else:
    print(
        f"[SCUTTLE_TELEOP] - Namespace settings found. Running node with namespace: {env_namespace}")
    ROBOT_NAMESPACE = env_namespace


def generate_launch_description():
    JOY_CONFIG = {
        'dev': '/dev/input/js0',
        'deadzone': 0.3,
        'autorepeat_rate': 20.0
    }

    TELEOP_TWIST_JOY_CONFIG = {
        'axis_linear': {
            'x': 1
        },  # Left thumb stick vertical
        'scale_linear': {
            'x': 0.15
        },
        'scale_linear_turbo': {
            'x': 1.0
        },
        'axis_angular':  {
            'yaw': 0
        },
        'scale_angular': {
            'yaw': 0.75
        },
        'enable_button': 2,  # Left trigger button
        'enable_turbo_button': 5  # Right trigger button
    }

    launch_include_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(ROBOT_NAMESPACE),
            Node(
                package='joy',
                executable='joy_node',
                output='screen',
                parameters=[JOY_CONFIG]
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                output='screen',
                parameters=[TELEOP_TWIST_JOY_CONFIG]
            )
        ]
    )

    return LaunchDescription([
        launch_include_with_namespace
    ])
