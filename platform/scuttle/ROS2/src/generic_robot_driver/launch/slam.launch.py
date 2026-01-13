import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    start_async_slam_toolbox_node = Node(
        parameters=[
          {
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'mode': 'mapping',
            
            # --- CONNECTION TUNING ---
            'transform_timeout': 1.0,     # Wait for Odom
            'tf_buffer_duration': 30.0,
            
            # --- PERFORMANCE TUNING ---
            # Update position every 0.1s (Matches typical 8-10Hz Lidar speed)
            # Setting this lower than your Lidar speed (e.g. 0.01) does nothing useful.
            'minimum_time_interval': 0.0, 

            # Process EVERY scan (Don't skip any data)
            'throttle_scans': 1,

            # Redraw the black/white map grid once per second.
            # (Saving CPU here helps the position tracker run smoother)
            'map_update_interval': 1.0,
            
            # Publish the correction to TF at 50Hz (Smooth visual slide)
            'transform_publish_period': 0.02,
            
            'resolution': 0.05,
            'max_laser_range': 12.0,
            
            # --- SCAN MATCHER (LOCKING) ---
            # These settings encourage the solver to trust the laser scan match
            'distance_variance_penalty': 0.3,
            'angle_variance_penalty': 0.1,
            'fine_search_angle_offset': 0.00349,
            'coarse_search_angle_offset': 0.349,
            'coarse_angle_resolution': 0.0349,
            'minimum_angle_penalty': 0.9,
            'minimum_distance_penalty': 0.5,
            'use_response_expansion': True,
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    return LaunchDescription([
        declare_use_sim_time_argument,
        start_async_slam_toolbox_node
    ])