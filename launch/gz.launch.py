"""
test.launch.py
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Entrypoint"""
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'world',
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'namespace',
            description='Namespace to use'
        ),
        DeclareLaunchArgument(
            'config_file',
            description='Configuration file for the bridge'
        ),
        SetParameter(name='use_sim_time', value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'config_file': LaunchConfiguration('config_file')},
            ]
        ),
        Node(
            package='as2_gazebo_assets',
            executable='ground_truth_bridge',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'name_space': LaunchConfiguration('namespace'),
                 'pose_frame_id': 'earth',
                 'twist_frame_id': [LaunchConfiguration('namespace'), '/base_link']},
            ]
        ),
    ])

    return launch_description
