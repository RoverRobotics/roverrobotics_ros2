#!/usr/bin/env python3
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params    = LaunchConfiguration('ekf_params_file')
    publish_tf    = LaunchConfiguration('publish_tf')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation/Gazebo clock')

    default_ekf = os.path.join(
        get_package_share_directory('roverrobotics_driver'),
        'config', 'localization_ekf.yaml')

    declare_ekf_params = DeclareLaunchArgument(
        'ekf_params_file', default_value=default_ekf,
        description='Full path to EKF params YAML')

    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='Let robot_localization publish TFs')

    # --- EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params,
            {'use_sim_time': use_sim_time,
             'publish_tf': publish_tf,
            },
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_ekf_params)
    ld.add_action(declare_publish_tf)
    ld.add_action(ekf_node)
    return ld
