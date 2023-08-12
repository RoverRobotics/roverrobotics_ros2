#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from math import pi

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')


    # Start robot localization using an Extended Kalman filter
    robot_localization_file_path = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config/localization_ekf.yaml')
    
    localization_node = Node(
    	package='robot_localization',
    	executable='ekf_node',
    	name='ekf_filter_node',
    	output='screen',
    	parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
    	)
    
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(localization_node)

    
    return ld

