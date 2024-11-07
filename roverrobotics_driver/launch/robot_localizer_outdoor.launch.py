#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from math import pi

def generate_launch_description():

    nmeasat_launch_path = os.path.join(get_package_share_directory("nmea_navsat_driver"), 'launch', 'nmea_serial_driver.launch.py')
    nmeasat_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(nmeasat_launch_path))


    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')


    # Start robot localization using an Extended Kalman filter
    robot_localization_file_path = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config/localization_outdoor_ekf.yaml')
    
    localization_node = Node(
    	package='robot_localization',
    	executable='ekf_node',
    	name='ekf_filter_node',
    	output='screen',
    	parameters=[{'use_sim_time': use_sim_time}, robot_localization_file_path],
        remappings=[('odometry/filtered', 'odometry/local')]
    	)
    
    # Start robot localization using an Extended Kalman filter...map->odom transform
    gps_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_gps',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_localization_file_path],
        remappings=[('odometry/filtered', 'odometry/global')]
        )
    
        # Start the navsat transform node which converts GPS data into the world coordinate frame
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_localization_file_path],
        remappings=[('imu', 'imu/data'),
                    ('gps/fix', 'fix'),
                    ('odometry/filtered', 'odometry/global')]
        )


    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(localization_node)
    ld.add_action(navsat_transform_node)
    ld.add_action(gps_localization_node)
    ld.add_action(nmeasat_launch)
    
    return ld

