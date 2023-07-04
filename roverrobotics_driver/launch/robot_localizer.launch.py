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
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    
    ld.add_action(declare_use_sim_time_argument)
    # # Realsense Node
    # realsense_configuration_path = Path(get_package_share_directory(
    #     'roverrobotics_driver'), 'config/realsense_config.yaml')
    imu_filter_config = os.path.join(get_package_share_directory(
        'roverrobotics_driver'), 'config/imu_filter_config.yaml')
    
    # realsense_node = Node(
    # 	package='realsense2_camera',
    #     name="realsense",
    #     executable='realsense2_camera_node',
    #     parameters=[realsense_configuration_path],
    #     output='screen'
    #     )
    # ld.add_action(realsense_node)
    
    imu_filter_madgwick = Node(
    	package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config],
        output='screen'
        )
    ld.add_action(imu_filter_madgwick)

    # Start robot localization using an Extended Kalman filter
    robot_localization_file_path = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config/localization_ekf.yaml')
    
    localization_node = Node(
    	package='robot_localization',
    	executable='ekf_node',
    	name='ekf_filter_node',
    	output='screen',
    	parameters=[{'use_sim_time': use_sim_time}, robot_localization_file_path]
    	)
    ld.add_action(localization_node)

    #  # RP Lidar Setup
    # serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    # serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') #for s2 is 1000000
    # frame_id = LaunchConfiguration('frame_id', default='laser')
    # inverted = LaunchConfiguration('inverted', default='false')
    # angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    # scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    # scan_frequency = LaunchConfiguration('scan_frequency', default='10.0')
    
    
    # serial_port_ld = DeclareLaunchArgument(
    #     'serial_port',
    #     default_value=serial_port,
    #     description='Specifying usb port to connected lidar')

    # serial_baud_ld = DeclareLaunchArgument(
    #     'serial_baudrate',
    #     default_value=serial_baudrate,
    #     description='Specifying usb port baudrate to connected lidar')
        
    # frame_ld = DeclareLaunchArgument(
    #     'frame_id',
    #     default_value=frame_id,
    #     description='Specifying frame_id of lidar')

    # inverted_ld = DeclareLaunchArgument(
    #     'inverted',
    #     default_value=inverted,
    #     description='Specifying whether or not to invert scan data')

    # angle_ld = DeclareLaunchArgument(
    #     'angle_compensate',
    #     default_value=angle_compensate,
    #     description='Specifying whether or not to enable angle_compensate of scan data')

    # scan_ld = DeclareLaunchArgument(
    #     'scan_mode',
    #     default_value=scan_mode,
    #     description='Specifying scan mode of lidar')

    # lidar_node = Node(
    #     package='rplidar_ros',
    #     executable='rplidar_node',
    #     name='rplidar_node',
    #     parameters=[{'serial_port': serial_port, 
    #                  'serial_baudrate': serial_baudrate, 
    #                  'frame_id': frame_id,
    #                  'inverted': inverted, 
    #                  'angle_compensate': angle_compensate, 
    #                  'scan_mode': scan_mode,
    #                  'scan_frequency': scan_frequency}],
    #     output='screen')
    
    # # Add lidar setup to launch description
    # ld.add_action(serial_port_ld)
    # ld.add_action(serial_baud_ld)
    # ld.add_action(frame_ld)
    # ld.add_action(inverted_ld)
    # ld.add_action(angle_ld)
    # ld.add_action(scan_ld)
    # ld.add_action(lidar_node)
    
    # # Static Transforms
    # camera_tf = Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output='screen',
    #         arguments=['0.1905', '0', '0.3302', '0', '0', '0', 'base_link', 'camera_link'],
    #     ) 
    # ld.add_action(camera_tf)
    
    # laser_tf = Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output='screen',
    #         arguments=['0.15875', '0', '0.37465', str(pi), '0', '0', 'base_link', 'laser'],
    #     )
    # ld.add_action(laser_tf)
    
    return ld

