#!/usr/bin/env python3

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    accessories_config_path = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config/accessories.yaml')

    with open(accessories_config_path, 'r') as f:
        accessories_config = yaml.safe_load(f)

    imu_config = accessories_config.get('bno055', {}).get('ros__parameters', {})
    if not imu_config.get('active', False):
        return LaunchDescription([LogInfo(
            msg='IMU is off: set active: true under bno055 in accessories.yaml')])

    # BNO055 IMU; rover-bno055.service resets the port and restarts it if it exits
    imu_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[str(accessories_config_path)],
        remappings=[
            ('/imu', '/imu/data')
        ],
        output='screen',
        on_exit=Shutdown())

    return LaunchDescription([imu_node])
