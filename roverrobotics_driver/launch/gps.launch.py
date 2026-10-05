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

    gps_config = accessories_config.get('ublox_gps_node', {}).get('ros__parameters', {})
    if not gps_config.get('active', False):
        return LaunchDescription([LogInfo(
            msg='GPS is off: set active: true under ublox_gps_node in accessories.yaml')])

    # u-blox GPS; rover-ublox.service restarts it if it exits
    gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        parameters=[str(accessories_config_path)],
        remappings=[
            ('~/fix', '/fix'),
            ('~/fix_velocity', '/fix_velocity'),
            ('~/navpvt', '/navpvt')
        ],
        output='screen',
        on_exit=Shutdown())

    return LaunchDescription([gps_node])
