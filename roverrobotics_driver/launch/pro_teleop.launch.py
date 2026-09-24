from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    hardware_config = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config', 'pro_config.yaml')

    # Read pro-specific joy_manager params from pro_config.yaml
    with open(hardware_config) as f:
        pro_cfg = yaml.safe_load(f)
    joy_params = pro_cfg.get('joy_manager', {}).get('ros__parameters', {})

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/pro.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/ps5_controller.launch.py']),
            launch_arguments={
                'lin_increment': str(joy_params.get('lin_increment', -1.0)),
                'ang_increment': str(joy_params.get('ang_increment', -1.0)),
                'max_lin_speed': str(joy_params.get('max_lin_speed', -1.0)),
                'max_ang_speed': str(joy_params.get('max_ang_speed', -1.0)),
                'start_lin_throttle': str(joy_params.get('start_lin_throttle', -1.0)),
                'start_ang_throttle': str(joy_params.get('start_ang_throttle', -1.0)),
            }.items()),
    ])
