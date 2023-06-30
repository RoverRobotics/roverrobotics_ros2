from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # urdf = Path(get_package_share_directory(
    #     'roverrobotics_description'), 'urdf', 'rover.urdf')
    # assert urdf.is_file()
    hardware_config = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config', 'miti_config.yaml')
    assert hardware_config.is_file()
    ld = LaunchDescription()

    robot_driver = Node(
        package = 'roverrobotics_driver',
        name = 'roverrobotics_driver',
        executable = 'roverrobotics_driver',
        parameters = [hardware_config],
        output='screen',
        respawn=True,
        respawn_delay=1
    )
    
    ld.add_action(robot_driver)
    return ld
