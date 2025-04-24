from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    controller_config = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config', 'ps5_controller_config.yaml')
    assert controller_config.is_file()
    topics_config = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config', 'topics.yaml')
    assert topics_config.is_file()
    ld = LaunchDescription()

    node = Node(
        package='roverrobotics_input_manager',
        executable='joys_manager.py',
        output='screen',
        parameters=[
                {"controller": str(controller_config),
                 "topics": str(topics_config)}],
        respawn=True,
        respawn_delay=1
    )

    ld.add_action(node)
    node2 = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        parameters=[],
        respawn=True,
        respawn_delay=1
    )
    ld.add_action(node2)
    return ld
