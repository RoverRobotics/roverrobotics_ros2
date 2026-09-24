from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_config = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config', 'ps4_controller_config.yaml')
    assert controller_config.is_file()
    topics_config = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config', 'topics.yaml')
    assert topics_config.is_file()
    ld = LaunchDescription()

    # Declare optional launch arguments for robot-specific overrides
    # Default -1.0 means "use topics.yaml default"
    ld.add_action(DeclareLaunchArgument('lin_increment', default_value='-1.0'))
    ld.add_action(DeclareLaunchArgument('ang_increment', default_value='-1.0'))
    ld.add_action(DeclareLaunchArgument('max_lin_speed', default_value='-1.0'))
    ld.add_action(DeclareLaunchArgument('max_ang_speed', default_value='-1.0'))
    ld.add_action(DeclareLaunchArgument('start_lin_throttle', default_value='-1.0'))
    ld.add_action(DeclareLaunchArgument('start_ang_throttle', default_value='-1.0'))

    node = Node(
        package='roverrobotics_input_manager',
        executable='joys_manager.py',
        output='screen',
        parameters=[
                {"controller": str(controller_config),
                 "topics": str(topics_config),
                 "lin_increment": LaunchConfiguration('lin_increment'),
                 "ang_increment": LaunchConfiguration('ang_increment'),
                 "max_lin_speed": LaunchConfiguration('max_lin_speed'),
                 "max_ang_speed": LaunchConfiguration('max_ang_speed'),
                 "start_lin_throttle": LaunchConfiguration('start_lin_throttle'),
                 "start_ang_throttle": LaunchConfiguration('start_ang_throttle')}],
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
