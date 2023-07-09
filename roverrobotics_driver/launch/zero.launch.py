from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rover_path = get_package_share_path('roverrobotics_description')
    default_model_path = rover_path / 'urdf/rover_4wd.urdf'
    
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
   
    hardware_config = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config', 'zero_config.yaml')
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

    accessories_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('roverrobotics_driver'), '/launch/accessories.launch.py']),
    )
   
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    ld.add_action(model_arg)
    ld.add_action(robot_driver)
    ld.add_action(accessories_launch)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
   
    return ld
