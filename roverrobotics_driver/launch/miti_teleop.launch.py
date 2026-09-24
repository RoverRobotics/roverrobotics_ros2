from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from math import pi


def generate_launch_description():

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/miti.launch.py'])),
        # 1.25 x 2.5 rad/s: slower spins stick-slip and hop on the MITI
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/ps5_controller.launch.py']),
            launch_arguments={'start_ang_throttle': '1.25'}.items()),
    ])
