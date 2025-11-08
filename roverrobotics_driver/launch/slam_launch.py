#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params  = LaunchConfiguration('slam_params_file')
    autostart    = LaunchConfiguration('autostart_lifecycle')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation/Gazebo clock')

    default_slam_params = os.path.join(
        get_package_share_directory('roverrobotics_driver'),
        'config', 'slam_configs', 'mapper_params_online_async.yaml')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params,
        description='Full path to slam_toolbox params YAML')

    declare_autostart = DeclareLaunchArgument(
        'autostart_lifecycle', default_value='true',
        description='Auto-activate slam_toolbox lifecycle')

    # --- Include EKF (so slam + ekf always start together)
    rl_launch_path = os.path.join(
        get_package_share_directory('roverrobotics_driver'),
        'launch', 'robot_localizer.launch.py'
    )
    robot_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rl_launch_path),launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- slam_toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Ensure lifecycle node reaches ACTIVE in Jazzy reliably
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['slam_toolbox'],
            # Avoids long bonds if you kill quickly
            'bond_timeout': 0.0,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam_params)
    ld.add_action(declare_autostart)
    ld.add_action(robot_localizer_launch)
    ld.add_action(slam_node)
    ld.add_action(lifecycle_mgr)
    return ld
