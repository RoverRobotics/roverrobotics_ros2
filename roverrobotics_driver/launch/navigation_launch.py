# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3
# Licensed under Apache 2.0

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # --- Package dirs
    rover_dir   = get_package_share_directory('roverrobotics_driver')
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_ld  = os.path.join(bringup_dir, 'launch')

    # --- Common args
    use_sim_time    = LaunchConfiguration('use_sim_time')
    log_level       = LaunchConfiguration('log_level')
    autostart       = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    slam            = LaunchConfiguration('slam')

    # Nav2 params
    params_file     = LaunchConfiguration('params_file')

    # SLAM (mapping) file (reuses your existing slam_launch.py)
    slam_params_file = LaunchConfiguration('slam_params_file')

    # SLAM toolbox **localization** (NO YAML) args
    slam_localization_params_file = LaunchConfiguration('slam_localization_params_file')
    map_file_name = LaunchConfiguration('map_file_name')  # <<<< your serialized map path (no YAML)

    # EKF params
    ekf_params_file = LaunchConfiguration('ekf_params_file')

    # --- Declarations
    declares = [
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument('log_level', default_value='info', description='RCUTILS log level'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart lifecycle nodes'),
        DeclareLaunchArgument('use_composition', default_value='False', description='Use component container for Nav2'),
        DeclareLaunchArgument('slam', default_value='False',description='False: localization with slam_toolbox (no YAML)'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(rover_dir, 'config', 'nav2_params.yaml'),
                              description='Nav2 parameters YAML'),
        # EKF params
        DeclareLaunchArgument('ekf_params_file',
                              default_value=os.path.join(rover_dir, 'config', 'localization_ekf.yaml'),
                              description='robot_localization EKF params'),
        # Mapping params (your existing async SLAM launch will use this)
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(rover_dir, 'config', 'slam_configs',
                                                         'mapper_params_online_async.yaml'),
                              description='slam_toolbox params for mapping'),
        # Localization params (slam_toolbox localization mode)
        DeclareLaunchArgument('slam_localization_params_file',
                              default_value=os.path.join(rover_dir, 'config', 'slam_configs',
                                                         'mapper_params_localization.yaml'),
                              description='slam_toolbox params for localization mode'),
        # **No YAML**: pass serialized map via map_file_name (absolute or relative path)
        DeclareLaunchArgument('map_file_name',
                              default_value='',
                              description='Path to slam_toolbox serialized map (e.g., /home/user/maps/office_map)'),
    ]

    # Namespace-safe TF remaps
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # Unbuffer logs
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Ensure use_sim_time reaches Nav2 servers even under namespaces
    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)

    # --- Always include EKF (world_frame kept at odom; SLAM/Loc publishes map->odom)
    robot_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rover_dir, 'launch', 'robot_localizer.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ekf_params_file': ekf_params_file,
        }.items()
    )

    # --- Localization WITHOUT YAML: slam_toolbox localization + lifecycle manager
    slam_localization_node = Node(
        condition=UnlessCondition(slam),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        # namespace=namespace,
        output='screen',
        parameters=[
            slam_localization_params_file,
            {
                'use_sim_time': use_sim_time,
                # The important bit: point to serialized map, NOT a YAML
                'map_file_name': map_file_name,
            },
        ],
    )

    slam_localization_lifecycle = Node(
        condition=UnlessCondition(slam),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
    )

    # --- Optional Nav2 component container
    nav2_container = Node(
        condition=IfCondition(use_composition),
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_nav2_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
        output='screen'
    )

    # --- Core Nav2 stack (planner/controller/BT navigator/etc.)
    nav2_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rover_dir, 'launch', 'nav2_backend.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_nav2_params,
            'use_composition': use_composition,
            'container_name': 'nav2_container',
        }.items()
    )

    # --- Namespace group
    bringup_group = GroupAction([
        nav2_container,
        nav2_backend,
    ])

    # --- LD
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for d in declares:
        ld.add_action(d)

    # Nodes / includes
    ld.add_action(robot_localizer_launch)
    ld.add_action(slam_localization_node)       # only if slam:=false
    ld.add_action(slam_localization_lifecycle)  # only if slam:=false
    ld.add_action(bringup_group)

    return ld