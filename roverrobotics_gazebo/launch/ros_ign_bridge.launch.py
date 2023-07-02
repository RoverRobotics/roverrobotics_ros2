# Copyright 2023 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='RoverRobotics',
                          description='Ignition model name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='World name'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')


    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    create3_ros_gz_bridge_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ros_ignition_bridge.launch.py'])

    create3_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ros_gz_bridge_launch]),
        launch_arguments=[
            ('robot_name', robot_name),
            ('dock_name', dock_name),
            ('namespace', namespace),
            ('world', world)
        ]
    )
    
    # cmd_vel bridge
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            [namespace,
             '/cmd_vel' + '@geometry_msgs/msg/Twist' + '[ignition.msgs.Twist']
        ],
        remappings=[
            ([namespace, '/cmd_vel'], 'cmd_vel'),
            (['/model/', robot_name, '/cmd_vel'],
             'diffdrive_controller/cmd_vel_unstamped')
        ])

    # Pose bridge
    pose_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                       name='pose_bridge',
                       output='screen',
                       parameters=[{
                            'use_sim_time': use_sim_time
                       }],
                       arguments=[
                           ['/model/', robot_name, '/pose' +
                            '@tf2_msgs/msg/TFMessage' +
                            '[ignition.msgs.Pose_V'],
                           ['/model/', dock_name, '/pose' +
                            '@tf2_msgs/msg/TFMessage' +
                            '[ignition.msgs.Pose_V']
                       ],
                       remappings=[
                           (['/model/', robot_name, '/pose'],
                            '_internal/sim_ground_truth_pose'),
                           (['/model/', dock_name, '/pose'],
                            '_internal/sim_ground_truth_dock_pose')
                       ])




    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(create3_bridge)
    return ld
