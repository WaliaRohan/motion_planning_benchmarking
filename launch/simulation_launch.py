# Copyright (c) 2021 Samsung Research America
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

# Original file: https://github.com/ros-planning/navigation2/blob/main/nav2_simple_commander/launch/nav_to_pose_example_launch.py

# This file is intended for:
#   1. Launching gazebo with a given world
#   2. Launching nav2 map_server with a given file

"""Launch turtlebot3_waffle in a world with given map file."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    nav2_benchmark_dir = get_package_share_directory('nav2_benchmark')
    map_yaml_file = LaunchConfiguration('map')

    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value= os.path.join(nav2_benchmark_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map yaml file to load')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf])

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml_file}.items())

    # print(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py'))
    # print(os.path.join(nav2_benchmark_dir, 'maps', 'turtlebot3_world.yaml'))

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(rviz_cmd)
    # ld.add_action(bringup_cmd)
    return ld