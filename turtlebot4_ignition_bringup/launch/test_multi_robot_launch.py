#!/usr/bin/env python3

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

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Dock Model.'),
]

def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory(
        'turtlebot4_ignition_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_navigation2_bringup = get_package_share_directory(
        'nav2_bringup')

    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')
    pkg_irobot_create_ignition_plugins = get_package_share_directory(
        'irobot_create_ignition_plugins')

    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'), ':' +
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'), ':' +
            str(Path(pkg_turtlebot4_description).parent.resolve()), ':' +
            str(Path(pkg_irobot_create_description).parent.resolve())])

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'), ':' +
            os.path.join(pkg_irobot_create_ignition_plugins, 'lib')])

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])


    map_yaml_file = os.path.join(pkg_turtlebot4_navigation, 'maps/depot.yaml')
    bt_xml_file = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                               'behavior_trees',
                               'navigate_to_pose_w_replanning_and_recovery.xml')
    
    robot1_params_file = os.path.join(pkg_turtlebot4_ignition_bringup,
                                      'config/nav2_multirobot_params_1.yaml')
    robot2_params_file = os.path.join(pkg_turtlebot4_ignition_bringup,
                                      'config/nav2_multirobot_params_2.yaml')

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.01},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.01}]

    # Launch Ignition gazebo server for simulation
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [
                LaunchConfiguration('world'), '.sdf',
                ' -v 4',
                ' --gui-config ', PathJoinSubstitution(
                    [pkg_turtlebot4_ignition_bringup,
                     'gui',
                     LaunchConfiguration('model'),
                     'gui.config'])])
        ]
    )

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_turtlebot4_ignition_bringup,
                        'launch',
                        'turtlebot4_spawn.launch.py'
                    )),
                launch_arguments={
                                'x': TextSubstitution(text=str(robot['x_pose'])),
                                'y': TextSubstitution(text=str(robot['y_pose'])),
                                'z': TextSubstitution(text=str(robot['z_pose'])),
                                'robot_name': TextSubstitution(text=robot['name']),
                                'namespace': TextSubstitution(text=robot['name']),
                                'model' : LaunchConfiguration('model'),
                                'spawn_dock': LaunchConfiguration('spawn_dock'),
                                }.items()
            )
        )

    # Define commands for launching the robot state publishers
    # robot_state_pubs_cmds = []
    # for robot in robots:
    #     robot_state_pubs_cmds.append(
    #         Node(
    #             package='robot_state_publisher',
    #             executable='robot_state_publisher',
    #             namespace=TextSubstitution(text=robot['name']),
    #             output='screen',
    #             parameters=[{'use_sim_time': 'True'}],
    #             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    #             arguments=[urdf]))

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = eval(f"{robot['name']}_params_file")

        group = GroupAction([
            # Instances use the robot's name for namespace
            PushRosNamespace(robot['name']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_navigation2_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                                  'namespace': robot['name'],
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'bt_xml_file': bt_xml_file,
                                  'autostart': 'True',
                                  'use_remappings': 'True'}.items())
        ])
        nav_instances_cmds.append(group)


    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    #ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),)
    #ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo)
    for spawn_robot in spawn_robots_cmds:
        ld.add_action(spawn_robot)
    #for state_pub in robot_state_pubs_cmds:
    #    ld.add_action(state_pub)
    for nav_instance in nav_instances_cmds:
        ld.add_action(nav_instance)
    return ld
