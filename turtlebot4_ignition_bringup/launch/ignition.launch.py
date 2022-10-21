# Copyright 2021 Clearpath Robotics, Inc.
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

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('slam', default_value='off',
                          choices=['off', 'sync', 'async'],
                          description='Whether to run a SLAM'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to run localization'),
    DeclareLaunchArgument('nav2', default_value='false',
                          choices=['true', 'false'],
                          description='Run nav2'),
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
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='robot namespace'),
    DeclareLaunchArgument('yaw', default_value='3.145',
                          description='robot yaw rotation at spawn'),
]

for pose_element in ['x', 'y', 'z']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory(
        'turtlebot4_ignition_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')

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

    # Launch configurations
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')

    # Ignition gazebo
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

    # Turtlebot4
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_turtlebot4_ignition_bringup, 'launch',
                                                   'turtlebot4_spawn.launch.py')),
        launch_arguments={'x': LaunchConfiguration('x'),
                          'y': LaunchConfiguration('y'),
                          'z': LaunchConfiguration('z'),
                          'yaw': LaunchConfiguration('yaw'),
                          'robot_name': robot_name,
                          'robot_description': '/robot_description',
                          'world' : LaunchConfiguration('world'),
                          'model' : LaunchConfiguration('model'),
                          'slam' : LaunchConfiguration('slam'),
                          'localization' : LaunchConfiguration('localization'),
                          'nav2' : LaunchConfiguration('nav2'),
                          'rviz' : LaunchConfiguration('rviz'),
                          'namespace': namespace,
                          'spawn_dock': LaunchConfiguration('spawn_dock'),
                          }.items()
    )
    
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(spawn_robot)
    return ld
