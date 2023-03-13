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
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model')
]


def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory('turtlebot4_ignition_bringup')

    # Parameters
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_ignition_bringup, 'config', 'turtlebot4_node.yaml']),
        description='Turtlebot4 Robot param file'
    )

    turtlebot4_node_yaml_file = LaunchConfiguration('param_file')

    # Turtlebot4 node
    turtlebot4_node = Node(
        package='turtlebot4_node',
        name='turtlebot4_node',
        executable='turtlebot4_node',
        parameters=[turtlebot4_node_yaml_file,
                    {'model': LaunchConfiguration('model')}],
        output='screen',
    )

    # Turtlebot4 Ignition Hmi node
    turtlebot4_ignition_hmi_node = Node(
        package='turtlebot4_ignition_toolbox',
        name='turtlebot4_ignition_hmi_node',
        executable='turtlebot4_ignition_hmi_node',
        output='screen',
        condition=LaunchConfigurationEquals('model', 'standard')
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(turtlebot4_node)
    ld.add_action(turtlebot4_ignition_hmi_node)
    return ld
