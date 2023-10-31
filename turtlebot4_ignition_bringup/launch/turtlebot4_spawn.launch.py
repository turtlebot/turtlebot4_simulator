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


from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to launch localization'),
    DeclareLaunchArgument('slam', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to launch SLAM'),
    DeclareLaunchArgument('nav2', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to launch Nav2'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    # Paths
    turtlebot4_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ros_ign_bridge.launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    turtlebot4_node_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_nodes.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_ignition_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ignition_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])
    dock_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'dock_description.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    slam_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'slam.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])

    # Parameters
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_ignition_bringup, 'config', 'turtlebot4_node.yaml']),
        description='Turtlebot4 Robot param file')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    turtlebot4_node_yaml_file = LaunchConfiguration('param_file')

    robot_name = GetNamespacedName(namespace, 'turtlebot4')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    # Calculate dock offset due to yaw rotation
    dock_offset_x = RotationalOffsetX(0.157, yaw)
    dock_offset_y = RotationalOffsetY(0.157, yaw)
    # Spawn dock at robot position + rotational offset
    x_dock = OffsetParser(x, dock_offset_x)
    y_dock = OffsetParser(y, dock_offset_y)
    # Spawn robot slightly clsoer to the floor to reduce the drop
    # Ensures robot remains properly docked after the drop
    z_robot = OffsetParser(z, -0.0025)
    # Rotate dock towards robot
    yaw_dock = OffsetParser(yaw, 3.1416)

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments=[('model', LaunchConfiguration('model')),
                              ('use_sim_time', LaunchConfiguration('use_sim_time'))]
        ),

        # Dock description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([dock_description_launch]),
            # The robot starts docked
            launch_arguments={'gazebo': 'ignition'}.items(),
        ),

        # Spawn TurtleBot 4
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z_robot,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),

        # Spawn Dock
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=['-name', dock_name,
                       '-x', x_dock,
                       '-y', y_dock,
                       '-z', z,
                       '-Y', yaw_dock,
                       '-topic', 'standard_dock_description'],
            output='screen',
        ),

        # ROS IGN bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_ros_ign_bridge_launch]),
            launch_arguments=[
                ('model', LaunchConfiguration('model')),
                ('robot_name', robot_name),
                ('dock_name', dock_name),
                ('namespace', namespace)]
        ),

        # TurtleBot 4 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_node_launch]),
            launch_arguments=[('model', LaunchConfiguration('model')),
                              ('param_file', turtlebot4_node_yaml_file)]
        ),

        # Create 3 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_nodes_launch]),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # Create 3 Ignition nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_ignition_nodes_launch]),
            launch_arguments=[
                ('robot_name', robot_name),
                ('dock_name', dock_name),
            ]
        ),

        # RPLIDAR static transforms
        Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0.0',
                'rplidar_link', [robot_name, '/rplidar_link/rplidar']],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),

        # OAKD static transform
        # Required for pointcloud. See https://github.com/gazebosim/gz-sensors/issues/239
        Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0',
                '1.5707', '-1.5707', '0',
                'oakd_rgb_camera_optical_frame',
                [robot_name, '/oakd_rgb_camera_frame/rgbd_camera']
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),

    ])

    # Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('nav2'))
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(spawn_robot_group_action)
    ld.add_action(localization)
    ld.add_action(slam)
    ld.add_action(nav2)
    ld.add_action(rviz)
    return ld
