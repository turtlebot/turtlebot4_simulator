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
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
]


def generate_launch_description():
    leds = [
        'power',
        'motors',
        'comms',
        'wifi',
        'battery',
        'user1',
        'user2'
    ]

    namespace = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    create3_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ros_ignition_bridge.launch.py'])

    create3_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ros_ign_bridge_launch]),
        launch_arguments=[
            ('robot_name', LaunchConfiguration('robot_name')),
            ('world', LaunchConfiguration('world'))
        ]
    )

    # lidar bridge
    lidar_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/world/', LaunchConfiguration('world'),
             '/model/', LaunchConfiguration('robot_name'),
             '/link/rplidar_link/sensor/rplidar/scan' +
             '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ],
        remappings=[
            (['/world/', LaunchConfiguration('world'),
              '/model/', LaunchConfiguration('robot_name'),
              '/link/rplidar_link/sensor/rplidar/scan'],
             '/scan')
        ])

    # Display message bridge
    hmi_display_msg_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_display_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', LaunchConfiguration('robot_name'), '/hmi/display/raw' +
             '@std_msgs/msg/String' +
             ']ignition.msgs.StringMsg'],
            ['/model/', LaunchConfiguration('robot_name'), '/hmi/display/selected' +
             '@std_msgs/msg/Int32' +
             ']ignition.msgs.Int32']
        ],
        remappings=[
            (['/model/', LaunchConfiguration('robot_name'), '/hmi/display/raw'],
             '/hmi/display/_raw'),
            (['/model/', LaunchConfiguration('robot_name'), '/hmi/display/selected'],
             '/hmi/display/_selected')
        ],
        condition=LaunchConfigurationEquals('model', 'standard'))

    # Buttons message bridge
    hmi_buttons_msg_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_buttons_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', LaunchConfiguration('robot_name'), '/hmi/buttons' +
             '@std_msgs/msg/Int32' +
             '[ignition.msgs.Int32']
        ],
        remappings=[
            (['/model/', LaunchConfiguration('robot_name'), '/hmi/buttons'],
             '/hmi/buttons/_set')
        ],
        condition=LaunchConfigurationEquals('model', 'standard'))

    # Buttons message bridge
    hmi_led_msg_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_led_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', LaunchConfiguration('robot_name'), '/hmi/led/' + led +
             '@std_msgs/msg/Int32' +
             ']ignition.msgs.Int32'] for led in leds
        ],
        remappings=[
            (['/model/', LaunchConfiguration('robot_name'), '/hmi/led/' + led],
             '/hmi/led/_' + led) for led in leds
        ],
        condition=LaunchConfigurationEquals('model', 'standard'))

    # Camera sensor bridge
    oakd_pro_camera_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_link/sensor/rgbd_camera/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_link/sensor/rgbd_camera/depth_image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_link/sensor/rgbd_camera/points' +
                '@sensor_msgs/msg/PointCloud2' +
                '[ignition.msgs.PointCloudPacked'],
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_link/sensor/rgbd_camera/camera_info' +
                '@sensor_msgs/msg/CameraInfo' +
                '[ignition.msgs.CameraInfo'],
                ],
        remappings=[
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_link/sensor/rgbd_camera/image'],
             '/color/image'),
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_link/sensor/rgbd_camera/depth_image'],
             '/stereo/depth'),
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_link/sensor/rgbd_camera/points'],
             '/stereo/depth/points'),
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_link/sensor/rgbd_camera/camera_info'],
             '/color/camera_info')
                ],
        condition=LaunchConfigurationEquals('model', 'standard'))

    oakd_lite_camera_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', LaunchConfiguration('world'),
             '/model/', LaunchConfiguration('robot_name'),
             '/link/oakd_pro_link/sensor/rgbd_camera/image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', LaunchConfiguration('world'),
             '/model/', LaunchConfiguration('robot_name'),
             '/link/oakd_pro_link/sensor/rgbd_camera/depth_image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', LaunchConfiguration('world'),
             '/model/', LaunchConfiguration('robot_name'),
             '/link/oakd_pro_link/sensor/rgbd_camera/points' +
             '@sensor_msgs/msg/PointCloud2' +
             '[ignition.msgs.PointCloudPacked'],
            ['/world/', LaunchConfiguration('world'),
             '/model/', LaunchConfiguration('robot_name'),
             '/link/oakd_pro_link/sensor/rgbd_camera/camera_info' +
             '@sensor_msgs/msg/CameraInfo' +
             '[ignition.msgs.CameraInfo'],
                ],
        remappings=[
            (['/world/', LaunchConfiguration('world'),
              '/model/', LaunchConfiguration('robot_name'),
              '/link/oakd_pro_link/sensor/rgbd_camera/image'],
             '/color/image'),
            (['/world/', LaunchConfiguration('world'),
              '/model/', LaunchConfiguration('robot_name'),
              '/link/oakd_pro_link/sensor/rgbd_camera/depth_image'],
             '/stereo/depth'),
            (['/world/', LaunchConfiguration('world'),
              '/model/', LaunchConfiguration('robot_name'),
              '/link/oakd_pro_link/sensor/rgbd_camera/points'],
             '/stereo/depth/points'),
            (['/world/', LaunchConfiguration('world'),
              '/model/', LaunchConfiguration('robot_name'),
              '/link/oakd_pro_link/sensor/rgbd_camera/camera_info'],
             '/color/camera_info')
                ],
        condition=LaunchConfigurationEquals('model', 'lite'))

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(create3_bridge)
    ld.add_action(hmi_display_msg_bridge)
    ld.add_action(hmi_buttons_msg_bridge)
    ld.add_action(hmi_led_msg_bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(oakd_pro_camera_bridge)
    ld.add_action(oakd_lite_camera_bridge)
    return ld
