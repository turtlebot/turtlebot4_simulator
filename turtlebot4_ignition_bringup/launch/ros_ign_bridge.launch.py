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
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    DeclareLaunchArgument('namespace', default_value='',
                          description='robot namespace'),
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

    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    create3_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ros_ignition_bridge.launch.py'])

    create3_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ros_ign_bridge_launch]),
        launch_arguments=[
            ('robot_name', robot_name),
            ('world', world),
            ('namespace', namespace)
        ]
    )

    # lidar bridge
    lidar_bridge = Node(
        condition=LaunchConfigurationEquals('namespace', ''),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/world/', world,
             '/model/', robot_name,
             '/link/rplidar_link/sensor/rplidar/scan' +
             '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/rplidar_link/sensor/rplidar/scan'],
             '/scan')
        ])

    lidar_bridge_namespaced = Node(
        condition=LaunchConfigurationNotEquals('namespace', ''),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/world/', world,
             '/model/', robot_name,
             '/link/rplidar_link/sensor/rplidar/scan' +
             '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/rplidar_link/sensor/rplidar/scan'],
             ['/', namespace, '/scan'])
        ])

    # Display message bridge
    hmi_display_msg_bridge = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' == '' and '", model, "' == 'standard'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_display_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', robot_name, '/hmi/display/raw' +
             '@std_msgs/msg/String' +
             ']ignition.msgs.StringMsg'],
            ['/model/', robot_name, '/hmi/display/selected' +
             '@std_msgs/msg/Int32' +
             ']ignition.msgs.Int32']
        ],
        remappings=[
            (['/model/', robot_name, '/hmi/display/raw'],
             '/hmi/display/_raw'),
            (['/model/', robot_name, '/hmi/display/selected'],
             '/hmi/display/_selected')
        ])

    hmi_display_msg_bridge_namespaced = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' != '' and '", model, "' == 'standard'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_display_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', robot_name, '/hmi/display/raw' +
             '@std_msgs/msg/String' +
             ']ignition.msgs.StringMsg'],
            ['/model/', robot_name, '/hmi/display/selected' +
             '@std_msgs/msg/Int32' +
             ']ignition.msgs.Int32']
        ],
        remappings=[
            (['/model/', robot_name, '/hmi/display/raw'],
             ['/', namespace, '/hmi/display/_raw']),
            (['/model/', robot_name, '/hmi/display/selected'],
             ['/', namespace, '/hmi/display/_selected'])
        ])

    # Buttons message bridge
    hmi_buttons_msg_bridge = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' == '' and '", model, "' == 'standard'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_buttons_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', robot_name, '/hmi/buttons' +
             '@std_msgs/msg/Int32' +
             '[ignition.msgs.Int32']
        ],
        remappings=[
            (['/model/', robot_name, '/hmi/buttons'],
             '/hmi/buttons/_set')
        ])

    hmi_buttons_msg_bridge_namespaced = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' != '' and '", model, "' == 'standard'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_buttons_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', robot_name, '/hmi/buttons' +
             '@std_msgs/msg/Int32' +
             '[ignition.msgs.Int32']
        ],
        remappings=[
            (['/model/', robot_name, '/hmi/buttons'],
             ['/', namespace, '/hmi/buttons/_set'])
        ])

    # Led message bridge
    hmi_led_msg_bridge = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' == '' and '", model, "' == 'standard'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_led_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', robot_name, '/hmi/led/' + led +
             '@std_msgs/msg/Int32' +
             ']ignition.msgs.Int32'] for led in leds
        ],
        remappings=[
            (['/model/', robot_name, '/hmi/led/' + led],
             '/hmi/led/_' + led) for led in leds
        ])

    hmi_led_msg_bridge_namespaced = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' != '' and '", model, "' == 'standard'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='hmi_led_msg_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/model/', robot_name, '/hmi/led/' + led +
             '@std_msgs/msg/Int32' +
             ']ignition.msgs.Int32'] for led in leds
        ],
        remappings=[
            (['/model/', robot_name, '/hmi/led/' + led],
             ['/', namespace, '/hmi/led/_' + led]) for led in leds
        ])

    # Camera sensor bridge
    oakd_pro_camera_bridge = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' == '' and '", model, "' == 'standard'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', world,
                '/model/', robot_name,
                '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/', world,
                '/model/', robot_name,
                '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/depth_image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/', world,
                '/model/', robot_name,
                '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/points' +
                '@sensor_msgs/msg/PointCloud2' +
                '[ignition.msgs.PointCloudPacked'],
            ['/world/', world,
                '/model/', robot_name,
                '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/camera_info' +
                '@sensor_msgs/msg/CameraInfo' +
                '[ignition.msgs.CameraInfo'],
                ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/image'],
             '/color/image'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/depth_image'],
             '/stereo/depth'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/points'],
             '/stereo/depth/points'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/camera_info'],
             '/color/camera_info')
        ])
    
    oakd_pro_camera_bridge_namespaced = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' != '' and '", model, "' == 'standard'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', world,
                '/model/', robot_name,
                '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/', world,
                '/model/', robot_name,
                '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/depth_image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/', world,
                '/model/', robot_name,
                '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/points' +
                '@sensor_msgs/msg/PointCloud2' +
                '[ignition.msgs.PointCloudPacked'],
            ['/world/', world,
                '/model/', robot_name,
                '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/camera_info' +
                '@sensor_msgs/msg/CameraInfo' +
                '[ignition.msgs.CameraInfo'],
                ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/image'],
             ['/', namespace, '/color/image']),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/depth_image'],
             ['/', namespace, '/stereo/depth']),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/points'],
             ['/', namespace, '/stereo/depth/points']),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_pro_rgb_camera_frame/sensor/rgbd_camera/camera_info'],
             ['/', namespace, '/color/camera_info'])
        ])

    oakd_lite_camera_bridge = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' == '' and '", model, "' == 'lite'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/depth_image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/points' +
             '@sensor_msgs/msg/PointCloud2' +
             '[ignition.msgs.PointCloudPacked'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/camera_info' +
             '@sensor_msgs/msg/CameraInfo' +
             '[ignition.msgs.CameraInfo'],
                ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/image'],
             '/color/image'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/depth_image'],
             '/stereo/depth'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/points'],
             '/stereo/depth/points'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/camera_info'],
             '/color/camera_info')
        ])

    oakd_lite_camera_bridge_namespaced = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' != '' and '", model, "' == 'lite'"])),
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/depth_image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/points' +
             '@sensor_msgs/msg/PointCloud2' +
             '[ignition.msgs.PointCloudPacked'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/camera_info' +
             '@sensor_msgs/msg/CameraInfo' +
             '[ignition.msgs.CameraInfo'],
                ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/image'],
             ['/', namespace, '/color/image']),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/depth_image'],
             ['/', namespace, '/stereo/depth']),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/points'],
             ['/', namespace, '/stereo/depth/points']),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_lite_rgb_camera_frame/sensor/rgbd_camera/camera_info'],
             ['/', namespace, '/color/camera_info'])
        ])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(create3_bridge)
    ld.add_action(hmi_display_msg_bridge)
    ld.add_action(hmi_buttons_msg_bridge)
    ld.add_action(hmi_led_msg_bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(oakd_pro_camera_bridge)
    ld.add_action(oakd_lite_camera_bridge)
    ld.add_action(hmi_display_msg_bridge_namespaced)
    ld.add_action(hmi_buttons_msg_bridge_namespaced)
    ld.add_action(hmi_led_msg_bridge_namespaced)
    ld.add_action(lidar_bridge_namespaced)
    ld.add_action(oakd_pro_camera_bridge_namespaced)
    ld.add_action(oakd_lite_camera_bridge_namespaced)
    return ld
