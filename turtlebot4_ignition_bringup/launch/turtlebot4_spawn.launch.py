from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node


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
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    # Paths
    turtlebot4_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ros_ign_bridge.launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    nav_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav_bringup.launch.py'])
    node_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_nodes.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_ignition_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ignition_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])
    dock_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'dock_description.launch.py'])

    # Parameters
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_ignition_bringup, 'config', 'turtlebot4_node.yaml']),
        description='Turtlebot4 Robot param file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_navigation, 'maps', 'depot.yaml']),
        description='Full path to map yaml file to load')

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    turtlebot4_node_yaml_file = LaunchConfiguration('param_file')
    robot_name = LaunchConfiguration('robot_name')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    namespaced_robot_description = [namespace, '/robot_description']
    namespaced_dock_description = [namespace, '/standard_dock_description']


    # Robot description
    robot_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments={'gazebo': 'ignition', 'namespace': namespace}.items())

    # Dock description     
    x_dock = OffsetParser(x, 0.157)
    yaw_dock = OffsetParser(yaw, 3.1416)
    dock_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_description_launch]),
        condition=IfCondition(LaunchConfiguration('spawn_dock')),
        # The robot starts docked
        launch_arguments={'x': x_dock, 'y': y, 'z': z, 'yaw': yaw_dock,
                          'namespace': namespace,
                          'gazebo': 'ignition'}.items()
    )

    # Spawn Turtlebot4
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            '-topic', namespaced_robot_description],
        output='screen')

    # Spawn dock
    spawn_dock = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', (robot_name, '_standard_dock'),
            '-x', x_dock,
            '-y', y,
            '-z', z,
            '-Y', yaw_dock,
            '-topic', namespaced_dock_description],
        output='screen')

    # ROS Ign bridge
    turtlebot4_ros_ign_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_ros_ign_bridge_launch]),
        launch_arguments=[('model', LaunchConfiguration('model')),
                          ('robot_name', robot_name),
                          ('namespace', namespace)]
    )

    # Rviz2
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch]),
        condition=LaunchConfigurationEquals('namespace', ''),
        launch_arguments=[('slam', LaunchConfiguration('slam')),
                          ('nav2', LaunchConfiguration('nav2')),
                          ('localization', LaunchConfiguration('localization')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time')),
                          ('map', LaunchConfiguration('map')),
                          ('use_namespace', 'false')]
    )

    navigation_namespaced = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch]),
        condition=LaunchConfigurationNotEquals('namespace', ''),
        launch_arguments=[('slam', LaunchConfiguration('slam')),
                          ('nav2', LaunchConfiguration('nav2')),
                          ('localization', LaunchConfiguration('localization')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time')),
                          ('map', LaunchConfiguration('map')),
                          ('namespace', namespace),
                          ('use_namespace', 'true')]
    )

    turtlebot4_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([node_launch]),
        launch_arguments=[('model', LaunchConfiguration('model')),
                          ('param_file', turtlebot4_node_yaml_file),
                          ('namespace', namespace)]
    )

    # Create3 nodes
    create3_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch]),
        launch_arguments=[('namespace', namespace)]
    )

    create3_ignition_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ignition_nodes_launch]),
        launch_arguments=[('robot_name', LaunchConfiguration('robot_name')),
                          ('namespace', namespace)]
    )

    # RPLIDAR static transforms
    rplidar_stf = Node(
            condition=LaunchConfigurationEquals('namespace', ''),
            name='rplidar_stf',
            namespace=namespace,
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0.0', '0.0',
                'rplidar_link', [LaunchConfiguration('robot_name'), '/rplidar_link/rplidar']]
        )

    rplidar_stf_namespaced = Node(
            condition=LaunchConfigurationNotEquals('namespace', ''),
            name='rplidar_stf',
            namespace=namespace,
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0.0', '0.0',
                ['/', namespace, '/rplidar_link'], [LaunchConfiguration('robot_name'), '/rplidar_link/rplidar']]
        )

    # OAKD static transforms
    oakd_pro_stf = Node(
            condition=IfCondition(PythonExpression(["'", namespace, "' == '' and '", model, "' == 'standard'"])),
            name='camera_stf',
            namespace=namespace,
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'oakd_pro_rgb_camera_optical_frame',
                [LaunchConfiguration('robot_name'), '/oakd_pro_rgb_camera_frame/rgbd_camera']
            ]
        )

    oakd_pro_stf_namespaced = Node(
            condition=IfCondition(PythonExpression(["'", namespace, "' != '' and '", model, "' == 'standard'"])),
            name='camera_stf',
            namespace=namespace,
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                ['/', namespace, '/oakd_pro_rgb_camera_optical_frame'],
                [LaunchConfiguration('robot_name'), '/oakd_pro_rgb_camera_frame/rgbd_camera']
            ],
        )

    oakd_lite_stf = Node(
            condition=IfCondition(PythonExpression(["'", namespace, "' == '' and '", model, "' == 'lite'"])),
            name='camera_stf',
            namespace=namespace,
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'oakd_lite_rgb_camera_optical_frame',
                [LaunchConfiguration('robot_name'), '/oakd_lite_rgb_camera_frame/rgbd_camera']
            ]
        )

    oakd_lite_stf_namespaced = Node(
        condition=IfCondition(PythonExpression(["'", namespace, "' != '' and '", model, "' == 'lite'"])),
        name='camera_stf',
        namespace=namespace,
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            ['/', namespace, '/oakd_lite_rgb_camera_optical_frame'],
            [LaunchConfiguration('robot_name'), '/oakd_lite_rgb_camera_frame/rgbd_camera']
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(turtlebot4_ros_ign_bridge)
    ld.add_action(rviz2)
    ld.add_action(robot_description_launch)
    ld.add_action(dock_description)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_dock)
    ld.add_action(create3_nodes)
    ld.add_action(create3_ignition_nodes)
    ld.add_action(turtlebot4_node)
    ld.add_action(navigation)
    ld.add_action(navigation_namespaced)
    ld.add_action(rplidar_stf)
    ld.add_action(oakd_pro_stf)
    ld.add_action(oakd_lite_stf)
    ld.add_action(rplidar_stf_namespaced)
    ld.add_action(oakd_pro_stf_namespaced)
    ld.add_action(oakd_lite_stf_namespaced)
    return ld
