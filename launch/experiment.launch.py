#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package = get_package_share_directory('uwb_feature_gtsam_sim')
    frontier = get_package_share_directory('frontier_ws')
    world = os.path.join(package, 'worlds', 'uwb_feature_world.world')
    params = os.path.join(package, 'config', 'experiment.yaml')
    urdf_path = os.path.join(
        package, 'urdf', 'turtlebot3_waffle_pi_tf.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    start_frontier = LaunchConfiguration('start_frontier')
    gui = LaunchConfiguration('gui')
    actions = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('start_frontier', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                frontier, 'launch', 'turtlebot3_house_multi.launch.py')),
            launch_arguments={'world': world, 'gui': gui}.items()),
    ]

    for robot in ('tb3_0', 'tb3_1'):
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot,
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
                'frame_prefix': f'{robot}/',
            }]))
        actions.append(TimerAction(period=12.0, actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            namespace=robot,
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': f'{robot}/odom',
                'map_frame': f'{robot}/map',
                'base_frame': f'{robot}/base_footprint',
                'scan_topic': f'/{robot}/scan',
                'resolution': 0.05,
                'max_laser_range': 3.5,
                'min_laser_range': 0.12,
                'minimum_travel_distance': 0.08,
                'minimum_travel_heading': 0.08,
                'map_update_interval': 1.0,
                'transform_publish_period': 0.03,
                'mode': 'mapping'}],
            remappings=[
                ('scan', f'/{robot}/scan'),
                ('/map', f'/{robot}/map'),
                ('/map_metadata', f'/{robot}/map_metadata')])]))
        actions.append(TimerAction(period=12.0, actions=[Node(
            package='uwb_feature_gtsam_sim',
            executable='uwb_range_sim',
            name=f'uwb_range_sim_{robot}',
            output='screen',
            parameters=[params, {'robot_name': robot}])]))
        actions.append(TimerAction(period=12.0, actions=[Node(
            package='uwb_feature_gtsam_sim',
            executable='circle_feature_detector',
            name=f'circle_feature_detector_{robot}',
            output='screen',
            parameters=[params, {'robot_name': robot}])]))
    fusion = Node(
        package='uwb_feature_gtsam_sim',
        executable='gtsam_fusion',
        name='gtsam_fusion',
        output='screen',
        parameters=[params])
    waiter = Node(
        package='uwb_feature_gtsam_sim',
        executable='alignment_waiter',
        name='alignment_waiter',
        output='screen',
        parameters=[{'use_sim_time': True}])
    merge = Node(
        package='merge_map',
        executable='merge_map',
        name='experiment_world_merge',
        output='screen',
        parameters=[{'use_sim_time': True}])

    frontier_params = os.path.join(frontier, 'config', 'params.yaml')
    dwb_params = os.path.join(frontier, 'config', 'dwb_controller.yaml')
    controller_nodes = []
    for robot in ('tb3_0', 'tb3_1'):
        configured_dwb = ParameterFile(
            RewrittenYaml(
                source_file=dwb_params,
                root_key=robot,
                param_rewrites={
                    'use_sim_time': 'True',
                    'local_costmap.local_costmap.ros__parameters.'
                    'robot_base_frame': f'{robot}/base_footprint',
                    'local_costmap.local_costmap.ros__parameters.'
                    'obstacle_layer.scan.topic': f'/{robot}/scan',
                },
                convert_types=True),
            allow_substs=True)
        controller_nodes.extend([
            Node(
                package='nav2_controller',
                executable='controller_server',
                namespace=robot,
                name='controller_server',
                output='screen',
                condition=IfCondition(start_frontier),
                parameters=[configured_dwb],
                remappings=[('cmd_vel', 'cmd_vel_dwb')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace=robot,
                name='lifecycle_manager_controller',
                output='screen',
                condition=IfCondition(start_frontier),
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['controller_server'],
                }]),
        ])
    frontier_nodes = [
        Node(
            package='frontier_ws',
            executable='frontier_multi',
            namespace=robot,
            name='frontier_multi',
            output='screen',
            condition=IfCondition(start_frontier),
            parameters=[frontier_params, {
                'use_sim_time': True,
                'robot_id': robot,
                'map_topic': '/merge_map',
                'map_frame': 'world',
                'base_frame': f'{robot}/base_footprint',
                'global_frame': 'world',
                'local_map_topic': f'/{robot}/map',
            }])
        for robot in ('tb3_0', 'tb3_1')
    ]

    actions.extend([
        TimerAction(period=13.0, actions=[fusion, waiter]),
        RegisterEventHandler(OnProcessExit(
            target_action=waiter,
            on_exit=[
                merge,
                *controller_nodes,
                TimerAction(period=3.0, actions=frontier_nodes),
            ])),
    ])
    return LaunchDescription(actions)
