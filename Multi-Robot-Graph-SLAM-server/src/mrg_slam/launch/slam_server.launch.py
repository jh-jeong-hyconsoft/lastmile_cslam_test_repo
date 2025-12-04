#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
"""
CSLAM Server Backend Launch (B-plan)

Usage:
    ros2 launch mrg_slam slam_server.launch.py
    ros2 launch mrg_slam slam_server.launch.py rviz:=true
    ros2 launch mrg_slam slam_server.launch.py config:=slam_server.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """Setup launch nodes with resolved parameters."""
    
    # Resolve launch configurations
    config_file = LaunchConfiguration('config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    enable_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    
    # Package directory
    pkg_share = get_package_share_directory('mrg_slam')
    
    # Resolve config file path
    if not os.path.isabs(config_file):
        config_file = os.path.join(pkg_share, 'config', config_file)
    
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    print(f"[slam_server] Config: {config_file}")
    print(f"[slam_server] use_sim_time: {use_sim_time}")
    
    # Robot configuration
    robot_names = ['robot1', 'robot2', 'robot3']
    
    nodes = []
    
    # Static transforms: world -> robot maps
    for robot_name in robot_names:
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_world_to_{robot_name}_map',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'world',
                    '--child-frame-id', f'{robot_name}/map'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )
    
    # world -> map transform
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_map',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'map'
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    )
    
    # SLAM Server Component
    slam_server_node = ComposableNode(
        package='mrg_slam',
        plugin='mrg_slam::SlamServerComponent',
        name='slam_server',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('keyframe_upload', '/slam_server/keyframe_upload'),
            ('robot_init_pose', '/slam_server/robot_init_pose'),
            ('map_points', '/slam_server/map_points'),
            ('slam_status', '/slam_server/slam_status'),
            ('save_graph', '/slam_server/save_graph'),
            ('save_map', '/slam_server/save_map'),
            ('publish_map', '/slam_server/publish_map'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    # Container for composable nodes
    slam_container = ComposableNodeContainer(
        name='slam_server_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[slam_server_node],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    nodes.append(slam_container)
    
    # RViz2 (optional)
    if enable_rviz:
        rviz_config = os.path.join(pkg_share, 'rviz', 'slam_server.rviz')
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        )
        print(f"[slam_server] RViz enabled: {rviz_config}")
    
    print(f"[slam_server] Launching with {len(robot_names)} robot transforms")
    
    return nodes


def generate_launch_description():
    # Package directory for default config
    pkg_share = get_package_share_directory('mrg_slam')
    default_config = os.path.join(pkg_share, 'config', 'slam_server.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'config',
            default_value='slam_server.yaml',
            description='Configuration file name (in config/) or absolute path'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Setup nodes
        OpaqueFunction(function=launch_setup),
    ])
