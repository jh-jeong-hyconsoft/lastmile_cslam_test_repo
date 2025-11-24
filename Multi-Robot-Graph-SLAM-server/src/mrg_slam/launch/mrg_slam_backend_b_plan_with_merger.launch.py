#!/usr/bin/env python3
"""
서버 백엔드 B안 런치 (전역 맵 병합 포함)
- KeyframeEvent 수신
- 전역 그래프 병합 및 최적화
- 로봇 간 루프 클로저
- 전역 통합 맵 생성 ★
"""

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def load_config(context):
    """Config 파일 로드"""
    config_file = context.launch_configurations.get('config', 'backend_b_plan.yaml')
    
    config_paths = [
        os.path.join(os.getcwd(), config_file),
        os.path.join(os.path.dirname(__file__), '..', 'config', config_file),
        config_file,
    ]
    
    for path in config_paths:
        if os.path.exists(path):
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
            print(f"[backend_b_plan_with_merger] Loaded config: {path}")
            return config
    
    raise FileNotFoundError(f"Config file not found: {config_file}")


def generate_nodes(context, *args, **kwargs):
    """B안 백엔드 노드 생성 + 전역 맵 병합"""
    config = load_config(context)
    
    robots = ['robot1', 'robot2', 'robot3']
    global_cfg = config['global']
    opt_cfg = config['optimization']
    loop_cfg = config['loop_closure']
    
    # RViz 설정
    enable_rviz = context.launch_configurations.get('rviz', 'false').lower() == 'true'
    
    nodes = []
    
    # 1. 각 로봇별 MrgSlamComponent 인스턴스
    composable_nodes = []
    
    for robot_name in robots:
        composable_nodes.append(
            ComposableNode(
                package='mrg_slam',
                plugin='mrg_slam::MrgSlamComponent',
                name=f'{robot_name}_backend',
                namespace=robot_name,
                parameters=[{
                    'own_name': robot_name,
                    'multi_robot_names': robots,
                    'map_frame_id': f'{robot_name}/map',
                    'odom_frame_id': f'{robot_name}/odom',
                    
                    # 그래프 최적화
                    'g2o_solver_type': opt_cfg['g2o_solver_type'],
                    'g2o_solver_num_iterations': opt_cfg['g2o_solver_num_iterations'],
                    'graph_update_interval': opt_cfg['graph_update_interval'],
                    'save_graph': opt_cfg['save_graph'],
                    
                    # 루프 클로저
                    'candidate_max_xy_distance': loop_cfg['candidate_max_xy_distance'],
                    'accum_distance_thresh_same_robot': loop_cfg['accum_distance_thresh_same_robot'],
                    'accum_distance_thresh_other_robot': loop_cfg['accum_distance_thresh_other_robot'],
                    'fitness_score_thresh': loop_cfg['fitness_score_thresh'],
                    'registration_method': loop_cfg['registration_method'],
                    'reg_num_threads': loop_cfg['reg_num_threads'],
                    
                    # B안: 키프레임 이벤트 발행 비활성화 (서버는 수신만)
                    'enable_keyframe_event_publish': False,
                    # B안: 키프레임 이벤트 수신 활성화 (서버 모드)
                    'enable_keyframe_event_reception': True,
                }],
            )
        )
    
    # Container
    container = ComposableNodeContainer(
        name='mrg_slam_backend_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )
    nodes.append(container)
    
    # 2. Global Map Merger Component (C++) ★
    composable_nodes_merger = [
        ComposableNode(
            package='mrg_slam',
            plugin='mrg_slam::GlobalMapMergerComponent',
            name='global_map_merger',
            parameters=[{
                'robots': robots,
                'merge_interval': 30.0,             # 초 (자동 병합 주기)
                'map_timeout': 60.0,                # 초 (오래된 맵 무시)
                'downsample_resolution': 0.0,       # m (VoxelGrid)
                'min_points_per_voxel': 2,          # 최소 포인트 (노이즈 제거)
                'global_frame_id': 'world',         # 전역 프레임
                'enable_auto_merge': True,          # 자동 병합 활성화
                'max_points_per_robot': 5000000,    # 로봇당 최대 포인트 (메모리 보호)
            }],
        )
    ]
    
    merger_container = ComposableNodeContainer(
        name='global_map_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes_merger,
        output='screen',
    )
    nodes.append(merger_container)
    
    # 2.5. Static TF: world 프레임 발행 (RViz Fixed Frame용)
    # world <- robot1/map, robot2/map, robot3/map을 연결
    for robot_name in robots:
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_world_to_{robot_name}_map',
            arguments=[
                '0', '0', '0',  # x, y, z
                '0', '0', '0',  # roll, pitch, yaw
                'world',
                f'{robot_name}/map',
            ],
            output='log',
        )
        nodes.append(static_tf_node)
    
    # 3. RViz (선택적)
    if enable_rviz:
        rviz_config_path = os.path.join(
            os.path.dirname(__file__), '..', 'rviz', 'global_map_view.rviz'
        )
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        )
        nodes.append(rviz_node)
        print(f"[backend_b_plan_with_merger] RViz enabled: {rviz_config_path}")
    
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='backend_b_plan.yaml',
            description='Config file name (in deployment/server/configs/)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz for global map visualization'
        ),
        OpaqueFunction(function=generate_nodes),
    ])


