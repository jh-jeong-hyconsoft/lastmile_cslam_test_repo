import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def load_config(context):
    """ROS2 형식 YAML 파라미터 파일 로드"""
    config_file = context.launch_configurations.get('config', 'robot1_front.yaml')

    config_paths = [
        os.path.join(os.getcwd(), config_file),
        os.path.join(os.path.dirname(__file__), '..', 'config', config_file),
        os.path.join(os.path.dirname(__file__), '..', 'configs', config_file),
        config_file,
    ]

    for path in config_paths:
        if os.path.exists(path):
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
            print(f"[robot_full_b_plan] Loaded config: {path}")
            return config

    raise FileNotFoundError(f"Config file not found: {config_file}")


def generate_nodes(context, *args, **kwargs):
    """ROS2 파라미터 파일 기반 노드 생성 (B안)"""
    config = load_config(context)

    # 섹션별 파라미터
    shared_params = config["/**"]["ros__parameters"]
    prefilter_cfg = config["prefiltering_component"]["ros__parameters"]
    smo_cfg = config["scan_matching_odometry_component"]["ros__parameters"]
    slam_cfg = config["mrg_slam_component"]["ros__parameters"]
    uploader_cfg = config["keyframe_uploader"]["ros__parameters"]

    # floor_detection_component 섹션은 선택적 처리
    floor_cfg = {}
    if "floor_detection_component" in config:
        floor_cfg = config["floor_detection_component"].get("ros__parameters", {})
    else:
        print("[robot_full_b_plan] floor_detection_component section not found in config. "
              "Floor detection node will not be launched.")

    # 공통 설정
    robot_ns = shared_params["robot_namespace"]
    lidar_topic = shared_params["lidar_points_topic"]
    wheel_odom_topic = shared_params["wheel_odom_topic"]
    enable_slam_tf_broadcast = shared_params.get("enable_slam_tf_broadcast", False)

    if not slam_cfg.get("enable_mrg_slam", True):
        print("[robot_full_b_plan] WARNING: mrg_slam_component.enable_mrg_slam is False! "
              "B-plan requires it to be True.")

    # 프레임 prefix 처리 (robot_namespace 붙이기)
    # base_link
    prefilter_cfg["base_link_frame"] = f"{robot_ns}/{prefilter_cfg['base_link_frame']}"

    # scan odom frames
    smo_cfg["odom_frame_id"] = f"{robot_ns}/{smo_cfg['odom_frame_id']}"
    smo_cfg["robot_odom_frame_id"] = f"{robot_ns}/{smo_cfg['robot_odom_frame_id']}"

    # map/odom frame (로컬 SLAM용)
    slam_map_frame = slam_cfg.get("map_frame_id", "map")
    slam_odom_frame = slam_cfg.get("odom_frame_id", "odom")
    slam_cfg["map_frame_id"] = f"{robot_ns}/{slam_map_frame}"
    slam_cfg["odom_frame_id"] = f"{robot_ns}/{slam_odom_frame}"

    # own_name, multi_robot_names 세팅 (단일 로봇 B안)
    slam_cfg["own_name"] = robot_ns
    slam_cfg["multi_robot_names"] = [robot_ns]

    composable_nodes = []

    # 1. Prefiltering Component
    composable_nodes.append(
        ComposableNode(
            package="mrg_slam",
            plugin="mrg_slam::PrefilteringComponent",
            name="prefiltering_component",
            namespace=robot_ns,
            parameters=[
                prefilter_cfg,
                shared_params,
            ],
            remappings=[
                ("velodyne_points", lidar_topic),
            ],
        )
    )

    # 2. ScanMatchingOdometryComponent
    scan_params = dict(smo_cfg)
    scan_params["enable_tf_broadcast"] = enable_slam_tf_broadcast

    composable_nodes.append(
        ComposableNode(
            package="mrg_slam",
            plugin="mrg_slam::ScanMatchingOdometryComponent",
            name="scan_matching_odometry_component",
            namespace=robot_ns,
            parameters=[
                scan_params,
                shared_params,
            ],
        )
    )

    # 3. MrgSlamComponent (로컬 SLAM)
    composable_nodes.append(
        ComposableNode(
            package="mrg_slam",
            plugin="mrg_slam::MrgSlamComponent",
            name="mrg_slam_component",
            namespace=robot_ns,
            parameters=[
                slam_cfg,
                shared_params,
            ],
        )
    )

    # 4. KeyframeUploaderComponent
    composable_nodes.append(
        ComposableNode(
            package="mrg_slam",
            plugin="mrg_slam::KeyframeUploaderComponent",
            name="keyframe_uploader",
            namespace=robot_ns,
            parameters=[
                uploader_cfg,
                shared_params,
            ],
        )
    )

    # 5. FloorDetectionComponent (선택적 – floor_cfg가 비어 있으면 생략)
    if floor_cfg:
        composable_nodes.append(
            ComposableNode(
                package="mrg_slam",
                plugin="mrg_slam::FloorDetectionComponent",
                name="floor_detection_component",
                namespace=robot_ns,
                parameters=[
                    floor_cfg,
                    shared_params,
                ],
                # 토픽은 모두 네임스페이스 내부 상대 이름:
                # - sub:  "prefiltering/filtered_points"
                # - pub:  "floor_detection/floor_coeffs"
                #         "floor_detection/floor_filtered_points"
                #         "floor_detection/floor_points"
                # PrefilteringComponent가 같은 네임스페이스에서
                # "prefiltering/filtered_points"를 내보내면 remap 불필요.
            )
        )
    else:
        print("[robot_full_b_plan] FloorDetectionComponent is NOT launched because "
              "floor_detection_component.ros__parameters is missing or empty.")

    # Container
    container = ComposableNodeContainer(
        name="mrg_slam_full_container",
        namespace=robot_ns,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
        parameters=[shared_params],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="robot1_front.yaml",
                description="B-plan ROS2 parameter file (e.g., robot1_front.yaml in configs/)",
            ),
            OpaqueFunction(function=generate_nodes),
        ]
    )