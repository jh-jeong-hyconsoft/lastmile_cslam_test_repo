import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CONFIGS = {
    "indoor": os.path.join(BASE_PATH, "config", "robot1_indoor_ndt_ouster.yaml"),
    "outdoor": os.path.join(BASE_PATH, "config", "robot1_outdoor_ndt_ouster.yaml"),
}


def load_config(mode: str):
    """Load YAML config file based on mode (indoor/outdoor)."""
    if mode not in DEFAULT_CONFIGS:
        raise ValueError(f"Unknown mode '{mode}'. Use one of {list(DEFAULT_CONFIGS.keys())}.")

    config_file = DEFAULT_CONFIGS[mode]
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config not found for mode '{mode}': {config_file}")

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)
    print(f"[robot_full_b_plan_mode] Loaded config for mode '{mode}': {config_file}")
    return config


def generate_nodes(context, *args, **kwargs):
    """Create composable nodes using the selected mode's config."""
    mode = LaunchConfiguration("mode").perform(context)
    config = load_config(mode)

    # Sections
    shared_params = config["/**"]["ros__parameters"]
    prefilter_cfg = config["prefiltering_component"]["ros__parameters"]
    smo_cfg = config["scan_matching_odometry_component"]["ros__parameters"]
    slam_cfg = config["mrg_slam_component"]["ros__parameters"]
    uploader_cfg = config["keyframe_uploader"]["ros__parameters"]
    floor_cfg = config.get("floor_detection_component", {}).get("ros__parameters", {})

    robot_ns = shared_params["robot_namespace"]
    lidar_topic = shared_params["lidar_points_topic"]
    enable_slam_tf_broadcast = shared_params.get("enable_slam_tf_broadcast", False)

    if not slam_cfg.get("enable_mrg_slam", True):
        print("[robot_full_b_plan_mode] WARNING: mrg_slam_component.enable_mrg_slam is False!")

    # prefix frames with namespace
    prefilter_cfg["base_link_frame"] = f"{robot_ns}/{prefilter_cfg['base_link_frame']}"
    smo_cfg["odom_frame_id"] = f"{robot_ns}/{smo_cfg['odom_frame_id']}"
    smo_cfg["robot_odom_frame_id"] = f"{robot_ns}/{smo_cfg['robot_odom_frame_id']}"
    smo_cfg["map_frame_id"] = f"{robot_ns}/{smo_cfg['map_frame_id']}"

    slam_map_frame = slam_cfg.get("map_frame_id", "map")
    slam_odom_frame = slam_cfg.get("odom_frame_id", "odom")
    slam_cfg["map_frame_id"] = f"{robot_ns}/{slam_map_frame}"
    slam_cfg["odom_frame_id"] = f"{robot_ns}/{slam_odom_frame}"
    slam_cfg["own_name"] = robot_ns
    slam_cfg["multi_robot_names"] = [robot_ns]

    composable_nodes = []

    composable_nodes.append(
        ComposableNode(
            package="mrg_slam",
            plugin="mrg_slam::PrefilteringComponent",
            name="prefiltering_component",
            namespace=robot_ns,
            parameters=[prefilter_cfg, shared_params],
            remappings=[("velodyne_points", lidar_topic)],
        )
    )

    scan_params = dict(smo_cfg)
    scan_params["enable_tf_broadcast"] = enable_slam_tf_broadcast
    composable_nodes.append(
        ComposableNode(
            package="mrg_slam",
            plugin="mrg_slam::ScanMatchingOdometryComponent",
            name="scan_matching_odometry_component",
            namespace=robot_ns,
            parameters=[scan_params, shared_params],
        )
    )

    composable_nodes.append(
        ComposableNode(
            package="mrg_slam",
            plugin="mrg_slam::MrgSlamComponent",
            name="mrg_slam_component",
            namespace=robot_ns,
            parameters=[slam_cfg, shared_params],
        )
    )

    composable_nodes.append(
        ComposableNode(
            package="mrg_slam",
            plugin="mrg_slam::KeyframeUploaderComponent",
            name="keyframe_uploader",
            namespace=robot_ns,
            parameters=[uploader_cfg, shared_params],
        )
    )

    if floor_cfg:
        composable_nodes.append(
            ComposableNode(
                package="mrg_slam",
                plugin="mrg_slam::FloorDetectionComponent",
                name="floor_detection_component",
                namespace=robot_ns,
                parameters=[floor_cfg, shared_params],
            )
        )
    else:
        print("[robot_full_b_plan_mode] FloorDetectionComponent not launched (missing params).")

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
                "mode",
                default_value="indoor",
                description="Select config mode: indoor | outdoor",
            ),
            OpaqueFunction(function=generate_nodes),
        ]
    )
