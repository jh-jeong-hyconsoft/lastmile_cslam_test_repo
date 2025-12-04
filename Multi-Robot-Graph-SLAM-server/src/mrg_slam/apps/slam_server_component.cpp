// SPDX-License-Identifier: BSD-2-Clause
/**
 * @file slam_server_component.cpp
 * @brief CSLAM Server Backend Component (B-plan)
 * @details 
 *   - 로봇들로부터 KeyframeEvent 수신
 *   - 전역 그래프 구축 및 최적화
 *   - 로봇 간 루프 클로저 검출
 *   - 전역 맵 생성 및 발행
 */

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

// boost
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// g2o
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

// mrg_slam
#include <mrg_slam/edge.hpp>
#include <mrg_slam/graph_database.hpp>
#include <mrg_slam/graph_slam.hpp>
#include <mrg_slam/keyframe.hpp>
#include <mrg_slam/loop_detector.hpp>
#include <mrg_slam/map_cloud_generator.hpp>
#include <mrg_slam/markers_publisher.hpp>
#include <mrg_slam/ros_utils.hpp>

// mrg_slam_msgs
#include <mrg_slam_msgs/msg/keyframe_event.hpp>
#include <mrg_slam_msgs/msg/pose_with_name.hpp>
#include <mrg_slam_msgs/msg/slam_status.hpp>
#include <mrg_slam_msgs/srv/save_graph.hpp>
#include <mrg_slam_msgs/srv/save_map.hpp>
#include <mrg_slam_msgs/srv/publish_map.hpp>

// ROS2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace mrg_slam {

class SlamServerComponent : public rclcpp::Node {
public:
    typedef pcl::PointXYZI PointT;

    explicit SlamServerComponent( const rclcpp::NodeOptions& options ) :
        Node( "slam_server", options ),
        map_cloud_msg_( std::make_shared<sensor_msgs::msg::PointCloud2>() ),
        map_cloud_msg_update_required_( false )
    {
        // One-shot timer for initialization (to get shared_from_this())
        init_timer_ = create_wall_timer( 
            std::chrono::milliseconds( 100 ), 
            std::bind( &SlamServerComponent::onInit, this ) 
        );
    }

    virtual ~SlamServerComponent() {}

private:
    void onInit()
    {
        RCLCPP_INFO( get_logger(), "Initializing CSLAM Server Backend..." );
        init_timer_->cancel();

        initialize_params();

        // Initialize core components
        graph_slam_.reset( new GraphSLAM( get_parameter( "g2o_solver_type" ).as_string() ) );
        graph_slam_->set_save_graph( get_parameter( "save_graph" ).as_bool() );

        graph_database_.reset( new GraphDatabase( shared_from_this(), graph_slam_ ) );
        loop_detector_.reset( new LoopDetector( shared_from_this() ) );
        map_cloud_generator_.reset( new MapCloudGenerator() );

        // Subscribe to keyframe uploads from all robots
        keyframe_sub_ = create_subscription<mrg_slam_msgs::msg::KeyframeEvent>(
            "keyframe_upload", 
            rclcpp::QoS( 100 ).reliable(),
            std::bind( &SlamServerComponent::keyframe_callback, this, std::placeholders::_1 ) 
        );

        // Subscribe to robot initial poses
        robot_init_pose_sub_ = create_subscription<mrg_slam_msgs::msg::PoseWithName>(
            "robot_init_pose",
            rclcpp::QoS( 10 ).reliable().transient_local(),
            std::bind( &SlamServerComponent::robot_init_pose_callback, this, std::placeholders::_1 )
        );

        // Publishers
        map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>( 
            "map_points", 
            rclcpp::QoS( 1 ).transient_local() 
        );

        slam_status_pub_ = create_publisher<mrg_slam_msgs::msg::SlamStatus>( 
            "slam_status", 
            rclcpp::QoS( 16 ) 
        );

        // Timers
        optimization_timer_ = create_wall_timer(
            std::chrono::duration<double>( get_parameter( "graph_update_interval" ).as_double() ),
            std::bind( &SlamServerComponent::optimization_callback, this )
        );

        map_publish_timer_ = create_wall_timer(
            std::chrono::duration<double>( get_parameter( "map_cloud_update_interval" ).as_double() ),
            std::bind( &SlamServerComponent::map_publish_callback, this )
        );

        // Services
        save_graph_service_ = create_service<mrg_slam_msgs::srv::SaveGraph>(
            "save_graph",
            std::bind( &SlamServerComponent::save_graph_service, this, 
                      std::placeholders::_1, std::placeholders::_2 )
        );

        save_map_service_ = create_service<mrg_slam_msgs::srv::SaveMap>(
            "save_map",
            std::bind( &SlamServerComponent::save_map_service, this,
                      std::placeholders::_1, std::placeholders::_2 )
        );

        publish_map_service_ = create_service<mrg_slam_msgs::srv::PublishMap>(
            "publish_map",
            std::bind( &SlamServerComponent::publish_map_service, this,
                      std::placeholders::_1, std::placeholders::_2 )
        );

        // Markers publisher
        markers_pub_.onInit( shared_from_this() );

        // Log parameters
        print_ros2_parameters( get_node_parameters_interface(), get_logger() );

        RCLCPP_INFO( get_logger(), "CSLAM Server Backend initialized" );
    }

    void initialize_params()
    {
        // Server identity
        declare_parameter<std::string>( "own_name", "server" );
        declare_parameter<std::vector<std::string>>( "robot_names", std::vector<std::string>{ "robot1", "robot2", "robot3" } );

        // Frame IDs
        declare_parameter<std::string>( "map_frame_id", "world" );

        // Map generation
        declare_parameter<double>( "map_cloud_resolution", 0.1 );
        declare_parameter<int>( "map_cloud_min_points_per_voxel", 2 );
        declare_parameter<double>( "map_cloud_distance_far_thresh", 10000.0 );
        declare_parameter<double>( "map_cloud_update_interval", 30.0 );

        // Graph optimization
        declare_parameter<std::string>( "g2o_solver_type", "lm_var_cholmod" );
        declare_parameter<int>( "g2o_solver_num_iterations", 1024 );
        declare_parameter<bool>( "g2o_verbose", false );
        declare_parameter<bool>( "save_graph", true );
        declare_parameter<double>( "graph_update_interval", 5.0 );
        declare_parameter<int>( "max_keyframes_per_update", 20 );

        // Odometry edge
        declare_parameter<std::string>( "odometry_edge_robust_kernel", "NONE" );
        declare_parameter<double>( "odometry_edge_robust_kernel_size", 1.0 );

        // Loop closure
        declare_parameter<double>( "candidate_max_xy_distance", 15.0 );
        declare_parameter<double>( "accum_distance_thresh_same_robot", 10.0 );
        declare_parameter<double>( "accum_distance_thresh_other_robot", 5.0 );
        declare_parameter<double>( "fitness_score_max_range", 999999.0 );
        declare_parameter<double>( "fitness_score_thresh", 0.4 );
        declare_parameter<bool>( "use_planar_registration_guess", false );
        declare_parameter<bool>( "enable_loop_closure_consistency_check", true );
        declare_parameter<double>( "loop_closure_consistency_max_delta_trans", 0.5 );
        declare_parameter<double>( "loop_closure_consistency_max_delta_angle", 10.0 );
        declare_parameter<std::string>( "loop_closure_edge_robust_kernel", "Huber" );
        declare_parameter<double>( "loop_closure_edge_robust_kernel_size", 1.0 );

        // Registration
        declare_parameter<std::string>( "registration_method", "FAST_GICP" );
        declare_parameter<int>( "reg_num_threads", 0 );
        declare_parameter<double>( "reg_transformation_epsilon", 0.1 );
        declare_parameter<int>( "reg_maximum_iterations", 128 );
        declare_parameter<double>( "reg_max_correspondence_distance", 2.5 );
        declare_parameter<int>( "reg_max_optimizer_iterations", 20 );
        declare_parameter<bool>( "reg_use_reciprocal_correspondences", false );
        declare_parameter<int>( "reg_correspondence_randomness", 20 );
        declare_parameter<double>( "reg_resolution", 1.0 );
        declare_parameter<std::string>( "reg_nn_search_method", "DIRECT7" );

        // Information matrix
        declare_parameter<bool>( "use_const_inf_matrix", false );
        declare_parameter<double>( "const_stddev_x", 0.5 );
        declare_parameter<double>( "const_stddev_q", 0.1 );
        declare_parameter<double>( "var_gain_a", 20.0 );
        declare_parameter<double>( "min_stddev_x", 0.1 );
        declare_parameter<double>( "max_stddev_x", 5.0 );
        declare_parameter<double>( "min_stddev_q", 0.05 );
        declare_parameter<double>( "max_stddev_q", 0.2 );

        // Output
        declare_parameter<std::string>( "result_dir", "/data/graphs" );
    }

    /**
     * @brief Callback for KeyframeEvent from robots
     */
    void keyframe_callback( const mrg_slam_msgs::msg::KeyframeEvent::SharedPtr msg )
    {
        std::lock_guard<std::mutex> lock( main_mutex_ );

        // Validation
        if( msg->robot_name.empty() ) {
            RCLCPP_WARN( get_logger(), "Keyframe has empty robot_name, skipping" );
            return;
        }

        if( msg->cloud.data.empty() ) {
            RCLCPP_WARN( get_logger(), "Keyframe has empty cloud, skipping" );
            return;
        }

        // Get robot's initial pose for frame transformation
        Eigen::Isometry3d world_pose;
        tf2::fromMsg( msg->estimate, world_pose );

        // Apply initial pose offset if available
        auto it = robot_init_poses_.find( msg->robot_name );
        if( it != robot_init_poses_.end() ) {
            world_pose = it->second * world_pose;
        }

        // Add to graph database
        graph_database_->add_external_keyframe( msg, world_pose );

        keyframe_count_++;
        RCLCPP_INFO( get_logger(), "Received keyframe [%d]: robot=%s, uuid=%s, first=%s",
                    keyframe_count_, msg->robot_name.c_str(), 
                    msg->keyframe_uuid.substr( 0, 8 ).c_str(),
                    msg->first_keyframe ? "true" : "false" );
    }

    /**
     * @brief Callback for robot initial pose
     */
    void robot_init_pose_callback( const mrg_slam_msgs::msg::PoseWithName::SharedPtr msg )
    {
        std::lock_guard<std::mutex> lock( main_mutex_ );

        Eigen::Isometry3d pose;
        tf2::fromMsg( msg->pose, pose );

        robot_init_poses_[msg->robot_name] = pose;

        RCLCPP_INFO( get_logger(), "Received initial pose for robot %s: [%.2f, %.2f, %.2f]",
                    msg->robot_name.c_str(),
                    pose.translation().x(),
                    pose.translation().y(),
                    pose.translation().z() );
    }

    /**
     * @brief Periodic graph optimization
     */
    void optimization_callback()
    {
        std::lock_guard<std::mutex> lock( main_mutex_ );

        // Process external keyframe queue
        bool keyframe_updated = graph_database_->flush_external_keyframe_queue();

        if( !keyframe_updated ) {
            return;
        }

        const auto& keyframes = graph_database_->get_keyframes();
        const auto& edges = graph_database_->get_edges();
        const auto& new_keyframes = graph_database_->get_new_keyframes();

        if( new_keyframes.empty() ) {
            return;
        }

        // Publish status
        mrg_slam_msgs::msg::SlamStatus status_msg;
        status_msg.robot_name = get_parameter( "own_name" ).as_string();
        status_msg.in_loop_closure = true;
        slam_status_pub_->publish( status_msg );

        // Loop detection
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Loop::Ptr> loops = loop_detector_->detect( graph_database_ );
        graph_database_->insert_loops( loops );
        auto end = std::chrono::high_resolution_clock::now();

        double loop_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();

        // Optimize graph
        status_msg.in_loop_closure = false;
        status_msg.in_optimization = true;
        slam_status_pub_->publish( status_msg );

        start = std::chrono::high_resolution_clock::now();
        int num_iterations = graph_slam_->optimize( 
            get_parameter( "g2o_solver_num_iterations" ).as_int(),
            get_parameter( "g2o_verbose" ).as_bool() 
        );
        end = std::chrono::high_resolution_clock::now();

        double opt_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();

        if( num_iterations < 0 ) {
            RCLCPP_WARN( get_logger(), "Graph optimization failed" );
        } else {
            RCLCPP_INFO( get_logger(), "Optimization: keyframes=%zu, edges=%zu, loops=%zu, "
                        "loop_time=%.1fms, opt_time=%.1fms, iterations=%d",
                        graph_database_->get_keyframes().size(),
                        graph_database_->get_edges().size(),
                        loops.size(),
                        loop_time_ms, opt_time_ms, num_iterations );
        }

        // Update keyframe snapshots for map generation
        auto marginals = graph_slam_->compute_marginals();
        const auto& all_keyframes = graph_database_->get_keyframes();

        std::vector<KeyFrameSnapshot::Ptr> snapshots( all_keyframes.size() );
        std::transform( all_keyframes.begin(), all_keyframes.end(), snapshots.begin(),
                       [=]( const KeyFrame::Ptr& k ) { 
                           return std::make_shared<KeyFrameSnapshot>( k, marginals ); 
                       } );

        {
            std::lock_guard<std::mutex> lock( snapshots_mutex_ );
            keyframes_snapshot_.swap( snapshots );
            map_cloud_msg_update_required_ = true;
        }

        // Save keyframe poses
        graph_database_->save_keyframe_poses();

        // Publish markers
        if( markers_pub_.getNumSubscribers() ) {
            markers_pub_.publish( graph_slam_, graph_database_->get_slam_uuid(), 
                                 all_keyframes, edges, graph_database_->get_prev_robot_keyframe(), {} );
        }

        status_msg.in_optimization = false;
        slam_status_pub_->publish( status_msg );
    }

    /**
     * @brief Periodic map publishing
     */
    void map_publish_callback()
    {
        if( !map_points_pub_->get_subscription_count() ) {
            return;
        }

        std::lock_guard<std::mutex> lock( snapshots_mutex_ );

        if( !map_cloud_msg_update_required_ || keyframes_snapshot_.empty() ) {
            return;
        }

        auto cloud = map_cloud_generator_->generate( 
            keyframes_snapshot_,
            get_parameter( "map_cloud_resolution" ).as_double(),
            get_parameter( "map_cloud_min_points_per_voxel" ).as_int(),
            get_parameter( "map_cloud_distance_far_thresh" ).as_double(),
            false 
        );

        if( !cloud ) {
            return;
        }

        cloud->header.frame_id = get_parameter( "map_frame_id" ).as_string();
        cloud->header.stamp = keyframes_snapshot_.back()->cloud->header.stamp;

        pcl::toROSMsg( *cloud, *map_cloud_msg_ );
        map_points_pub_->publish( *map_cloud_msg_ );

        map_cloud_msg_update_required_ = false;

        RCLCPP_INFO( get_logger(), "Published map: %zu points", cloud->size() );
    }

    // Service handlers
    void save_graph_service( const mrg_slam_msgs::srv::SaveGraph::Request::ConstSharedPtr req,
                             mrg_slam_msgs::srv::SaveGraph::Response::SharedPtr res )
    {
        std::lock_guard<std::mutex> lock( main_mutex_ );

        std::string directory = req->directory;
        if( directory.back() == '/' ) {
            directory.pop_back();
        }

        RCLCPP_INFO( get_logger(), "Saving graph to: %s", directory.c_str() );

        // Create directories
        std::string keyframe_dir = directory + "/keyframes";
        std::string edge_dir = directory + "/edges";
        std::string g2o_dir = directory + "/g2o";

        boost::filesystem::create_directories( keyframe_dir );
        boost::filesystem::create_directories( edge_dir );
        boost::filesystem::create_directories( g2o_dir );

        // Save keyframes
        const auto& keyframes = graph_database_->get_keyframes();
        for( size_t i = 0; i < keyframes.size(); ++i ) {
            std::stringstream ss;
            ss << keyframe_dir << "/" << std::setfill( '0' ) << std::setw( 6 ) << i;
            keyframes[i]->save( ss.str() );
        }

        // Save edges
        const auto& edges = graph_database_->get_edges();
        for( size_t i = 0; i < edges.size(); ++i ) {
            std::stringstream ss;
            ss << edge_dir << "/" << std::setfill( '0' ) << std::setw( 6 ) << i;
            edges[i]->save( ss.str() );
        }

        // Save g2o
        graph_slam_->save( g2o_dir + "/graph.g2o" );

        res->success = true;
        RCLCPP_INFO( get_logger(), "Graph saved: %zu keyframes, %zu edges", 
                    keyframes.size(), edges.size() );
    }

    void save_map_service( const mrg_slam_msgs::srv::SaveMap::Request::ConstSharedPtr req,
                           mrg_slam_msgs::srv::SaveMap::Response::SharedPtr res )
    {
        std::lock_guard<std::mutex> lock( snapshots_mutex_ );

        if( keyframes_snapshot_.empty() ) {
            RCLCPP_WARN( get_logger(), "No keyframes to save" );
            res->success = false;
            return;
        }

        double resolution = req->resolution == 0 ? 
            get_parameter( "map_cloud_resolution" ).as_double() : req->resolution;
        int min_points = req->min_points_per_voxel < 0 ?
            get_parameter( "map_cloud_min_points_per_voxel" ).as_int() : req->min_points_per_voxel;
        double far_thresh = req->distance_far_thresh == 0 ?
            get_parameter( "map_cloud_distance_far_thresh" ).as_double() : req->distance_far_thresh;

        auto cloud = map_cloud_generator_->generate( 
            keyframes_snapshot_, resolution, min_points, far_thresh, req->skip_first_cloud 
        );

        if( !cloud ) {
            RCLCPP_WARN( get_logger(), "Failed to generate map" );
            res->success = false;
            return;
        }

        cloud->header.frame_id = get_parameter( "map_frame_id" ).as_string();

        // Ensure directory exists
        auto dir = boost::filesystem::path( req->file_path ).parent_path();
        if( !dir.empty() && !boost::filesystem::exists( dir ) ) {
            boost::filesystem::create_directories( dir );
        }

        int ret = pcl::io::savePCDFileBinary( req->file_path, *cloud );
        res->success = ( ret == 0 );

        if( res->success ) {
            RCLCPP_INFO( get_logger(), "Map saved to %s: %zu points", 
                        req->file_path.c_str(), cloud->size() );
        } else {
            RCLCPP_ERROR( get_logger(), "Failed to save map to %s", req->file_path.c_str() );
        }
    }

    void publish_map_service( const mrg_slam_msgs::srv::PublishMap::Request::ConstSharedPtr req,
                              mrg_slam_msgs::srv::PublishMap::Response::SharedPtr res )
    {
        std::lock_guard<std::mutex> lock( snapshots_mutex_ );

        if( keyframes_snapshot_.empty() ) {
            res->success = false;
            return;
        }

        double resolution = req->resolution == 0 ? 
            get_parameter( "map_cloud_resolution" ).as_double() : req->resolution;
        int min_points = req->min_points_per_voxel < 0 ?
            get_parameter( "map_cloud_min_points_per_voxel" ).as_int() : req->min_points_per_voxel;
        double far_thresh = req->distance_far_thresh == 0 ?
            get_parameter( "map_cloud_distance_far_thresh" ).as_double() : req->distance_far_thresh;

        auto cloud = map_cloud_generator_->generate( 
            keyframes_snapshot_, resolution, min_points, far_thresh, req->skip_first_cloud 
        );

        if( !cloud ) {
            res->success = false;
            return;
        }

        cloud->header.frame_id = req->frame_id.empty() ? 
            get_parameter( "map_frame_id" ).as_string() : req->frame_id;
        cloud->header.stamp = keyframes_snapshot_.back()->cloud->header.stamp;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg( *cloud, msg );
        map_points_pub_->publish( msg );

        res->success = true;
    }

private:
    // Timers
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr optimization_timer_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;

    // Subscribers
    rclcpp::Subscription<mrg_slam_msgs::msg::KeyframeEvent>::SharedPtr keyframe_sub_;
    rclcpp::Subscription<mrg_slam_msgs::msg::PoseWithName>::SharedPtr robot_init_pose_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    rclcpp::Publisher<mrg_slam_msgs::msg::SlamStatus>::SharedPtr slam_status_pub_;

    // Services
    rclcpp::Service<mrg_slam_msgs::srv::SaveGraph>::SharedPtr save_graph_service_;
    rclcpp::Service<mrg_slam_msgs::srv::SaveMap>::SharedPtr save_map_service_;
    rclcpp::Service<mrg_slam_msgs::srv::PublishMap>::SharedPtr publish_map_service_;

    // Markers
    MarkersPublisher markers_pub_;

    // Core components
    std::shared_ptr<GraphSLAM> graph_slam_;
    std::shared_ptr<GraphDatabase> graph_database_;
    std::unique_ptr<LoopDetector> loop_detector_;
    std::unique_ptr<MapCloudGenerator> map_cloud_generator_;

    // Robot initial poses (for frame transformation)
    std::unordered_map<std::string, Eigen::Isometry3d> robot_init_poses_;

    // Map cloud
    std::mutex snapshots_mutex_;
    std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot_;
    sensor_msgs::msg::PointCloud2::SharedPtr map_cloud_msg_;
    std::atomic_bool map_cloud_msg_update_required_;

    // Thread safety
    std::mutex main_mutex_;

    // Statistics
    int keyframe_count_ = 0;
};

}  // namespace mrg_slam

RCLCPP_COMPONENTS_REGISTER_NODE( mrg_slam::SlamServerComponent )

