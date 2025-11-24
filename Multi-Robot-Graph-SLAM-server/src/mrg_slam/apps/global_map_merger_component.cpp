// SPDX-License-Identifier: BSD-2-Clause
/**
 * @file global_map_merger_component.cpp
 * @brief Global Map Merger Component for multi-robot SLAM (B-plan)
 * @details 각 로봇의 최적화된 맵을 구독하여 전역 통합 맵 생성
 */

#include <mutex>
#include <unordered_map>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mrg_slam/ros_time_hash.hpp>
#include <mrg_slam_msgs/srv/save_map.hpp>

namespace mrg_slam {

class GlobalMapMergerComponent : public rclcpp::Node
{
public:
    using PointT = pcl::PointXYZI;

    explicit GlobalMapMergerComponent( const rclcpp::NodeOptions& options ) : Node( "global_map_merger", options )
    {
        RCLCPP_INFO( get_logger(), "Initializing GlobalMapMergerComponent..." );

        // Parameters
        declare_parameter<std::vector<std::string>>( "robots", std::vector<std::string>{ "robot1", "robot2", "robot3" } );
        declare_parameter<double>( "merge_interval", 30.0 );          // seconds
        declare_parameter<double>( "map_timeout", 60.0 );             // seconds (오래된 맵 제거)
        declare_parameter<double>( "downsample_resolution", 0.1 );    // meters
        declare_parameter<int>( "min_points_per_voxel", 1 );
        declare_parameter<std::string>( "global_frame_id", "world" );
        declare_parameter<bool>( "enable_auto_merge", true );         // 자동 병합 활성화
        declare_parameter<int>( "max_points_per_robot", 5000000 );    // 로봇당 최대 포인트 (메모리 보호)

        robots_                  = get_parameter( "robots" ).as_string_array();
        merge_interval_          = get_parameter( "merge_interval" ).as_double();
        map_timeout_             = get_parameter( "map_timeout" ).as_double();
        downsample_resolution_   = get_parameter( "downsample_resolution" ).as_double();
        min_points_per_voxel_    = get_parameter( "min_points_per_voxel" ).as_int();
        global_frame_id_         = get_parameter( "global_frame_id" ).as_string();
        enable_auto_merge_       = get_parameter( "enable_auto_merge" ).as_bool();
        max_points_per_robot_    = get_parameter( "max_points_per_robot" ).as_int();

        // 각 로봇의 맵 구독
        for( const auto& robot : robots_ ) {
            std::string topic = "/" + robot + "/mrg_slam/map_points";
            
            auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, rclcpp::QoS( 10 ).reliable(),
                [this, robot]( const sensor_msgs::msg::PointCloud2::SharedPtr msg ) { map_callback( msg, robot ); } );
            
            map_subscribers_.push_back( sub );
            map_cache_[robot] = nullptr;
            
            RCLCPP_INFO( get_logger(), "Subscribed to %s", topic.c_str() );
        }

        // 전역 맵 발행
        global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>( 
            "/global/map_merged", 
            rclcpp::QoS( 10 ).reliable().transient_local() 
        );

        // 주기적 병합 타이머
        if( enable_auto_merge_ ) {
            merge_timer_ = create_wall_timer( 
                std::chrono::duration<double>( merge_interval_ ),
                std::bind( &GlobalMapMergerComponent::merge_and_publish, this ) 
            );
            RCLCPP_INFO( get_logger(), "Auto-merge enabled: interval=%.1fs", merge_interval_ );
        }

        // 수동 병합 서비스
        merge_service_ = create_service<std_srvs::srv::Trigger>(
            "global_map_merger/trigger_merge",
            std::bind( &GlobalMapMergerComponent::trigger_merge_service, this, 
                      std::placeholders::_1, std::placeholders::_2 ) );

        save_map_service_ = create_service<mrg_slam_msgs::srv::SaveMap>(
            "global_map_merger/save_map",
            std::bind( &GlobalMapMergerComponent::save_map_service, this,
                      std::placeholders::_1, std::placeholders::_2 ) );

        RCLCPP_INFO( get_logger(), "GlobalMapMergerComponent initialized for %zu robots", robots_.size() );
    }

private:
    struct MapCache
    {
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        rclcpp::Time                             timestamp;
        size_t                                   point_count;
    };

    void map_callback( const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string& robot_name )
    {
        std::lock_guard<std::mutex> lock( map_cache_mutex_ );

        // 포인트 개수 확인 (메모리 보호)
        size_t point_count = msg->width * msg->height;
        if( static_cast<int>( point_count ) > max_points_per_robot_ ) {
            RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 5000,
                                 "Map from %s exceeds max points (%zu > %d), skipping",
                                 robot_name.c_str(), point_count, max_points_per_robot_ );
            return;
        }

        // 캐시 업데이트
        MapCache cache;
        cache.msg         = msg;
        cache.timestamp   = now();
        cache.point_count = point_count;
        
        map_cache_[robot_name] = std::make_shared<MapCache>( cache );

        RCLCPP_DEBUG( get_logger(), "Cached map from %s: %zu points, frame=%s",
                     robot_name.c_str(), point_count, msg->header.frame_id.c_str() );
    }

    void merge_and_publish()
    {
        std::lock_guard<std::mutex> lock( map_cache_mutex_ );

        auto now_time = now();

        // 유효한 맵 필터링 (타임아웃 체크)
        std::vector<std::pair<std::string, std::shared_ptr<MapCache>>> valid_maps;
        
        for( const auto& robot : robots_ ) {
            auto cache_ptr = map_cache_[robot];
            
            if( !cache_ptr || !cache_ptr->msg ) {
                RCLCPP_DEBUG( get_logger(), "No map from %s", robot.c_str() );
                continue;
            }

            // 타임아웃 체크
            double age = ( now_time - cache_ptr->timestamp ).seconds();
            if( age > map_timeout_ ) {
                RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 10000,
                                     "Map from %s is too old (%.1fs), skipping", robot.c_str(), age );
                continue;
            }

            valid_maps.emplace_back( robot, cache_ptr );
        }

        if( valid_maps.empty() ) {
            RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 10000, "No valid maps available for merging" );
            return;
        }

        RCLCPP_INFO( get_logger(), "Merging maps from %zu robots...", valid_maps.size() );

        // 통합 맵 생성
        pcl::PointCloud<PointT>::Ptr merged_cloud( new pcl::PointCloud<PointT>() );
        
        size_t total_points_before = 0;
        
        for( const auto& [robot, cache] : valid_maps ) {
            pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
            pcl::fromROSMsg( *cache->msg, *cloud );

            // 프레임 확인 (모든 맵이 같은 프레임이어야 함)
            // B-plan: 서버의 각 MrgSlamComponent는 이미 world 좌표계에서 최적화됨
            // 따라서 각 로봇의 map은 이미 정합된 상태
            
            *merged_cloud += *cloud;
            total_points_before += cloud->size();

            RCLCPP_INFO( get_logger(), "  %s: %zu points (frame=%s)", 
                        robot.c_str(), cloud->size(), cache->msg->header.frame_id.c_str() );
        }

        if( merged_cloud->empty() ) {
            RCLCPP_WARN( get_logger(), "Merged cloud is empty" );
            return;
        }

        RCLCPP_INFO( get_logger(), "Total points before downsampling: %zu", total_points_before );

        // 다운샘플링 (VoxelGrid)
        pcl::PointCloud<PointT>::Ptr filtered_cloud( new pcl::PointCloud<PointT>() );
        
        if( downsample_resolution_ > 0.0 ) {
            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setInputCloud( merged_cloud );
            voxel_filter.setLeafSize( downsample_resolution_, downsample_resolution_, downsample_resolution_ );
            voxel_filter.setMinimumPointsNumberPerVoxel( min_points_per_voxel_ );
            voxel_filter.filter( *filtered_cloud );

            RCLCPP_INFO( get_logger(), "Downsampled: %zu → %zu points (resolution=%.2fm)",
                        total_points_before, filtered_cloud->size(), downsample_resolution_ );
        } else {
            filtered_cloud = merged_cloud;
            RCLCPP_INFO( get_logger(), "Skipping downsampling (resolution=0)" );
        }

        // PointCloud2 메시지 생성 및 발행
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg( *filtered_cloud, output_msg );
        
        output_msg.header.stamp    = now_time;
        output_msg.header.frame_id = global_frame_id_;

        global_map_pub_->publish( output_msg );

        RCLCPP_INFO( get_logger(), "Published global map: %zu points in frame '%s'",
                    filtered_cloud->size(), global_frame_id_.c_str() );
    }

    void trigger_merge_service( const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res )
    {
        (void)req;
        
        RCLCPP_INFO( get_logger(), "Manual merge triggered via service" );
        
        try {
            merge_and_publish();
            res->success = true;
            res->message = "Map merge completed successfully";
        } catch( const std::exception& e ) {
            RCLCPP_ERROR( get_logger(), "Merge failed: %s", e.what() );
            res->success = false;
            res->message = std::string( "Merge failed: " ) + e.what();
        }
    }

    void save_map_service( const std::shared_ptr<mrg_slam_msgs::srv::SaveMap::Request>  req,
                           std::shared_ptr<mrg_slam_msgs::srv::SaveMap::Response> res )
    {
        if( req->file_path.empty() ) {
            RCLCPP_ERROR( get_logger(), "global_map_merger/save_map: file_path is empty" );
            res->success = false;
            return;
        }

        std::lock_guard<std::mutex> lock( map_cache_mutex_ );

        auto now_time = now();

        // 유효한 맵 필터링 (merge_and_publish와 동일 로직)
        std::vector<std::pair<std::string, std::shared_ptr<MapCache>>> valid_maps;
        for( const auto& robot : robots_ ) {
            auto cache_ptr = map_cache_[robot];
            if( !cache_ptr || !cache_ptr->msg ) {
                continue;
            }

            double age = ( now_time - cache_ptr->timestamp ).seconds();
            if( age > map_timeout_ ) {
                continue;
            }

            valid_maps.emplace_back( robot, cache_ptr );
        }

        if( valid_maps.empty() ) {
            RCLCPP_WARN( get_logger(), "global_map_merger/save_map: No valid maps available for saving" );
            res->success = false;
            return;
        }

        // 통합 맵 생성
        pcl::PointCloud<PointT>::Ptr merged_cloud( new pcl::PointCloud<PointT>() );
        size_t                        total_points_before = 0;

        for( const auto& [robot, cache] : valid_maps ) {
            pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
            pcl::fromROSMsg( *cache->msg, *cloud );
            *merged_cloud += *cloud;
            total_points_before += cloud->size();
        }

        if( merged_cloud->empty() ) {
            RCLCPP_WARN( get_logger(), "global_map_merger/save_map: Merged cloud is empty" );
            res->success = false;
            return;
        }

        // 해상도/voxel 파라미터는 노드 파라미터를 그대로 사용
        // (요청의 resolution, min_points_per_voxel 등은 무시)
        double resolution = downsample_resolution_;
        int    min_pts    = min_points_per_voxel_;

        pcl::PointCloud<PointT>::Ptr filtered_cloud( new pcl::PointCloud<PointT>() );
        if( resolution > 0.0 ) {
            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setInputCloud( merged_cloud );
            voxel_filter.setLeafSize( resolution, resolution, resolution );
            voxel_filter.setMinimumPointsNumberPerVoxel( min_pts );
            voxel_filter.filter( *filtered_cloud );
        } else {
            filtered_cloud = merged_cloud;
        }

        if( filtered_cloud->empty() ) {
            RCLCPP_WARN( get_logger(), "global_map_merger/save_map: Filtered cloud is empty, nothing to save" );
            res->success = false;
            return;
        }

        // PCD 저장
        int ret = pcl::io::savePCDFileBinary( req->file_path, *filtered_cloud );
        res->success = ( ret == 0 );

        if( res->success ) {
            RCLCPP_INFO( get_logger(),
                         "Saved global map to %s (%zu points, resolution=%.2f, min_pts=%d, total_before=%zu)",
                         req->file_path.c_str(), filtered_cloud->size(), resolution, min_pts, total_points_before );
        } else {
            RCLCPP_ERROR( get_logger(), "Failed to save global map to %s", req->file_path.c_str() );
        }
    }

private:
    // Parameters
    std::vector<std::string> robots_;
    double                   merge_interval_;
    double                   map_timeout_;
    double                   downsample_resolution_;
    int                      min_points_per_voxel_;
    std::string              global_frame_id_;
    bool                     enable_auto_merge_;
    int                      max_points_per_robot_;

    // ROS2 interfaces
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> map_subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                 global_map_pub_;
    rclcpp::TimerBase::SharedPtr                                                merge_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                          merge_service_;
    rclcpp::Service<mrg_slam_msgs::srv::SaveMap>::SharedPtr                     save_map_service_;

    // Map cache
    std::unordered_map<std::string, std::shared_ptr<MapCache>> map_cache_;
    std::mutex                                                 map_cache_mutex_;
};

}  // namespace mrg_slam

RCLCPP_COMPONENTS_REGISTER_NODE( mrg_slam::GlobalMapMergerComponent )

