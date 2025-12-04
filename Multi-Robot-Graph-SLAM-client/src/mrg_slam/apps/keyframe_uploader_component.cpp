// SPDX-License-Identifier: BSD-2-Clause

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <mrg_slam_msgs/msg/keyframe_event.hpp>

namespace mrg_slam {

/**
 * @brief Keyframe Uploader Component (B-plan)
 * Rate-limits keyframe_event messages for server upload
 */
class KeyframeUploaderComponent : public rclcpp::Node {
public:
    KeyframeUploaderComponent( const rclcpp::NodeOptions& options ) : Node( "keyframe_uploader", options )
    {
        RCLCPP_INFO( get_logger(), "Initializing keyframe_uploader component..." );

        // Parameters
        declare_parameter<double>( "rate_limit", 0.3 );  // Hz
        declare_parameter<bool>( "enable_upload", true );

        rate_limit_    = get_parameter( "rate_limit" ).as_double();
        enable_upload_ = get_parameter( "enable_upload" ).as_bool();

        // QoS: KeyframeEvent requires reliability
        auto qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ).reliable();

        // Subscriber: local SLAM keyframe_event (internal topic)
        sub_ = create_subscription<mrg_slam_msgs::msg::KeyframeEvent>(
            "slam/keyframe_event", qos,
            std::bind( &KeyframeUploaderComponent::keyframe_callback, this, std::placeholders::_1 ) );

        // Publisher: upload topic (zenoh bridge should relay THIS topic)
        // Different topic to avoid infinite loop
        pub_ = create_publisher<mrg_slam_msgs::msg::KeyframeEvent>( "slam_server/keyframe_upload", qos );

        last_upload_time_ = now();
        keyframe_count_   = 0;
        uploaded_count_   = 0;

        RCLCPP_INFO( get_logger(), "Keyframe Uploader initialized: rate_limit=%.2f Hz, enable=%s", 
                    rate_limit_, enable_upload_ ? "true" : "false" );
    }

    virtual ~KeyframeUploaderComponent() {}

private:
    /**
     * @brief KeyframeEvent callback with rate limiting and validation
     */
    void keyframe_callback( const mrg_slam_msgs::msg::KeyframeEvent::SharedPtr msg )
    {
        if( !enable_upload_ ) {
            return;
        }

        keyframe_count_++;

        // Validation
        if( msg->robot_name.empty() ) {
            RCLCPP_WARN( get_logger(), "Keyframe has empty robot_name, skipping" );
            return;
        }
        if( msg->keyframe_uuid.empty() ) {
            RCLCPP_WARN( get_logger(), "Keyframe has empty uuid, skipping" );
            return;
        }
        if( msg->cloud.data.empty() ) {
            RCLCPP_WARN( get_logger(), "Keyframe has empty point cloud, skipping" );
            return;
        }

        auto   now_time = now();
        double elapsed  = ( now_time - last_upload_time_ ).seconds();

        // Rate limit check
        double min_interval = rate_limit_ > 0.0 ? ( 1.0 / rate_limit_ ) : 0.0;
        if( elapsed < min_interval ) {
            RCLCPP_DEBUG( get_logger(), "Keyframe skipped (rate limit): elapsed=%.2fs < %.2fs", elapsed, min_interval );
            skipped_count_++;
            return;
        }

        // Upload to different topic (to be relayed by zenoh)
        try {
            pub_->publish( *msg );
            last_upload_time_ = now_time;
            uploaded_count_++;

            double cloud_size_kb = msg->cloud.data.size() / 1024.0;
            RCLCPP_INFO( get_logger(), "Uploaded keyframe: robot=%s, uuid=%s..., accum_dist=%.2fm, cloud=%.1fKB, total=%d/%d (skipped=%d)",
                        msg->robot_name.c_str(), msg->keyframe_uuid.substr( 0, 8 ).c_str(), msg->accum_distance, cloud_size_kb,
                        uploaded_count_, keyframe_count_, skipped_count_ );
        } catch( const std::exception& e ) {
            RCLCPP_ERROR( get_logger(), "Failed to upload keyframe: %s", e.what() );
        }
    }

private:
    rclcpp::Subscription<mrg_slam_msgs::msg::KeyframeEvent>::SharedPtr sub_;
    rclcpp::Publisher<mrg_slam_msgs::msg::KeyframeEvent>::SharedPtr    pub_;

    double       rate_limit_;
    bool         enable_upload_;
    rclcpp::Time last_upload_time_;
    int          keyframe_count_;
    int          uploaded_count_;
    int          skipped_count_ = 0;
};

}  // namespace mrg_slam

RCLCPP_COMPONENTS_REGISTER_NODE( mrg_slam::KeyframeUploaderComponent )

