#!/usr/bin/env python3
"""
Keyframe Uploader Node (B-plan)
로컬 SLAM의 keyframe_event를 레이트 리미팅하여 서버로 업로드
"""

import rclpy
from rclpy.node import Node
from mrg_slam_msgs.msg import KeyframeEvent
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class KeyframeUploaderNode(Node):
    def __init__(self):
        super().__init__('keyframe_uploader')
        
        # 파라미터
        self.declare_parameter('rate_limit', 0.3)  # Hz
        self.declare_parameter('enable_upload', True)
        
        self.rate_limit = self.get_parameter('rate_limit').value
        self.enable_upload = self.get_parameter('enable_upload').value
        
        # QoS: KeyframeEvent는 신뢰성 필수
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 구독: 로컬 SLAM의 keyframe_event (내부 토픽)
        self.sub = self.create_subscription(
            KeyframeEvent,
            'slam/keyframe_event',
            self.keyframe_callback,
            qos
        )
        
        # 발행: 업로드용 토픽 (zenoh 브리지가 중계)
        # 로봇 네임스페이스가 이미 적용되어 있으므로 그대로 사용
        self.pub = self.create_publisher(
            KeyframeEvent,
            'slam/keyframe_event',  # 동일 토픽 (zenoh allowlist로 구분)
            qos
        )
        
        self.last_upload_time = self.get_clock().now()
        self.keyframe_count = 0
        self.uploaded_count = 0
        
        self.get_logger().info(f'Keyframe Uploader initialized: rate_limit={self.rate_limit} Hz, enable={self.enable_upload}')
    
    def keyframe_callback(self, msg: KeyframeEvent):
        """
        KeyframeEvent 수신 및 레이트 리밋 적용
        """
        if not self.enable_upload:
            return
        
        self.keyframe_count += 1
        now = self.get_clock().now()
        elapsed = (now - self.last_upload_time).nanoseconds / 1e9
        
        # Rate limit check
        min_interval = 1.0 / self.rate_limit if self.rate_limit > 0 else 0.0
        if elapsed < min_interval:
            self.get_logger().debug(f'Keyframe skipped (rate limit): elapsed={elapsed:.2f}s < {min_interval:.2f}s')
            return
        
        # 업로드
        self.pub.publish(msg)
        self.last_upload_time = now
        self.uploaded_count += 1
        
        cloud_size_kb = len(msg.cloud.data) / 1024.0
        self.get_logger().info(
            f'Uploaded keyframe: robot={msg.robot_name}, uuid={msg.keyframe_uuid[:8]}..., '
            f'accum_dist={msg.accum_distance:.2f}m, cloud={cloud_size_kb:.1f}KB, '
            f'total={self.uploaded_count}/{self.keyframe_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = KeyframeUploaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()