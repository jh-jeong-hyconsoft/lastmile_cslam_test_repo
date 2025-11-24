#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    """
    Quaternion -> yaw (rad)
    """
    # q: geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomDiffMonitor(Node):
    def __init__(self):
        super().__init__('odom_diff_monitor')

        # 파라미터 (토픽 이름은 필요하면 변경해서 사용)
        self.declare_parameter('wheel_odom_topic', '/robot1/odom')
        self.declare_parameter('scan_odom_topic', '/robot1/scan_matching_odometry/odom')
        self.declare_parameter('max_time_diff', 0.05)  # [s] 허용 시간 차이

        wheel_topic = self.get_parameter('wheel_odom_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_odom_topic').get_parameter_value().string_value
        self.max_time_diff = self.get_parameter('max_time_diff').get_parameter_value().double_value

        self.get_logger().info(f'wheel_odom_topic: {wheel_topic}')
        self.get_logger().info(f'scan_odom_topic : {scan_topic}')
        self.get_logger().info(f'max_time_diff   : {self.max_time_diff} [s]')

        # 최근 메시지 저장용
        self.wheel_odom = None
        self.scan_odom = None

        # 구독자 등록
        self.wheel_sub = self.create_subscription(
            Odometry, wheel_topic, self.wheel_callback, 10
        )
        self.scan_sub = self.create_subscription(
            Odometry, scan_topic, self.scan_callback, 10
        )

        # 주기적으로 diff 출력 (0.2초마다)
        self.timer = self.create_timer(0.2, self.print_diff_timer)

    def wheel_callback(self, msg: Odometry):
        self.wheel_odom = msg

    def scan_callback(self, msg: Odometry):
        self.scan_odom = msg

    def print_diff_timer(self):
        if self.wheel_odom is None or self.scan_odom is None:
            return

        # 시각 차이 계산
        tw = self.wheel_odom.header.stamp
        ts = self.scan_odom.header.stamp

        t_w = tw.sec + tw.nanosec * 1e-9
        t_s = ts.sec + ts.nanosec * 1e-9
        dt = t_s - t_w  # scan - wheel

        if abs(dt) > self.max_time_diff:
            self.get_logger().warn(
                f'Time mismatch: scan - wheel = {dt:.4f} s (> {self.max_time_diff:.3f})'
            )
            # 시간 차이가 너무 크면 위치 차이는 의미가 애매하므로 여기서 리턴해도 됨
            # return

        # 위치 차이
        pw = self.wheel_odom.pose.pose.position
        ps = self.scan_odom.pose.pose.position
        dx = ps.x - pw.x
        dy = ps.y - pw.y
        dz = ps.z - pw.z

        # yaw 차이
        qw = self.wheel_odom.pose.pose.orientation
        qs = self.scan_odom.pose.pose.orientation
        yaw_w = quat_to_yaw(qw)
        yaw_s = quat_to_yaw(qs)
        dyaw = yaw_s - yaw_w
        # -pi ~ pi 범위로 정규화
        while dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        while dyaw < -math.pi:
            dyaw += 2.0 * math.pi

        self.get_logger().info(
            f'Δt={dt:+.4f}s  '
            f'Δpos=({dx:+.3f}, {dy:+.3f}, {dz:+.3f}) m  '
            f'Δyaw={math.degrees(dyaw):+.2f} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomDiffMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()