#!/usr/bin/env python3
"""Seed AMCL with the robot's current pose instead of a hard-coded origin.

The saved map is expressed in the same frame as the odometry that was running
while it was recorded, so the odom pose at start-up is the best initial guess.
Starting AMCL at a fixed (0, 0, 0) while the robot is somewhere else makes the
map look shifted/rotated relative to the laser scan and the particle filter
cannot recover on its own.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry


class InitialPoseFromOdom(Node):

    def __init__(self):
        super().__init__('initial_pose_from_odom')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.repeat = int(self.declare_parameter('repeat', 5).value)
        self.odom = None

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched)

        self.sent = 0
        self.timer = self.create_timer(0.5, self.tick)

    def odom_cb(self, msg):
        self.odom = msg

    def tick(self):
        if self.odom is None:
            return
        if self.sent >= self.repeat:
            self.get_logger().info(
                'initial pose published %d times, exiting' % self.sent)
            self.destroy_timer(self.timer)
            rclpy.shutdown()
            return

        pose = self.odom.pose.pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = pose.position.x
        msg.pose.pose.position.y = pose.position.y
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        msg.pose.pose.orientation = Quaternion(
            z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.pub.publish(msg)
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info(
                'seeding AMCL from odom: (%.2f, %.2f, yaw %.2f)'
                % (pose.position.x, pose.position.y, yaw))


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseFromOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
