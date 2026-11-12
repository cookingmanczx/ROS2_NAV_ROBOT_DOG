#!/usr/bin/env python3
"""Pose driver for the Gazebo Classical A2 model.

The model is spawned as a static entity, so it is moved exclusively by
integrating /cmd_vel and calling the gazebo_ros SetEntityState service.
A nav_msgs/Odometry message and the odom->base_link TF are published for
downstream consumers (slam_toolbox, nav2).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from tf2_ros import TransformBroadcaster


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class CmdVelDriver(Node):

    def __init__(self):
        super().__init__('cmd_vel_driver')

        self.entity_name = self.declare_parameter('entity_name', 'a2').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.publish_rate = float(self.declare_parameter('publish_rate', 30.0).value)
        self.max_linear = float(self.declare_parameter('max_linear', 0.5).value)
        self.max_angular = float(self.declare_parameter('max_angular', 1.0).value)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)
        self.initial_x = float(self.declare_parameter('initial_x', 0.0).value)
        self.initial_y = float(self.declare_parameter('initial_y', 0.0).value)
        self.initial_yaw = float(self.declare_parameter('initial_yaw', 0.0).value)
        self.spawn_z = float(self.declare_parameter('spawn_z', 0.374).value)

        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw

        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.cmd_stamp = None
        self.applied_linear = 0.0
        self.applied_angular = 0.0

        self.last_time = None

        self.set_state_client = self.create_client(
            SetEntityState, '/set_entity_state')
        self.pending_future = None
        self.last_service_warning = None

        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(
            "cmd_vel_driver ready: entity='%s', start=(%.2f, %.2f, %.2f), "
            "odom_frame='%s', base_frame='%s'"
            % (self.entity_name, self.x, self.y, self.yaw,
               self.odom_frame, self.base_frame))

    def cmd_vel_callback(self, msg):
        self.cmd_linear = max(-self.max_linear,
                              min(self.max_linear, msg.linear.x))
        self.cmd_angular = max(-self.max_angular,
                               min(self.max_angular, msg.angular.z))
        self.cmd_stamp = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            dt = 0.0
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now
        # Guard against clock jumps / long pauses
        dt = max(0.0, min(dt, 0.2))

        if (self.cmd_stamp is None or
                (now - self.cmd_stamp) > Duration(seconds=self.cmd_timeout)):
            self.applied_linear = 0.0
            self.applied_angular = 0.0
        else:
            self.applied_linear = self.cmd_linear
            self.applied_angular = self.cmd_angular

        if dt > 0.0:
            self.x += self.applied_linear * math.cos(self.yaw) * dt
            self.y += self.applied_linear * math.sin(self.yaw) * dt
            self.yaw = wrap_angle(self.yaw + self.applied_angular * dt)

        self.send_entity_state()

        stamp = now.to_msg()
        self.publish_odom(stamp)
        self.publish_tf(stamp)

    def send_entity_state(self):
        if not self.set_state_client.service_is_ready():
            self.warn_service_unavailable()
            return
        # Do not queue requests: one in flight at a time
        if self.pending_future is not None and not self.pending_future.done():
            return

        request = SetEntityState.Request()
        request.state.name = self.entity_name
        request.state.reference_frame = 'world'
        request.state.pose.position.x = self.x
        request.state.pose.position.y = self.y
        request.state.pose.position.z = self.spawn_z
        request.state.pose.orientation = self.quaternion_from_yaw(self.yaw)
        request.state.twist.linear.x = self.applied_linear
        request.state.twist.angular.z = self.applied_angular

        self.pending_future = self.set_state_client.call_async(request)

    def warn_service_unavailable(self):
        now = self.get_clock().now()
        if (self.last_service_warning is None or
                (now - self.last_service_warning).nanoseconds * 1e-9 > 5.0):
            self.last_service_warning = now
            self.get_logger().warn(
                '/set_entity_state service not available yet; '
                'pose is only integrated locally')

    @staticmethod
    def quaternion_from_yaw(yaw):
        return Quaternion(x=0.0, y=0.0,
                          z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def publish_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.spawn_z
        msg.pose.pose.orientation = self.quaternion_from_yaw(self.yaw)
        msg.twist.twist.linear.x = self.applied_linear
        msg.twist.twist.angular.z = self.applied_angular
        # 2D pose + twist only, everything else is unknown
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.02
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[35] = 0.05
        self.pub_odom.publish(msg)

    def publish_tf(self, stamp):
        self.tf_broadcaster.sendTransform(self._new_tf(stamp))

    def _new_tf(self, stamp):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.transform.translation.x = self.x
        msg.transform.translation.y = self.y
        msg.transform.translation.z = self.spawn_z
        msg.transform.rotation = self.quaternion_from_yaw(self.yaw)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDriver()
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
