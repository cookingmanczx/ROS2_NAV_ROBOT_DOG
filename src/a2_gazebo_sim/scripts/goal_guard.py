#!/usr/bin/env python3
"""Validate NavigateToPose goals before they reach bt_navigator.

Why this exists
---------------
NavFn is allowed to substitute a nearby free cell when the requested goal is
blocked (``GridBased.tolerance``), and the controller's goal checker only
compares against the *end of the planned path*. A goal inside a wall or behind
furniture could therefore end as ``SUCCEEDED`` with the robot half a metre away
from the request, or spin through recovery cycles without ever terminating.

This node owns the public ``navigate_to_pose`` action and forwards valid goals
to bt_navigator (remapped to ``navigate_to_pose_impl``). Goals that are outside
the map, in unknown space, in an occupied cell or with less than
``min_clearance`` to the nearest obstacle are aborted immediately.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid

OCCUPIED_THRESHOLD = 65


class GoalGuard(Node):
    def __init__(self):
        super().__init__('goal_guard')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('min_clearance', 0.30)
        self.min_clearance = float(self.get_parameter('min_clearance').value)
        self.grid = None

        group = ReentrantCallbackGroup()
        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,
                                 self.get_parameter('map_topic').value,
                                 self.on_map, map_qos, callback_group=group)
        self.client = ActionClient(self, NavigateToPose,
                                   'navigate_to_pose_impl',
                                   callback_group=group)
        self.server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info(
            'goal_guard forwarding navigate_to_pose -> navigate_to_pose_impl '
            '(min_clearance=%.2f m)' % self.min_clearance)

    def on_map(self, msg):
        self.grid = msg

    def on_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def clearance(self, cx, cy):
        info = self.grid.info
        radius = int(math.ceil(self.min_clearance / info.resolution)) + 1
        best = 9.9
        for dy in range(-radius, radius + 1):
            row = cy + dy
            if not 0 <= row < info.height:
                continue
            base = row * info.width
            for dx in range(-radius, radius + 1):
                col = cx + dx
                if 0 <= col < info.width:
                    value = self.grid.data[base + col]
                    if value >= OCCUPIED_THRESHOLD:
                        d = math.hypot(dx, dy) * info.resolution
                        if d < best:
                            best = d
        return best

    def validate(self, x, y):
        if self.grid is None:
            return True, 'map not received yet, forwarding unchecked'
        info = self.grid.info
        res = info.resolution
        cx = int(math.floor((x - info.origin.position.x) / res))
        cy = int(math.floor((y - info.origin.position.y) / res))
        if not (0 <= cx < info.width and 0 <= cy < info.height):
            return False, 'goal (%.2f, %.2f) is outside the map bounds' % (x, y)
        value = self.grid.data[cy * info.width + cx]
        if value < 0:
            return False, 'goal (%.2f, %.2f) is in unknown space' % (x, y)
        if value >= OCCUPIED_THRESHOLD:
            return False, 'goal (%.2f, %.2f) is in an occupied cell' % (x, y)
        clear = self.clearance(cx, cy)
        if clear < self.min_clearance:
            return False, ('goal (%.2f, %.2f) has %.2f m clearance < %.2f m'
                           % (x, y, clear, self.min_clearance))
        return True, 'ok (clearance %.2f m)' % clear

    def feedback_cb(self, goal_handle):
        def cb(feedback_msg):
            if goal_handle.is_active:
                goal_handle.publish_feedback(feedback_msg.feedback)
        return cb

    def execute(self, goal_handle):
        result = NavigateToPose.Result()
        pose = goal_handle.request.pose.pose.position
        ok, reason = self.validate(pose.x, pose.y)
        if not ok:
            self.get_logger().warn('rejected goal: ' + reason)
            goal_handle.abort()
            return result

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'bt_navigator action navigate_to_pose_impl not available')
            goal_handle.abort()
            return result

        send_future = self.client.send_goal_async(
            goal_handle.request, feedback_callback=self.feedback_cb(goal_handle))
        deadline = time.time() + 10.0
        while not send_future.done() and time.time() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            time.sleep(0.02)
        if not send_future.done():
            self.get_logger().error('timed out sending goal to bt_navigator')
            goal_handle.abort()
            return result

        inner = send_future.result()
        if inner is None or not inner.accepted:
            goal_handle.abort()
            return result

        result_future = inner.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                inner.cancel_goal_async()
                goal_handle.canceled()
                return result
            time.sleep(0.02)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = GoalGuard()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
