#!/usr/bin/env python3
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from waypoint_follower.action import Mission


class MissionClient(Node):
    def __init__(self, mission_file):
        super().__init__('mission_client')
        self.mission_file = mission_file
        self._client = ActionClient(self, Mission, 'follow_mission')
        self._goal_handle = None
        self._done = False

    def send_goal(self):
        self.get_logger().info('waiting for action server...')
        self._client.wait_for_server()

        goal_msg = Mission.Goal()
        goal_msg.mission_file = self.mission_file

        self.get_logger().info(f'sending mission: {self.mission_file}')
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            self._done = True
            return

        self._goal_handle = goal_handle
        self.get_logger().info('goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[wp {fb.current_waypoint_index}] {fb.status} '
            f'(dist_to_target={fb.distance_to_target:.2f}m)')

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(
            f'RESULT success={result.success} '
            f'total_distance={result.total_distance:.2f}m '
            f'waypoints_completed={result.waypoints_completed}')
        self._done = True

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('cancelling mission...')
            self._goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)

    mission_file = 'mission_square.yaml'
    for arg in sys.argv[1:]:
        if not arg.startswith('--') and not arg.startswith('-'):
            mission_file = arg

    node = MissionClient(mission_file)
    node.send_goal()

    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.cancel_goal()
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
