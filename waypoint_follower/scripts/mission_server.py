#!/usr/bin/env python3
import math
import os
import time
import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from waypoint_follower.action import Mission

LINEAR_KP = 0.5
ANGULAR_KP = 1.2
MAX_LINEAR = 0.22
MAX_ANGULAR = 1.5
POS_TOLERANCE = 0.1
CONTROL_PERIOD = 0.05


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionServer(Node):
    def __init__(self):
        super().__init__('mission_server')
        self.cb_group = ReentrantCallbackGroup()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10, callback_group=self.cb_group)

        self._action_server = ActionServer(
            self,
            Mission,
            'follow_mission',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group)

        self.get_logger().info('mission_server ready')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def drive_to(self, goal_handle, target_x, target_y, index, status_prefix, total_distance_acc):
        """Drive to (target_x, target_y). Returns (cancelled, distance_travelled)."""
        distance_travelled = 0.0
        last_x, last_y = self.x, self.y

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                return True, distance_travelled

            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.hypot(dx, dy)

            if dist < POS_TOLERANCE:
                self.stop_robot()
                break

            target_heading = math.atan2(dy, dx)
            heading_error = math.atan2(
                math.sin(target_heading - self.yaw),
                math.cos(target_heading - self.yaw))

            twist = Twist()
            twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, ANGULAR_KP * heading_error))

            if abs(heading_error) < 0.4:
                twist.linear.x = max(0.0, min(MAX_LINEAR, LINEAR_KP * dist))

            self.cmd_pub.publish(twist)

            feedback = Mission.Feedback()
            feedback.current_waypoint_index = index
            feedback.status = f'{status_prefix}, {dist:.2f}m remaining'
            feedback.distance_to_target = dist
            goal_handle.publish_feedback(feedback)

            distance_travelled += math.hypot(self.x - last_x, self.y - last_y)
            last_x, last_y = self.x, self.y

            time.sleep(CONTROL_PERIOD)

        distance_travelled += math.hypot(self.x - last_x, self.y - last_y)
        return False, distance_travelled

    def execute_cb(self, goal_handle):
        mission_file = goal_handle.request.mission_file
        mission_path = os.path.join(
            get_package_share_directory('waypoint_follower'), 'missions', mission_file)

        with open(mission_path, 'r') as f:
            mission = yaml.safe_load(f)

        base = mission['base']
        waypoints = mission['waypoints']
        return_to_base = mission.get('return_to_base', False)

        total_distance = 0.0
        waypoints_completed = 0

        for i, wp in enumerate(waypoints):
            status = f'en route to waypoint {i + 1}/{len(waypoints)}'
            cancelled, dist = self.drive_to(goal_handle, wp['x'], wp['y'], i, status, total_distance)
            total_distance += dist

            if cancelled:
                goal_handle.canceled()
                result = Mission.Result()
                result.success = False
                result.total_distance = total_distance
                result.waypoints_completed = waypoints_completed
                return result

            waypoints_completed += 1

        if return_to_base:
            cancelled, dist = self.drive_to(
                goal_handle, base['x'], base['y'], len(waypoints), 'returning to base', total_distance)
            total_distance += dist

            if cancelled:
                goal_handle.canceled()
                result = Mission.Result()
                result.success = False
                result.total_distance = total_distance
                result.waypoints_completed = waypoints_completed
                return result

        goal_handle.succeed()
        result = Mission.Result()
        result.success = True
        result.total_distance = total_distance
        result.waypoints_completed = waypoints_completed
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
