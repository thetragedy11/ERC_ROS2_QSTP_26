import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('avoider_node')
        self.is_active = False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.srv = self.create_service(SetBool, '/toggle_robot', self.toggle_callback)
        self.current_linear_x = 0.05
        self.constant_angular_z = 1.0

        self.get_logger().info("Obstacle Avoider Node Started. Robot is OFF.")

    def toggle_callback(self, request, response):
        self.is_active = request.data
        response.success = True
        state_str = "ON" if self.is_active else "OFF"
        response.message = f"Robot brain switched to {state_str}"
        self.get_logger().info(response.message)
        return response

    def scan_callback(self, msg):
        twist = Twist()
        if not self.is_active:
            self.cmd_pub.publish(twist)
            return
        front_dist = msg.ranges[0] if not math.isinf(msg.ranges[0]) else msg.range_max
        left_dist = msg.ranges[90] if not math.isinf(msg.ranges[90]) else msg.range_max
        right_dist = msg.ranges[270] if not math.isinf(msg.ranges[270]) else msg.range_max
        if front_dist < 1.0:
            self.get_logger().info(f"Obstacle close! Front: {front_dist:.2f}m. Flanking...")
            self.current_linear_x = 0.05 
            twist.linear.x = 0.05 
            if left_dist > right_dist:
                twist.angular.z = 0.5  
            else:
                twist.angular.z = -0.5 
        elif front_dist > 0.7:
            twist.linear.x = self.current_linear_x
            twist.angular.z = self.constant_angular_z
            if self.current_linear_x < 0.5:  
                self.current_linear_x += 0.0005
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

