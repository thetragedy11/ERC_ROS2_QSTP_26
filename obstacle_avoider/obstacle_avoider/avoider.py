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
        self.lin_vel = 0.07
        self.ang_vel = 0.75

        self.get_logger().info("Obstacle Avoider Node Started. Robot is OFF.")

    def toggle_callback(self, request, response):
        self.is_active = request.data
        response.success = True
        response.message = f"Robot brain switched to {'ON' if self.is_active else 'OFF'}"
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
            self.get_logger().info(f"Obstacle! Front: {front_dist:.2f}m. Flanking...")
            twist.linear.x = 0.05 
            if left_dist > right_dist:
                twist.angular.z = 0.8 
            else:
                twist.angular.z = -0.8 
                
        elif front_dist > 0.7:
            if self.lin_vel < 0.13:
                self.lin_vel += 0.002 
            if self.ang_vel > 0.20:
                self.ang_vel -= 0.008 
            
            twist.linear.x = self.lin_vel
            twist.angular.z = self.ang_vel

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()