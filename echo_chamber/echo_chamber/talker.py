import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(Float32, '/random_number', 10)
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = random.uniform(0.0, 100.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data:.2f}"')

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    
    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()