import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            Float32,
            '/random_number',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        received_value = msg.data
        multiplied_value = received_value * 2.0
        self.get_logger().info(f'Received: [{received_value:.2f}]. Multiplied value: [{multiplied_value:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()