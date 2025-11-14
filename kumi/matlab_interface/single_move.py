import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import math


class singleTestMove(Node):
    def __init__(self):
        super().__init__('single_test_move')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/target_positions', 10)

        self.subscription_test_move = self.create_subscription(
            Float64,
            '/test_signal',
            self.test_callback,
            10,
        ) 

    def publish_values(self, values):
        msg = Float64MultiArray()
        msg.data = values
        self.publisher_.publish(msg)
        self.get_logger().info(f'Pubblicato: {msg.data}')

    def test_callback(self, msg: Float64):
        values = [40,-60,-20,30]
        values_rad = [math.radians(x) for x in values]
        if msg.data == 1:
            self.publish_values(values_rad)
        

def main(args=None):
    rclpy.init(args=args)
    node = singleTestMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
