import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.duration import Duration

class test_move(Node):
    def __init__(self):
        super().__init__('test_move')

        self.sequence_file_path = "/home/andreas/dev_ws/src/kumi/resource/.csv"
        self.sender_period = 0.5
        self.publish_duration = Duration(seconds=2.0)

        self.sender = self.create_publisher(
            String,
            '/target_sequence',
            100,
        )

        self.sender_timer = self.create_timer(self.sender_period, self.sender_callback)
        self.start_time = self.get_clock().now()

    def sender_callback(self):
        elapsed_time = self.get_clock().now() - self.start_time
        if elapsed_time > self.publish_duration:
            self.get_logger().info("DONE")
            self.sender_timer.cancel()
            return

        msg = String()
        msg.data = self.sequence_file_path
        self.sender.publish(msg)

        self.get_logger().info("publishing...")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = test_move()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()