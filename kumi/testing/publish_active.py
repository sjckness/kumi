import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ActiveStatusPublisher(Node):
    def __init__(self):
        super().__init__('active_status_publisher')
        self.publisher_ = self.create_publisher(String, 'active_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

    def timer_callback(self):
        msg = String()
        msg.data = "yes"
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "yes"')

def main(args=None):
    rclpy.init(args=args)
    node = ActiveStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
