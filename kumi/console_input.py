#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class UserInputPublisher(Node):
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/target_positions', 10)

    def publish_values(self, values):
        msg = Float64MultiArray()
        msg.data = values
        self.publisher_.publish(msg)
        self.get_logger().info(f'Pubblicato: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = UserInputPublisher()

    try:
        while rclpy.ok():
            user_input = input("👉 Inserisci 4 angoli in gradi (separati da spazi): ")
            try:
                numbers_deg = [float(x) for x in user_input.strip().split()]
                if len(numbers_deg) == 4:
                    # Conversione gradi → radianti
                    numbers_rad = [math.radians(x) for x in numbers_deg]
                    node.publish_values(numbers_rad)
                else:
                    print("⚠️ Devi inserire esattamente 4 numeri.")
            except ValueError:
                print("⚠️ Input non valido. Riprova.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
