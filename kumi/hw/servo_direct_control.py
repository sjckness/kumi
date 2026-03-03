#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import time


class ServoNode(Node):

    def __init__(self):
        super().__init__('servo_node')

        # usa pigpio backend se disponibile (più stabile)
        factory = PiGPIOFactory()
        self.servo = Servo(18, pin_factory=factory)

        self.subscription = self.create_subscription(
            Float32,
            'servo_angle',
            self.angle_callback,
            10
        )

        self.get_logger().info("Servo node avviato")

    def angle_callback(self, msg):
        angle = max(0.0, min(180.0, msg.data))

        # gpiozero usa range -1 a +1
        normalized = (angle / 90.0) - 1.0
        self.servo.value = normalized

        self.get_logger().info(f"Angolo: {angle:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()