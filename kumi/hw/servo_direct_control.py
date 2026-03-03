#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio


class ServoNode(Node):

    def __init__(self):
        super().__init__('servo_node')

        self.GPIO = 18

        # Connessione a pigpio daemon
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "pigpiod non è attivo. Avvia: sudo systemctl start pigpiod"
            )

        # Subscriber
        self.subscription = self.create_subscription(
            Float32,
            'servo_angle',
            self.angle_callback,
            10
        )

        self.get_logger().info("Servo node avviato. In ascolto su /servo_angle")

    def angle_to_pulsewidth(self, angle: float) -> int:
        angle = max(0.0, min(180.0, angle))
        return int(500 + (angle / 180.0) * 2000)

    def angle_callback(self, msg: Float32):
        angle = msg.data
        pulsewidth = self.angle_to_pulsewidth(angle)
        self.pi.set_servo_pulsewidth(self.GPIO, pulsewidth)

        self.get_logger().info(f"Servo spostato a {angle:.1f}°")

    def destroy_node(self):
        # Stop servo signal
        self.pi.set_servo_pulsewidth(self.GPIO, 0)
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()