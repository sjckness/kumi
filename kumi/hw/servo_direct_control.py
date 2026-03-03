#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Servo


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.gpio = 18  # GPIO BCM
        # min_pulse_width / max_pulse_width tipici; puoi aggiustarli
        self.servo = Servo(self.gpio, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

        self.sub = self.create_subscription(Float32, 'servo_angle', self.cb, 10)
        self.get_logger().info("Servo node avviato su /servo_angle (Float32 gradi 0..180)")

    def cb(self, msg: Float32):
        angle = float(msg.data)
        angle = max(0.0, min(180.0, angle))

        # gpiozero Servo.value è [-1..+1]
        self.servo.value = (angle / 90.0) - 1.0
        self.get_logger().info(f"Angle: {angle:.1f} deg")

    def destroy_node(self):
        try:
            self.servo.detach()  # smette di generare PWM
        except Exception:
            pass
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