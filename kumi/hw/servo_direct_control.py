#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import pigpio


class ServoSet45Node(Node):
    def __init__(self):
        super().__init__('servo_set_45_node')

        # Parametri (così puoi cambiarli da launch/CLI)
        self.declare_parameter('gpio', 18)     # BCM
        self.declare_parameter('angle', 45.0)  # gradi
        self.declare_parameter('min_deg', 0.0)
        self.declare_parameter('max_deg', 180.0)
        self.declare_parameter('min_us', 500)   # tipico: 500..2500 (a volte 1000..2000)
        self.declare_parameter('max_us', 2500)

        gpio = int(self.get_parameter('gpio').value)
        angle = float(self.get_parameter('angle').value)
        min_deg = float(self.get_parameter('min_deg').value)
        max_deg = float(self.get_parameter('max_deg').value)
        min_us = int(self.get_parameter('min_us').value)
        max_us = int(self.get_parameter('max_us').value)

        # Connetti pigpio daemon
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod non è attivo. Avvia: sudo systemctl start pigpiod")

        # Clamp + map angle -> pulsewidth
        angle = max(min_deg, min(max_deg, angle))
        span_deg = (max_deg - min_deg) if (max_deg - min_deg) != 0 else 1.0
        t = (angle - min_deg) / span_deg
        us = int(min_us + t * (max_us - min_us))

        self.pi.set_servo_pulsewidth(gpio, us)
        self.get_logger().info(f"Servo su GPIO {gpio} impostato a {angle:.1f}° ({us} us).")

        # nodo “one-shot”: aspetta un attimo e poi si chiude
        self.timer = self.create_timer(1.0, self._shutdown)

    def _shutdown(self):
        # se vuoi lasciare il servo “tenuto”, NON mettere pulsewidth a 0
        self.pi.stop()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ServoSet45Node()
    rclpy.spin(node)


if __name__ == '__main__':
    main()