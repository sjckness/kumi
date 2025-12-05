#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial


class ServoSerialBridge(Node):
    def __init__(self): 
        super().__init__('servo_serial_bridge')

        # Parametri configurabili da launch / CLI
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.last_angle = None   # memorizza ultimo angolo mandato

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f"Tentativo apertura seriale su {port} @ {baudrate}...")

        self.ser = None
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.get_logger().info("Seriale aperta con successo.")

        # Subscriber all’angolo
        self.sub = self.create_subscription(
            Float32,
            '/servo_angle',          # topic da cui leggi l'angolo
            self.angle_callback,
            10
        )

    
    def angle_callback(self, msg: Float32):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn_throttle(5.0, "Seriale non aperta.")
            return

        angle = int(msg.data)

        # Clamp
        angle = max(0, min(angle, 180))

        # Se l'angolo è IDENTICO all'ultimo già inviato → non fare nulla
        if self.last_angle == angle:
            return

        # Aggiorna ultimo angolo
        self.last_angle = angle

        # Manda alla seriale solo i cambi veri
        try:
            data = f"{angle}"
            self.ser.write(data.encode('ascii'))
            self.get_logger().info(f"Inviato → {angle}")
        except Exception as e:
            self.get_logger().error(f"Errore seriale: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

