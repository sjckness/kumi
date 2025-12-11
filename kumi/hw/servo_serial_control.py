#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial


class ServoSerialBridge(Node):
    def __init__(self): 
        super().__init__('servo_serial_bridge')

        # Parametri configurabili da launch / CLI
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.last_angles = None   # memorizza ultimo set di angoli mandato

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f"Tentativo apertura seriale su {port} @ {baudrate}...")

        self.ser = None
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.get_logger().info("Seriale aperta con successo.")

        # Subscriber all’angolo
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/servo_angle',          # topic da cui leggi l'angolo
            self.angle_callback,
            10
        )

    
    def angle_callback(self, msg: Float32MultiArray):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn_throttle(5.0, "Seriale non aperta.")
            return

        # Converti e clampa ogni valore
        angles = [max(0, min(int(val), 180)) for val in msg.data]

        if not angles:
            self.get_logger().warn("Messaggio vuoto su /servo_angle.")
            return

        # Se l'angolo è IDENTICO all'ultimo già inviato → non fare nulla
        if self.last_angles == tuple(angles):
            return

        # Aggiorna ultimo angolo
        self.last_angles = tuple(angles)

        # Manda alla seriale solo i cambi veri
        
        data = " ".join(str(a) for a in angles)
        self.ser.write(data.encode('ascii'))
        self.get_logger().info(f"Inviato → {data}")


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
