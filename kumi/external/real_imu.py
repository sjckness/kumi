import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math

class MPUReader(Node):

    def __init__(self):
        super().__init__("mpu_reader")

        # Apri la seriale (modifica con la tua porta)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Publisher del messaggio IMU
        self.pub = self.create_publisher(Imu, '/mpu/data', 10)

        # Timer a 100 Hz
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8').strip()

        if not line:
            return

        try:
            # formato: ax ay az gx gy gz
            vals = line.split()
            ax, ay, az, gx, gy, gz = map(float, vals)

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
 
            # Accelerazione in m/s^2 (converti se stai usando g) z-> x, 
            msg.linear_acceleration.x = -az * 9.81
            msg.linear_acceleration.y = ay * 9.81
            msg.linear_acceleration.z = ax * 9.81

            # Gyro già in °/s → converto in rad/s
            msg.angular_velocity.x = math.radians(gz)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gx)

            # Nessuna orientazione (non stai usando filtro)
            msg.orientation.w = 1.0

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Errore parsing: {e}  line='{line}'")


def main(args=None):
    rclpy.init(args=args)
    node = MPUReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
