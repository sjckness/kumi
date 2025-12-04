import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio

GPIO_SERVO = 18 

def angle_to_pulsewidth(angle_deg: float,
                        min_angle: float = 0.0,
                        max_angle: float = 180.0,
                        min_pw: int = 500,
                        max_pw: int = 2500) -> int:
    #clamp
    angle = max(min_angle, min(max_angle, angle_deg))
    # mapping lineare angolo -> pulsewidth
    ratio = (angle - min_angle) / (max_angle - min_angle)
    return int(min_pw + (max_pw - min_pw) * ratio)

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Impossibile connettersi a pigpio')
            raise RuntimeError('pigpio non connesso')

        self.get_logger().info(f'Servo collegato su GPIO {GPIO_SERVO}')

        self.sub = self.create_subscription(
            Float32,
            '/servo_angle',
            self.angle_callback,
            10
        )

    def angle_callback(self, msg: Float32):
        angle = msg.data
        pulsewidth = angle_to_pulsewidth(angle)
        self.pi.set_servo_pulsewidth(GPIO_SERVO, pulsewidth)
        self.get_logger().info(f'Angolo richiesto: {angle:.1f}°, pulsewidth: {pulsewidth} us')

    def destroy_node(self):
        # ferma il servo e chiude pigpio
        self.pi.set_servo_pulsewidth(GPIO_SERVO, 0)
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

