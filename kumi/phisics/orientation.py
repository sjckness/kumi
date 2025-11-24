import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from math import sqrt

class MahonyAHRS:
    def __init__(self, kp=1.0, ki=0.0):
        self.kp = kp
        self.ki = ki
        self.integral = np.zeros(3)
        self.q = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion wxyz

    def update(self, gx, gy, gz, ax, ay, az, dt):
        q1, q2, q3, q4 = self.q

        # Normalizza accelerometro
        norm = sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            return self.q
        ax /= norm
        ay /= norm
        az /= norm

        # Gravità stimata dal quaternion
        vx = 2*(q2*q4 - q1*q3)
        vy = 2*(q1*q2 + q3*q4)
        vz = q1*q1 - q2*q2 - q3*q3 + q4*q4

        # Errore tra acc e gravità
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        # Integrale
        if self.ki > 0:
            self.integral += np.array([ex, ey, ez]) * dt
        else:
            self.integral = np.zeros(3)

        # Correzione
        gx += self.kp * ex + self.ki * self.integral[0]
        gy += self.kp * ey + self.ki * self.integral[1]
        gz += self.kp * ez + self.ki * self.integral[2]

        # Integrazione gyro → quaternion
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt

        dq = np.array([
            -q2 * gx - q3 * gy - q4 * gz,
             q1 * gx + q3 * gz - q4 * gy,
             q1 * gy - q2 * gz + q4 * gx,
             q1 * gz + q2 * gy - q3 * gx
        ])

        self.q += dq
        self.q /= np.linalg.norm(self.q)

        return self.q


class OrientationNode(Node):
    def __init__(self):
        super().__init__("orientation_mahony_node")

        self.sub = self.create_subscription(Imu, "/mpu/data", self.imu_callback, 20)
        self.pub = self.create_publisher(PoseStamped, "/imu_pose", 20)

        self.filter = MahonyAHRS(kp=1.0, ki=0.0)
        self.prev_time = None

    def imu_callback(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = now - self.prev_time
        self.prev_time = now
        if dt <= 0:
            return

        # Gyro
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Accelerometro
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Filtro Mahony
        q = self.filter.update(gx, gy, gz, ax, ay, az, dt)

        # Pubblica PoseStamped (posizione = zero)
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "map"

        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = OrientationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

