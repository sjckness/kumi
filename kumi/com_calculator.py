#!/usr/bin/env python3

#------------------------------------------------------------------------#
#   COM and ZMP computation 
#   com published on /com topic
#   ZMP published on /zmp topic
#   both printed on the console
#------------------------------------------------------------------------#

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import numpy as np

# ANSI color codes per stampa colorata
GREEN = "\033[92m"
RED = "\033[91m"
ENDC = "\033[0m"

class CoMZMPNode(Node):
    def __init__(self):
        super().__init__('com_zmp_node')

        # Publisher
        self.com_pub = self.create_publisher(PointStamped, '/com', 10)
        self.zmp_pub = self.create_publisher(PointStamped, '/zmp', 10)

        # TF buffer e listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Link masses (esempio, sostituire con valori reali URDF)
        self.link_masses = {
            'body': 0.5,
            'front_leg': 0.2,
            'front_leg_d': 0.2,
            'front_foot': 0.15,
            'back_leg': 0.2,
            'back_leg_d': 0.2,
            'back_foot': 0.15,
        }
        self.link_frames = list(self.link_masses.keys())

        # Piede 0.3m x 0.3m
        self.foot_size = 0.25  # metà lato del quadrato

    def timer_callback(self):
        com = self.compute_com()
        if com is None:
            self.get_logger().warn("CoM non calcolabile, skipping iteration")
            return
        zmp = self.compute_zmp(com)

        # Pubblica CoM
        com_msg = PointStamped()
        com_msg.header.stamp = self.get_clock().now().to_msg()
        com_msg.header.frame_id = 'body'
        com_msg.point.x, com_msg.point.y, com_msg.point.z = com
        self.com_pub.publish(com_msg)

        # Pubblica ZMP
        zmp_msg = PointStamped()
        zmp_msg.header.stamp = self.get_clock().now().to_msg()
        zmp_msg.header.frame_id = 'body'
        zmp_msg.point.x, zmp_msg.point.y, zmp_msg.point.z = zmp
        self.zmp_pub.publish(zmp_msg)

        # Trasformazione nel frame del piede
        com_rel = self.transform_point_to_frame(com, 'body', 'front_foot')
        zmp_rel = self.transform_point_to_frame(zmp, 'body', 'front_foot')

        if com_rel is None or zmp_rel is None:
            self.get_logger().warn("Impossibile trasformare CoM/ZMP nel frame del piede")
            return

        com_over_foot = self.is_point_in_foot(com_rel[0], com_rel[1])
        zmp_over_foot = self.is_point_in_foot(zmp_rel[0], zmp_rel[1])

        if com_over_foot and zmp_over_foot:
            color = GREEN
        else:
            color = RED

        self.get_logger().info(f"{color}CoM(body): {com[0]:.3f}, {com[1]:.3f}, {com[2]:.3f} | "
                               f"ZMP(body): {zmp[0]:.3f}, {zmp[1]:.3f}, {zmp[2]:.3f}{ENDC}")

    def compute_com(self):
        total_mass = sum(self.link_masses.values())
        com_sum = np.zeros(3)

        for link in self.link_frames:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'body', link, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
                pos = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])
                com_sum += pos * self.link_masses[link]
            except Exception:
                continue

        if total_mass == 0:
            return None
        return com_sum / total_mass

    def compute_zmp(self, com):
        g = 9.81
        accel = np.zeros(3)  # accelerazione nulla per ora
        zmp = com.copy()
        zmp[0] -= com[2]/g * accel[0]
        zmp[1] -= com[2]/g * accel[1]
        return zmp

    def transform_point_to_frame(self, point, source_frame, target_frame):
        """
        Trasforma un punto (numpy array 3D) da source_frame a target_frame usando TF,
        senza usare scipy.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )

            # Traslazione
            t = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])

            # Quaternione
            q = trans.transform.rotation
            quat = np.array([q.x, q.y, q.z, q.w])

            # Converti quaternione in matrice di rotazione
            R = self.quaternion_to_rotation_matrix(quat)

            # Applica trasformazione: R * p + t
            point_trans = R.dot(point) + t
            return point_trans
        except Exception as e:
            self.get_logger().warn(f"Errore TF {source_frame}->{target_frame}: {e}")
            return None

    def quaternion_to_rotation_matrix(self, q):
        """
        Converte un quaternione [x,y,z,w] in matrice di rotazione 3x3
        """
        x, y, z, w = q
        R = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),         2*(x*z + y*w)],
            [2*(x*y + z*w),           1 - 2*(x**2 + z**2),   2*(y*z - x*w)],
            [2*(x*z - y*w),           2*(y*z + x*w),         1 - 2*(x**2 + y**2)]
        ])
        return R

    def is_point_in_foot(self, x, y):
        # Il piede è centrato in (0,0) nel frame front_foot
        return -self.foot_size <= x <= self.foot_size and -self.foot_size <= y <= self.foot_size

def main(args=None):
    rclpy.init(args=args)
    node = CoMZMPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
