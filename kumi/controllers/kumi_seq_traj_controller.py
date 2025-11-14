#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import csv
import os
import math
import threading
import sys
import termios
import tty


def getch():
    """Legge 1 carattere senza invio."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


class CSVJointTrajectory(Node):
    def __init__(self, csv_path):
        super().__init__('csv_joint_trajectory')

        # Publisher
        self.pub = self.create_publisher(
            JointTrajectory,
            '/multi_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Joint names
        self.joint_names = ['front_sh', 'front_ank', 'back_sh', 'back_ank']

        # CSV fisso
        csv_path = "/home/andreas/dev_ws/src/kumi/resource/demo_flip.csv"

        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"CSV non trovato: {csv_path}")

        self.positions_list = self.load_csv_in_radians(csv_path)

        self.get_logger().info(f"Caricate {len(self.positions_list)} pose dal CSV (in radianti).")
        self.get_logger().info("Premi SPAZIO per inviare il prossimo punto.")

        self.index = 0
        self.lock = threading.Lock()

        # Thread per ascoltare la tastiera
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def load_csv_in_radians(self, path):
        positions = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                degrees = [float(v) for v in row]
                radians = [math.radians(v) for v in degrees]
                positions.append(radians)
        return positions

    def keyboard_listener(self):
        """Thread che aspetta la pressione della barra spaziatrice."""
        while True:
            ch = getch()
            if ch == ' ':
                with self.lock:
                    self.send_next_point()

    def send_next_point(self):
        # Se siamo alla fine → ricomincia da capo
        if self.index >= len(self.positions_list):
            self.get_logger().info("Sequenza completata. Ripartenza da capo.")
            self.index = 0

        positions = self.positions_list[self.index]

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        # tempo target = 0.5s
        point.time_from_start = Duration(sec=0, nanosec=500_000_000)

        traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info(f"[{self.index}] inviato punto: {positions}")

        self.index += 1



def main(args=None):
    rclpy.init(args=args)

    csv_path = "/home/andreas/trajectory.csv"  # ignorato perché sovrascritto sopra

    node = CSVJointTrajectory(csv_path)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
