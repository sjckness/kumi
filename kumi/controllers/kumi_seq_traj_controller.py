#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import csv
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


class CSVJointTrajectory(Node):
    def __init__(self, csv_path: str | None = None):
        super().__init__('csv_joint_trajectory')

        # Publisher
        self.pub = self.create_publisher(
            JointTrajectory,
            '/multi_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Joint names
        self.joint_names = ['front_sh', 'front_ank', 'rear_sh', 'rear_ank']

        pkg_share = Path(get_package_share_directory('kumi'))
        default_csv = pkg_share / 'resource/demo_flip.csv'

        # consenti override via parametro ROS o argomento esplicito
        self.declare_parameter('csv_path', str(default_csv))
        param_csv = Path(self.get_parameter('csv_path').value)
        csv_path = Path(csv_path) if csv_path else param_csv

        if not csv_path.exists():
            raise FileNotFoundError(f"CSV non trovato: {csv_path}")

        self.positions_list = self.load_csv_in_radians(csv_path)

        self.get_logger().info(f"Caricate {len(self.positions_list)} pose dal CSV (in radianti).")
        self.get_logger().info("Invio automatico di un punto ogni 2.0 secondi (0.5 Hz).")

        self.index = 0

        # 🔔 TIMER: 0.5 Hz = periodo 2.0 secondi
        self.timer = self.create_timer(2.0, self.timer_callback)

    def load_csv_in_radians(self, path):
        positions = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                degrees = [float(v) for v in row]
                radians = [math.radians(v) for v in degrees]
                positions.append(radians)
        return positions

    def timer_callback(self):
        """Chiamata automaticamente ogni 2 secondi dal timer."""
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

        # ⏱ tempo target per il controller
        # Puoi lasciare 0.5s come prima, o metterlo a 2.0s
        point.time_from_start = Duration(sec=0, nanosec=500_000_000)

        traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info(f"[{self.index}] inviato punto: {positions}")

        self.index += 1


def main(args=None):
    rclpy.init(args=args)

    node = CSVJointTrajectory()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
