#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import csv
import math
import threading
import sys
import termios
import tty
import select
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory

DEFAULT_JOINT_NAMES = [
    'front_sh', 'front_ank_y', 'front_ank_z',
    'rear_sh', 'rear_ank_y', 'rear_ank_z'
]


class CSVJointTrajectory(Node):
    def __init__(self, csv_path: str | None = None):
        super().__init__('csv_joint_trajectory')

        # Publisher
        self.pub = self.create_publisher(
            JointTrajectory,
            '/multi_joint_trajectory_controller/joint_trajectory',
            10
        )

        pkg_share = Path(get_package_share_directory('kumi'))
        default_csv = pkg_share / 'resource/demo_flip.csv'
        controller_config = pkg_share / 'config/trajectory_control_config.yaml'

        # Joint names (auto-letti dal controller se possibile)
        self.joint_names = self._load_joint_names(controller_config)
        self.declare_parameter('joint_names', self.joint_names)
        self.joint_names = [str(j) for j in self.get_parameter('joint_names').value]

        # consenti override via parametro ROS o argomento esplicito
        self.declare_parameter('csv_path', str(default_csv))
        csv_param = Path(self.get_parameter('csv_path').value)
        resolved_csv = Path(csv_path) if csv_path else csv_param

        # CSV path (usa quello passato)
        if not resolved_csv.exists():
            raise FileNotFoundError(f"CSV non trovato: {resolved_csv}")

        self.positions_list = self.load_csv_in_radians(resolved_csv, len(self.joint_names))

        self.get_logger().info(f"Joint usati dal multi_joint_trajectory_controller: {self.joint_names}")
        self.get_logger().info(f"CSV: {resolved_csv}")
        self.get_logger().info(f"Caricate {len(self.positions_list)} pose dal CSV (in radianti).")
        self.get_logger().info("Premi SPAZIO per inviare il prossimo punto. Ctrl+C per uscire.")

        self.index = 0
        self.lock = threading.Lock()

        # Gestione terminale raw (salviamo e ripristiniamo)
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # meno invasivo di setraw

        # Flag stop per thread
        self._stop_event = threading.Event()

        # Thread per ascoltare la tastiera (non-bloccante)
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        # Quando ROS sta chiudendo, fermiamo thread e ripristiniamo terminale
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        # Segnala stop al thread e ripristina terminale
        self._stop_event.set()
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
        except Exception:
            pass

    def destroy_node(self):
        # Assicura ripristino anche se destroy_node viene chiamato esplicitamente
        self.on_shutdown()
        super().destroy_node()

    def load_csv_in_radians(self, path: Path, expected_len: int):
        positions = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row_idx, row in enumerate(reader, start=1):
                if not row:
                    continue
                if len(row) != expected_len:
                    raise ValueError(
                        f"Riga {row_idx} di {path} ha {len(row)} valori: "
                        f"attesi {expected_len} (joint: {self.joint_names})"
                    )
                degrees = [float(v) for v in row]
                radians = [math.radians(v) for v in degrees]
                positions.append(radians)
        return positions

    def getch_nonblocking(self, timeout_s: float = 0.1):
        """Ritorna 1 carattere se disponibile entro timeout, altrimenti None."""
        dr, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if dr:
            return sys.stdin.read(1)
        return None

    def keyboard_listener(self):
        # Loop finché ROS è ok e non ci chiedono di fermarci
        while rclpy.ok() and not self._stop_event.is_set():
            ch = self.getch_nonblocking(0.1)
            if ch == ' ':
                with self.lock:
                    self.send_next_point()

    def send_next_point(self):
        if not self.positions_list:
            self.get_logger().warn("CSV vuoto: nessun punto da inviare.")
            return

        # Se siamo alla fine → ricomincia
        if self.index >= len(self.positions_list):
            self.get_logger().info("Sequenza completata. Ripartenza da capo.")
            self.index = 0

        positions = self.positions_list[self.index]

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=500_000_000)  # 0.5s

        traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info(f"[{self.index}] inviato punto: {positions}")

        self.index += 1

    def _load_joint_names(self, config_path: Path) -> list[str]:
        if not config_path.exists():
            self.get_logger().warn(
                f"Config {config_path} non trovato. Uso lista predefinita {DEFAULT_JOINT_NAMES}"
            )
            return DEFAULT_JOINT_NAMES.copy()

        try:
            data = yaml.safe_load(config_path.read_text()) or {}
            joints = (
                data.get('multi_joint_trajectory_controller', {})
                .get('ros__parameters', {})
                .get('joints', [])
            )
            if not joints:
                self.get_logger().warn(
                    f"Nessuna lista 'joints' in {config_path}. Uso default {DEFAULT_JOINT_NAMES}"
                )
                return DEFAULT_JOINT_NAMES.copy()
            return [str(j) for j in joints]
        except Exception as exc:
            self.get_logger().warn(
                f"Errore leggendo {config_path}: {exc}. Uso default {DEFAULT_JOINT_NAMES}"
            )
            return DEFAULT_JOINT_NAMES.copy()


def main(args=None):
    rclpy.init(args=args)

    node = CSVJointTrajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C qui entra quasi sempre, ma anche se non entrasse:
        # on_shutdown/destroy_node ripristinano comunque
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
