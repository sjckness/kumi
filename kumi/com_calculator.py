import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import pinocchio as pin
import xacro
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class ComCalculator(Node):
    def __init__(self):
        super().__init__('com_calculator')

        # Ottieni il percorso dell'XACRO dal pacchetto ROS 2
        pkg_share = get_package_share_directory('kumi')
        xacro_file = os.path.join(pkg_share, 'description', 'kumi.xacro')

        if not os.path.exists(xacro_file):
            self.get_logger().error(f"File XACRO non trovato: {xacro_file}")
            raise FileNotFoundError(f"{xacro_file} non esiste")

        # Genera URDF temporaneo
        urdf_path = '/tmp/kumi.urdf'
        doc = xacro.process_file(xacro_file)
        with open(urdf_path, 'w') as f:
            f.write(doc.toxml())

        # Carica modello Pinocchio
        self.robot_model = pin.buildModelFromUrdf(urdf_path)
        self.robot_data = self.robot_model.createData()

        self.latest_joint_msg = None

        # Subscriber a /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer a 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_msg = msg

    def timer_callback(self):
        if self.latest_joint_msg is None:
            return

        # Mapping giunti: nome -> indice in Pinocchio
        joint_index = {name: i for i, name in enumerate(self.robot_model.names[1:])}  # skip "universe"
        q = np.zeros(self.robot_model.nq)

        # Aggiorna configurazione dai joint_states
        for name, pos in zip(self.latest_joint_msg.name, self.latest_joint_msg.position):
            if name in joint_index:
                q[joint_index[name]] = pos

        # Calcola centro di massa
        com = pin.centerOfMass(self.robot_model, self.robot_data, q)
        self.get_logger().info(f"Centro di massa: {com}")

def main(args=None):
    rclpy.init(args=args)
    node = ComCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
