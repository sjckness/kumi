import rclpy
from rclpy.node import Node
import py_trees

from .actions import SendNextCSVPoint
from .conditions import IsActive
from .helpers import load_csv_in_radians

class BTNode(Node):
    def __init__(self):
        super().__init__("bt_node")

        csv_path = "/home/andreas/dev_ws/src/kumi/resource/demo_flip.csv"
        positions_list = load_csv_in_radians(csv_path)

        # Sequence che controlla active + ostacolo solo prima di inviare il punto
        step_seq = py_trees.composites.Sequence("WalkStep", memory=True)
        step_seq.add_children([
            IsActive(self),
            SendNextCSVPoint(self, positions_list),
        ])

        self.tree = py_trees.trees.BehaviourTree(step_seq)

        # Tick del BT a 0.5 Hz → un punto ogni 2 secondi
        self.timer = self.create_timer(2.0, self.tree.tick)

def main(args=None):
    rclpy.init(args=args)
    node = BTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
