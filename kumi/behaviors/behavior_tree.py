import rclpy
from rclpy.node import Node
import py_trees
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from .actions import SendNextCSVPoint
from .conditions import IsActive, EmStop, ObstacleRec
from .helpers import load_csv_in_radians

class BTNode(Node):
    def __init__(self):
        super().__init__("bt_node")

        pkg_share = Path(get_package_share_directory('kumi'))
        csv_path = pkg_share / 'resource/demo_flip.csv'
        positions_list = load_csv_in_radians(csv_path)

        # step_seq: controlla che sia attivo prima di inviare tutti i punti
        step_seq = py_trees.composites.Selector("WalkStep", memory=True)
        step_seq.add_children([
            IsActive(self),
            ObstacleRec(self),
            SendNextCSVPoint(self, positions_list),
        ])

        # main: EmStop al vertice, seguito dalla sequenza di passi
        main = py_trees.composites.Sequence("Main", memory=False)
        main.add_children([
            EmStop(self),
            step_seq,
        ])

        self.tree = py_trees.trees.BehaviourTree(main)

        # Tick del BT a 10 Hz
        self.timer = self.create_timer(2, self.tree.tick)

def main(args=None):
    rclpy.init(args=args)
    node = BTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
