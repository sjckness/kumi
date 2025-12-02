import py_trees
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SendNextCSVPoint(py_trees.behaviour.Behaviour):
    def __init__(self, node, positions_list, name="SendNextCSVPoint"):
        super().__init__(name)
        self.node = node
        self.positions_list = positions_list
        self.index = 0
        self._completed = False

        self.pub = node.create_publisher(
            JointTrajectory,
            '/multi_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = ['front_sh', 'front_ank', 'rear_sh', 'rear_ank']

    def initialise(self):
        # Reset progress each time the behaviour is entered
        self.index = 0
        self._completed = False

    def update(self):
        if self._completed:
            return py_trees.common.Status.SUCCESS

        if not self.positions_list:
            self.node.get_logger().warn("[BT] Nessun punto da inviare")
            self._completed = True
            return py_trees.common.Status.SUCCESS

        positions = self.positions_list[self.index]

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=500_000_000)

        traj.points.append(point)
        self.pub.publish(traj)

        self.node.get_logger().info(f"[BT] Inviato punto {self.index}: {positions}")
        self.index += 1

        if self.index >= len(self.positions_list):
            self.node.get_logger().info("[BT] Sequenza completata")
            self._completed = True
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
