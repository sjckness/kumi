import py_trees
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SendNextCSVPoint(py_trees.behaviour.Behaviour):
    def __init__(self, node, positions_list, name="SendNextCSVPoint"):
        super().__init__(name)
        self.node = node
        self.positions_list = positions_list
        self.index = 0

        self.pub = node.create_publisher(
            JointTrajectory,
            '/multi_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = ['front_sh', 'front_ank', 'rear_sh', 'rear_ank']

    def update(self):
        if self.index >= len(self.positions_list):
            self.node.get_logger().info("Sequenza completata. Riparte da capo.")
            self.index = 0

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

        return py_trees.common.Status.SUCCESS
