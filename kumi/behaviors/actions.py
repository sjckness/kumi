import py_trees
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class SendNextCSVPoint(py_trees.behaviour.Behaviour):
    def __init__(self, node, positions_list, name="SendNextCSVPoint"):
        super().__init__(name)
        self.node = node
        self.positions_list = positions_list
        self.index = 0
        self._completed = False
        self._waiting_for_target = False
        self._last_joint_positions = {}
        self._current_target = {}
        self.position_tolerance = 0.01  # rad

        self.pub = node.create_publisher(
            JointTrajectory,
            '/multi_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        self.joint_names = ['front_sh', 'front_ank', 'rear_sh', 'rear_ank']

    def initialise(self):
        # Reset progress each time the behaviour is entered
        self.index = 0
        self._completed = False
        self._waiting_for_target = False
        self._current_target = {}

    def _joint_state_callback(self, msg: JointState):
        # Keep the latest joint positions in a dict for quick lookup
        self._last_joint_positions = dict(zip(msg.name, msg.position))

    def _has_reached_target(self) -> bool:
        if not self._current_target:
            return True

        for joint, target_pos in self._current_target.items():
            current_pos = self._last_joint_positions.get(joint)
            if current_pos is None:
                return False
            if abs(current_pos - target_pos) > self.position_tolerance:
                return False
        return True

    def _publish_point(self, positions):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=300_000_000)

        traj.points.append(point)
        self.pub.publish(traj)

        self._current_target = dict(zip(self.joint_names, positions))
        self._waiting_for_target = True
        self.node.get_logger().info(f"[BT] Inviato punto {self.index}: {positions}")

    def update(self):
        if self._completed:
            return py_trees.common.Status.SUCCESS

        if not self.positions_list:
            self.node.get_logger().warn("[BT] Nessun punto da inviare")
            self._completed = True
            return py_trees.common.Status.SUCCESS

        if self._waiting_for_target:
            if not self._has_reached_target():
                return py_trees.common.Status.RUNNING

            self.node.get_logger().info(f"[BT] Punto {self.index} raggiunto")
            self._waiting_for_target = False
            self.index += 1

            if self.index >= len(self.positions_list):
                self.node.get_logger().info("[BT] Sequenza completata")
                self._completed = True
                return py_trees.common.Status.SUCCESS

        positions = self.positions_list[self.index]
        self._publish_point(positions)

        return py_trees.common.Status.RUNNING
