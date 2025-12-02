import py_trees
from std_msgs.msg import String
from rclpy.duration import Duration
from rclpy.time import Time

class IsActive(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="IsActive"):
        super().__init__(name)
        self.node = node
        self._last_yes_time: Time | None = None

        # Listen for activation messages and cache the latest state for the BT tick
        self.subscription_joint_state = node.create_subscription(
            String,
            '/active_status',
            self._on_state_update,
            10,
        )

    def _on_state_update(self, msg: String) -> None:
        if msg.data == "yes":
            self._last_yes_time = self.node.get_clock().now()

    def update(self) -> py_trees.common.Status:
        now = self.node.get_clock().now()

        if self._last_yes_time is None:
            self.node.get_logger().info("walk: not active")
            return py_trees.common.Status.FAILURE

        if (now - self._last_yes_time) <= Duration(seconds=1):
            self.node.get_logger().info("walk: active")
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info("walk: not active")
        return py_trees.common.Status.FAILURE
        
class EmStop(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="EmStop"):
        super().__init__(name)
        self.node = node
        self._last_yes_time: Time | None = None

        # Listen for activation messages and cache the latest state for the BT tick
        self.subscription_joint_state = node.create_subscription(
            String,
            '/Emergency_Stop',
            self._on_state_update,
            10,
        )

    def _on_state_update(self, msg: String) -> None:
        if msg.data == "yes":
            self._last_yes_time = self.node.get_clock().now()

    def update(self) -> py_trees.common.Status:
        now = self.node.get_clock().now()

        if self._last_yes_time is None:
            self.node.get_logger().info("NO EMERGENCY")
            return py_trees.common.Status.SUCCESS

        if (now - self._last_yes_time) >= Duration(seconds=10):
            self.node.get_logger().info("NO EMERGENCY")
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info("EMERGENCY")
        return py_trees.common.Status.FAILURE
