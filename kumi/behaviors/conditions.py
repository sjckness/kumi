import py_trees
from std_msgs.msg import String, Float32
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

    def _on_state_update(self, msg: Float32) -> None:
        if msg.data == "yes":
            self._last_yes_time = self.node.get_clock().now()

    def update(self) -> py_trees.common.Status:
        now = self.node.get_clock().now()

        if self._last_yes_time is None:
            self.node.get_logger().info("walk: not active")
            return py_trees.common.Status.SUCCESS

        if (now - self._last_yes_time) <= Duration(seconds=1):
            self.node.get_logger().info("walk: active")
            return py_trees.common.Status.FAILURE

        self.node.get_logger().info("walk: not active")
        return py_trees.common.Status.SUCCESS

class ObstacleRec(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="ObstacleRec"):
        super().__init__(name)
        self.node = node
        self._last_yes_time: Time | None = None
        self.distance = None  # latest obstacle distance
        self._last_close_time: Time | None = None

        # Listen for activation messages and cache the latest state for the BT tick
        self.subscription_joint_state = node.create_subscription(
            Float32,
            '/front_obstacle_distance',
            self._on_state_update,
            10,
        )

    def _on_state_update(self, msg: Float32) -> None:
        self.distance = msg.data 

    def update(self) -> py_trees.common.Status:
        now = self.node.get_clock().now()

        if self.distance is None:
            self.node.get_logger().info("obstacle distance: no data")
            return py_trees.common.Status.FAILURE

        if self.distance <= 0.8:
            self.node.get_logger().info("obstacle in <=0.8m :(")
            self._last_close_time = now
            return py_trees.common.Status.SUCCESS

        # If we recently saw an obstacle, wait 3s before reporting clear
        if self._last_close_time and (now - self._last_close_time) <= Duration(seconds=3):
            self.node.get_logger().info("obstacle just cleared, waiting 3s")
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info("no obstacles ahead :)")
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
