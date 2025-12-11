import py_trees
from std_msgs.msg import Float32MultiArray


class SendNextCSVPoint(py_trees.behaviour.Behaviour):
    def __init__(self, node, positions_list, name="SendNextCSVPoint", send_period=2.0):
        """
        send_period: intervallo (in secondi) tra un punto e il successivo
        """
        super().__init__(name)
        self.node = node
        self.positions_list = positions_list
        self.index = 0
        self._completed = False
        self.send_period = send_period
        self._last_send_time = None

        self.pub = node.create_publisher(
            Float32MultiArray,
            '/servo_angle',
            10
        )

    def initialise(self):
        self.index = 0
        self._completed = False
        self._last_send_time = None

    def _publish_point(self, positions):
        msg = Float32MultiArray()
        msg.data = list(positions)
        self.pub.publish(msg)
        self._last_send_time = self.node.get_clock().now()
        self.node.get_logger().info(f"[BT] Inviato punto {self.index}: {positions}")

    def update(self):
        if self._completed:
            return py_trees.common.Status.SUCCESS

        if not self.positions_list:
            self.node.get_logger().warn("[BT] Nessun punto da inviare")
            self._completed = True
            return py_trees.common.Status.SUCCESS

        now = self.node.get_clock().now()

        # Pubblica un punto ogni send_period secondi
        if self._last_send_time is None or (now - self._last_send_time).nanoseconds / 1e9 >= self.send_period:
            positions = self.positions_list[self.index]
            self._publish_point(positions)
            self.index += 1

            if self.index >= len(self.positions_list):
                self.node.get_logger().info("[BT] Sequenza completata")
                self._completed = True
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
