import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_effort_controller')

        # Allow external launch files to set / use simulation time.
        self.declare_parameter('use_sim_time', False)

        # Ordered joint list; adjust to match the hardware controller.
        self.joints = ['front_sh', 'front_ank', 'back_sh', 'back_ank']
        self.num_joints = len(self.joints)

        # Gains entered directly in this file (Nm/rad).
        self.kp = np.array([1.45, 1.45, 1.45, 1.45], dtype=float)
        self.ki = np.array([2.0, 2.0, 2.0, 2.0], dtype=float)
        self.kd = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

        # Effort saturation limits.
        self.max_effort = 10.0
        self.max_delta = 1.0

        # Internal state variables.
        self.control_period = 0.01  # 100 Hz
        self.target_positions = np.zeros(self.num_joints)
        self.last_positions = np.zeros(self.num_joints)
        self.integrals = np.zeros(self.num_joints)
        self.last_efforts = np.zeros(self.num_joints)
        self.previous_target_positions = np.zeros(self.num_joints)
        self.prev_errors = np.zeros(self.num_joints)
        self.joint_indices = None

        # ROS interfaces.
        self.subscription_joint_state = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            100,
        )
        self.subscription_target = self.create_subscription(
            Float64MultiArray,
            '/target_positions',
            self.target_callback,
            100,
        )
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_effort_controller/commands',
            100,
        )
        self.pid_data_publisher = self.create_publisher(
            Float64MultiArray,
            '/PID_data',
            100,
        )

        self.control_timer = self.create_timer(self.control_period, self.control_loop)
        self.print_timer = self.create_timer(1.0, self.print_data)

    # --- Callbacks ---
    def joint_state_callback(self, msg: JointState) -> None:
        if not msg.name:
            return

        if self.joint_indices is None:
            name_to_index = {name: idx for idx, name in enumerate(msg.name)}
            missing = [joint for joint in self.joints if joint not in name_to_index]
            if missing:
                self.get_logger().warning(
                    f"JointState missing joints {missing}; waiting for complete data."
                )
                return
            self.joint_indices = [name_to_index[joint] for joint in self.joints]

        positions = np.array(msg.position, dtype=float)
        if positions.size <= max(self.joint_indices):
            self.get_logger().warning("JointState array shorter than expected; skipping update.")
            return

        self.last_positions = positions[self.joint_indices]

    def target_callback(self, msg: Float64MultiArray) -> None:
        data = np.array(msg.data, dtype=float)
        if data.size != self.num_joints:
            self.get_logger().warning(
                f"Expected {self.num_joints} target positions, received {data.size}."
            )
            return
        self.target_positions = data

    # --- Control loop ---
    def control_loop(self) -> None:
        if self.joint_indices is None:
            return

        # Reset integrator when target changes.
        target_changed = self.target_positions != self.previous_target_positions
        self.integrals[target_changed] = 0.0

        errors = self.target_positions - self.last_positions
        self.integrals += errors * self.control_period
        derivatives = (errors - self.prev_errors) / self.control_period
        self.prev_errors = errors.copy()

        p_term = self.kp * errors
        i_term = self.ki * self.integrals
        d_term = self.kd * derivatives

        efforts = p_term + i_term + d_term
        efforts = np.clip(efforts, -self.max_effort, self.max_effort)

        delta = efforts - self.last_efforts
        delta = np.clip(delta, -self.max_delta, self.max_delta)
        efforts = self.last_efforts + delta
        self.last_efforts = efforts

        command_msg = Float64MultiArray()
        command_msg.data = efforts.tolist()
        self.command_publisher.publish(command_msg)

        pid_msg = Float64MultiArray()
        pid_msg.data = np.concatenate(
            (p_term, i_term, d_term, self.target_positions, self.last_positions)
        ).tolist()
        self.pid_data_publisher.publish(pid_msg)

        self.previous_target_positions = self.target_positions.copy()

    def print_data(self) -> None:
        efforts_str = ", ".join(self._format_effort(value) for value in self.last_efforts)
        targets_str = ", ".join(f"{value:.3f}" for value in self.target_positions)
        self.get_logger().info(f"Efforts: [{efforts_str}]")
        self.get_logger().info(f"Target positions: [{targets_str}]")

    def _format_effort(self, value: float) -> str:
        formatted = f"{value:.3f}"
        if abs(value) >= self.max_effort:
            return f"{self.ANSI_RED}{formatted}{self.ANSI_RESET}"
        if abs(value) > 0.0:
            return f"{self.ANSI_GREEN}{formatted}{self.ANSI_RESET}"
        return formatted


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
