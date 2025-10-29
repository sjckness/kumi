#--------------------------------------------------------------------------------#
#   
#   simple PID controller 
#   
#   reads the /target_positions and perform a PID on the efforts using /joint_states as feedback
#   
#   -> not tuned
#   
#   
#--------------------------------------------------------------------------------#

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PIDController(Node):
    ANSI_RED = "\033[91m"
    ANSI_GREEN = "\033[92m"
    ANSI_RESET = "\033[0m"

    def __init__(self):
        super().__init__('pid_effort_controller')

        # Allow external launch files to set / use simulation time.
        #self.declare_parameter('use_sim_time', False)

        # joint list
        self.joints = ['front_sh', 'front_ank', 'back_sh', 'back_ank']
        self.num_joints = len(self.joints)

        # Gains by joint
        self.kp = np.array([1.45, 1.45, 1.45, 1.45], dtype=float)
        self.ki = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)
        self.kd = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

        # Effort limits
        self.max_effort = 6.0   #(N/cm)
        self.max_delta = 1.0    #max delta effort (to be find a good value)
        tolerance_value = float(self.declare_parameter('position_tolerance', 0.05).value)   #tollerance arround the target positions
        self.position_tolerance = np.full(self.num_joints, tolerance_value, dtype=float)

        # Internal state variables.
        self.control_period = 0.01                                  #freq control -> 100 Hz
        self.target_positions = np.zeros(self.num_joints)
        self.last_positions = np.zeros(self.num_joints)
        self.integrals = np.zeros(self.num_joints)
        self.last_efforts = np.zeros(self.num_joints)
        self.previous_target_positions = np.zeros(self.num_joints)
        self.prev_errors = np.zeros(self.num_joints)
        self.joint_indices = None

        # Joint_states for feedback
        self.subscription_joint_state = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            100,
        )
        #target positions
        self.subscription_target = self.create_subscription(
            Float64MultiArray,
            '/target_positions',
            self.target_callback,
            100,
        )
        #efforts published 
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_effort_controller/commands',
            100,
        )
        #publish efforts and actual positions of the joints
        self.pid_data_publisher = self.create_publisher(
            Float64MultiArray,
            '/PID_data',
            100,
        )

        self.control_timer = self.create_timer(self.control_period, self.control_loop)  #control timer
        self.print_timer = self.create_timer(1.0, self.print_data)                      #console log timer

    # Callbacks
    def joint_state_callback(self, msg: JointState) -> None:
        if not msg.name:
            return

        if self.joint_indices is None:
            name_to_index = {name: idx for idx, name in enumerate(msg.name)}
            self.joint_indices = [name_to_index[joint] for joint in self.joints]

        positions = np.array(msg.position, dtype=float)         #update actual positions
        self.last_positions = positions[self.joint_indices]

    def target_callback(self, msg: Float64MultiArray) -> None:
        data = np.array(msg.data, dtype=float)
        self.target_positions = data                            #update target positions
    
    #---------------------
    # --- Control loop ---
    #---------------------
    def control_loop(self) -> None:
        if self.joint_indices is None:
            return

        #compute errors
        errors = self.target_positions - self.last_positions
        within_tolerance = np.abs(errors) <= self.position_tolerance

        # Reset integrator when target changes
        target_changed = self.target_positions != self.previous_target_positions
        self.integrals[target_changed] = 0.0

        #integral and derivatives
        self.integrals += errors * self.control_period
        self.integrals[within_tolerance] = 0.0
        derivatives = (errors - self.prev_errors) / self.control_period
        self.prev_errors = errors.copy()

        p_term = self.kp * errors
        i_term = self.ki * self.integrals
        d_term = self.kd * derivatives

        efforts = p_term + i_term + d_term
        efforts = np.clip(efforts, -self.max_effort, self.max_effort)       #remap efforts in the limits

        delta = efforts - self.last_efforts
        delta = np.clip(delta, -self.max_delta, self.max_delta)             #applys the max delta efforts
        efforts = self.last_efforts + delta
        self.last_efforts = efforts

        command_msg = Float64MultiArray()
        command_msg.data = efforts.tolist()
        self.command_publisher.publish(command_msg)                         #publish the efforts

        pid_msg = Float64MultiArray()
        pid_msg.data = np.concatenate(
            (p_term, i_term, d_term, self.target_positions, self.last_positions)
        ).tolist()
        self.pid_data_publisher.publish(pid_msg)                            #publish all pid information

        self.previous_target_positions = self.target_positions.copy()

    def print_data(self) -> None:                                           #console print of the PID data
        efforts_str = ", ".join(f"{value:.3f}" for value in self.last_efforts)
        targets_str = ", ".join(f"{value:.3f}" for value in self.target_positions)
        position_str = ", ".join(f"{value:.3f}" for value in self.last_positions)
        pos_error_str = ", ".join(self._format_effort(value) for value in (self.target_positions - self.last_positions))
        self.get_logger().info(f"Efforts: [{efforts_str}]")
        self.get_logger().info(f"Positions: [{pos_error_str}]")

    def _format_effort(self, value: float) -> str:                          #effort saturations printed in red
        formatted = f"{value:.3f}"
        if abs(value) >= 0.05:
            return f"{self.ANSI_RED}{formatted}{self.ANSI_RESET}"
        if abs(value) > 0:
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
