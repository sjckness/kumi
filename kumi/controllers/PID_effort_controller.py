import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Float32MultiArray
import os
import csv
import math
from rclpy.parameter import Parameter

class PIDController(Node):
    ANSI_RED = "\033[91m"
    ANSI_GREEN = "\033[92m"
    ANSI_RESET = "\033[0m"

    def __init__(self):
        super().__init__('pid_effort_controller')

        # joint list
        self.joints = ['front_sh', 'front_ank', 'rear_sh', 'rear_ank']
        self.num_joints = len(self.joints)

        # --- PARAMETRI ---
        self.declare_parameter('csv_rate', 2)   # tempo (s) tra un target e l'altro
        self.declare_parameter('loop_sequence', False)  # True → ricomincia da capo
        self.declare_parameter('Kp_fs', 1.45)
        self.declare_parameter('Kp_fa', 1.45)
        self.declare_parameter('Kp_bs', 1.45)
        self.declare_parameter('Kp_ba', 1.45)
        self.declare_parameter('Ki_fs', 0.0)
        self.declare_parameter('Ki_fa', 0.0)
        self.declare_parameter('Ki_bs', 0.0)
        self.declare_parameter('Ki_ba', 0.0)
        self.declare_parameter('Kd_fs', 0.0)
        self.declare_parameter('Kd_fa', 0.0)
        self.declare_parameter('Kd_bs', 0.0)
        self.declare_parameter('Kd_ba', 0.0)

        self.Kp_fs_val = self.get_parameter('Kp_fs').value
        self.Kp_fa_val = self.get_parameter('Kp_fa').value
        self.Kp_bs_val = self.get_parameter('Kp_bs').value
        self.Kp_ba_val = self.get_parameter('Kp_ba').value
        self.Ki_fs_val = self.get_parameter('Ki_fs').value
        self.Ki_fa_val = self.get_parameter('Ki_fa').value
        self.Ki_bs_val = self.get_parameter('Ki_bs').value
        self.Ki_ba_val = self.get_parameter('Ki_ba').value
        self.Kd_fs_val = self.get_parameter('Kd_fs').value
        self.Kd_fa_val = self.get_parameter('Kd_fa').value
        self.Kd_bs_val = self.get_parameter('Kd_bs').value
        self.Kd_ba_val = self.get_parameter('Kd_ba').value
       
        # Gains by joint
        self.kp = np.array([self.Kp_fs_val, self.Kp_fa_val, self.Kp_bs_val, self.Kp_ba_val],       dtype=float)
        self.ki = np.array([self.Ki_fs_val, self.Ki_fa_val, self.Ki_bs_val, self.Ki_ba_val],        dtype=float)
        self.kd = np.array([self.Kd_fs_val, self.Kd_fa_val, self.Kd_bs_val, self.Kd_ba_val],        dtype=float)

        # Effort limits
        self.max_effort = 6.0   #(N/cm)
        self.max_delta = 100.0    #max delta effort (to be find a good value)
        tolerance_value = float(self.declare_parameter('position_tolerance', 0.01).value)   #tollerance arround the target positions
        self.position_tolerance = np.full(self.num_joints, tolerance_value, dtype=float)

        # Internal state variables.
        self.control_period = 0.01                                  #freq control -> 1 kHz
        self.target_positions = np.zeros(self.num_joints)
        self.last_positions = np.zeros(self.num_joints)
        self.integrals = np.zeros(self.num_joints)
        self.last_efforts = np.zeros(self.num_joints)
        self.previous_target_positions = np.zeros(self.num_joints)
        self.prev_errors = np.zeros(self.num_joints)
        self.joint_indices = None

        # Sequenza CSV
        self.sequence = []
        self.current_step = 0
        self.sequence_timer = None
        self.sequence_active = False

        # Joint_states for feedback
        self.subscription_joint_state = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            100,
        )
        # target positions command: ros2 topic pub /target_sequence std_msgs/msg/String "data: '$(ros2 pkg prefix kumi)/share/kumi/resource/traj.csv'"
        self.subscription_target = self.create_subscription(
            Float64MultiArray,
            '/target_positions',
            self.target_callback,
            100,
        )
        #get target sequence
        self.subscription_sequence = self.create_subscription(
            String,
            '/target_sequence',
            self.sequence_callback,
            10,
        )
        
        #efforts published 
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_effort_controller/commands',
            100,
        )
        #publish efforts and actual positions of the joints
        self.pid_data_publisher = self.create_publisher(
            Float32MultiArray,
            '/PID_data',
            100,
        )

        self.control_timer = self.create_timer(self.control_period, self.control_loop)  #control timer
        self.print_timer = self.create_timer(1.0, self.print_data)                      #console log timer

    # Callbacks

    def param_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == 'Kp':
                self.Kp = param.value
            elif param.name == 'Ki':
                self.Ki = param.value
            elif param.name == 'Kd':
                self.Kd = param.value

        self.get_logger().info(
            f'PID updated: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}'
        )
    
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
    
    def sequence_callback(self, msg: str) -> None:
        """Riceve il percorso di un CSV con la sequenza dei target."""
        path = msg.data.strip()
        if not path:
            self.get_logger().warn("Ricevuto nome file CSV vuoto.")
            return

        self.sequence = self.load_csv_sequence(path)
        if not self.sequence:
            self.get_logger().error(f"Sequenza vuota o file non valido: {path}")
            return

        # (Ri)avvia la sequenza
        self.get_logger().info(f"Caricata sequenza di {len(self.sequence)} target da {path}")
        self.current_step = 0
        self.sequence_active = True
        if self.sequence_timer is not None:
            self.sequence_timer.cancel()

        csv_rate = float(self.get_parameter('csv_rate').value)
        self.sequence_timer = self.create_timer(csv_rate, self.next_target_from_sequence)

    def load_csv_sequence(self, filepath):
        if not os.path.exists(filepath):
            self.get_logger().error(f"File CSV non trovato: {filepath}")
            return []
        sequence = []
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                # Conversione gradi → radianti
                values = np.array([float(x) * math.pi / 180.0 for x in row], dtype=float)
                sequence.append(values)
        return sequence

    def next_target_from_sequence(self):
        if not self.sequence:
            return
        self.target_positions = self.sequence[self.current_step]
        self.current_step += 1

        # fine sequenza → o loop o stop
        if self.current_step >= len(self.sequence):
            if self.get_parameter('loop_sequence').value:
                self.current_step = 0
            else:
                self.get_logger().info("Sequenza completata.")
                self.sequence_timer.cancel()
                self.sequence_active = False

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
        #self.integrals[within_tolerance] = 0.0
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

        pid_msg = Float32MultiArray()
        pid_msg.data = np.concatenate(
            (self.target_positions, self.last_positions)
        ).tolist()
        self.pid_data_publisher.publish(pid_msg)                            #publish all pid information

        self.previous_target_positions = self.target_positions.copy()

    def print_data(self) -> None:                                           #console print of the PID data
        efforts_str = ", ".join(f"{value:.3f}" for value in self.last_efforts)
        targets_str = ", ".join(f"{value:.3f}" for value in self.target_positions)
        position_str = ", ".join(f"{value:.3f}" for value in self.last_positions)
        pos_error_str = ", ".join(self._format_effort(value) for value in (self.target_positions - self.last_positions))
        self.get_logger().info(f"Efforts: [{efforts_str}]")
        self.get_logger().info(f"Error: [{pos_error_str}]")

    def _format_effort(self, value: float) -> str:                          #effort saturations printed in red
        formatted = f"{value:.3f}"
        if abs(value) >= 0.1:
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
