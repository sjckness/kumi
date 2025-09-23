import rclpy
import os
from launch_ros.substitutions import FindPackageShare
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import csv
import math
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

pkg_share = FindPackageShare("kumi").find("kumi")
traj_file = os.path.join(pkg_share, 'cntr_files', 'targets.csv')

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Parametri
        self.declare_parameter('joints', ['front_sh', 'front_knee', 'front_ank',
                                          'back_sh', 'back_knee','back_ank'])
        self.declare_parameter('kp', [1.45] * 6)
        self.declare_parameter('ki', [2.0] * 6)
        self.declare_parameter('kd', [0.0] * 6)
        self.declare_parameter('max_effort', 6.0)
        self.declare_parameter('max_delta', 1.0)
        self.declare_parameter('use_pid', True)

        self.joints = self.get_parameter('joints').get_parameter_value().string_array_value
        self.kp = np.array(self.get_parameter('kp').get_parameter_value().double_array_value)
        self.ki = np.array(self.get_parameter('ki').get_parameter_value().double_array_value)
        self.kd = np.array(self.get_parameter('kd').get_parameter_value().double_array_value)
        self.max_effort = self.get_parameter('max_effort').get_parameter_value().double_value
        self.max_delta = self.get_parameter('max_delta').get_parameter_value().double_value
        self.use_pid = self.get_parameter('use_pid').get_parameter_value().bool_value

        # Stato dei giunti
        self.target_positions = np.zeros(6)
        self.last_positions = np.zeros(6)
        self.last_velocities = np.zeros(6)
        self.integrals = np.zeros(6)
        self.last_efforts = np.zeros(6)
        self.previous_target_positions = np.copy(self.target_positions)

        # Subscriber giunti
        self.subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            100
        )

        # Subscriber PID e target positions
        self.sub_pid_gains = self.create_subscription(
            Float64MultiArray,
            '/pi_gains',
            self.pid_gains_callback,
            100
        )

        self.sub_target = self.create_subscription(
            Float64MultiArray,
            '/target_positions',
            self.target_callback,
            100
        )

        # Publisher effort
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_joint_controller/commands',
            100
        )

        # Publisher PID data
        self.publisher_PID_data = self.create_publisher(
            Float64MultiArray,
            '/PID_data',
            100
        )

        # Timer a 5ms (~200 Hz)
        self.timer = self.create_timer(0.01, self.control_loop)
        self.timer = self.create_timer(1.0, self.print_data)

    # --- Callback Subscriber ---
    def joint_state_callback(self, msg):
        joint_names = msg.name
        joint_positions = np.array(msg.position)
        joint_velocities = np.array(msg.velocity)
        joint_indices = [joint_names.index(j) for j in self.joints]
        self.last_positions = joint_positions[joint_indices]
        self.last_velocities = joint_velocities[joint_indices]

    def pid_gains_callback(self, msg):
        data = np.array(msg.data)
        if len(data) == 18:  # 6 joint * 3 parametri
            pid_matrix = data.reshape((6,3))
            self.kp[:] = pid_matrix[:,0]
            self.ki[:] = pid_matrix[:,1]
            self.kd[:] = pid_matrix[:,2]
            #self.get_logger().info(f"Updated PID params: P={self.kp}, I={self.ki}, D={self.kd}")

    def target_callback(self, msg):
        data = np.array(msg.data)
        if len(data) == 6:
            self.target_positions[:] = data
            #self.get_logger().info(f"Updated target positions: {self.target_positions}")

    # --- Controllo ---
    def control_loop(self):
        # Reset integrale se cambia target
        for j in range(6):
            if self.target_positions[j] != self.previous_target_positions[j]:
                self.integrals[j] = 0.0

        # Calcolo errori
        errors = self.target_positions - self.last_positions
        self.integrals += errors
        derivatives = self.last_velocities

        self.p = self.kp * errors
        self.i = self.ki * 0.01 * self.integrals
        self.d = self.kd * derivatives

        efforts = self.p + self.i + self.d
        efforts = np.clip(efforts, -self.max_effort, self.max_effort)

        delta_efforts = efforts - self.last_efforts
        delta_efforts = np.clip(delta_efforts, -self.max_delta, self.max_delta)
        efforts = self.last_efforts + delta_efforts
        self.last_efforts = efforts

        # --- Pubblica efforts ---
        msg_effort = Float64MultiArray()
        msg_effort.data = efforts.tolist()
        self.publisher.publish(msg_effort)

        # --- Pubblica PID data ---
        msg_pid = Float64MultiArray()
        msg_pid.data = np.concatenate([
            self.kp,
            self.ki,
            self.kd,
            self.p,
            self.i,
            self.d,
            self.target_positions,
            self.last_positions
        ]).tolist()
        self.publisher_PID_data.publish(msg_pid)

        # Aggiorna previous_target_positions
        self.previous_target_positions[:] = self.target_positions

        

    def print_data(self):
        self.get_logger().info(f"Efforts: {self.last_efforts}")
        self.get_logger().info(f"Target positions: {self.target_positions}")


def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
