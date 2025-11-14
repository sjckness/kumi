#!/usr/bin/env python3

#--------------------------------------------------------------------------------#
#   
#   reads /target_positions and controls the trajectory
#   
#   published on /multi_joint_trajectory_controller/joint_trajectory
#   
#--------------------------------------------------------------------------------#

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray


class SimpleJointTrajectory(Node):
    def __init__(self):
        super().__init__('simple_joint_trajectory')

        # Publisher verso il controller dei giunti
        self.pub = self.create_publisher(
            JointTrajectory,
            '/multi_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber per i target
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/target_positions',
            self.target_callback,
            10
        )

        # Nomi dei giunti (devono corrispondere al controller)
        self.joint_names = [
            'front_sh',  'front_ank' ,'back_sh', 'back_ank'
        ]

        self.get_logger().info("Nodo pronto: ascolto su /target_positions")

    def target_callback(self, msg: Float64MultiArray):

        if len(msg.data) != len(self.joint_names):
            self.get_logger().error(
                f"Ricevuti {len(msg.data)} valori, ma servono {len(self.joint_names)}"
            )
            return

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(msg.data)
        point.time_from_start = Duration(nanosec=500000000)#tempo per raggiungere la posizione

        traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info(f"Inviata traiettoria: {point.positions}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
