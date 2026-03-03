#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from std_msgs.msg import String
from subprocess import Popen, check_output
import time
from pathlib import Path
from std_msgs.msg import Float64MultiArray, String, Float64
from ament_index_python.packages import get_package_share_directory


class ResetAndSpawnXacroCLI(Node):
    def __init__(self):
        super().__init__('reset_cli')

        self.subscription_reset_signal = self.create_subscription(
            Float64,
            '/reset_signal',
            self.reset_callback,
            10,
        )

    def reset_callback(self, msg: Float64): 
        # Parametri
        self.world = 'empty'
        self.robot_name = 'kumi'
        pkg_share = Path(get_package_share_directory('kumi'))
        self.xacro_path = pkg_share / 'description/xacro/kumi.xacro'

        if not self.xacro_path.exists():
            self.get_logger().error(f'File Xacro non trovato: {self.xacro_path}')
            rclpy.shutdown()
            return
        
        if msg.data == 1:
            self.get_logger().info('Reset simulazione...')
            reset_client = self.create_client(ControlWorld, f'/world/{self.world}/control')
            while not reset_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('In attesa del servizio di reset...')
            req = ControlWorld.Request()
            req.world_control.reset.all = True
            future = reset_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('Reset completato.')


def main(args=None):
    rclpy.init(args=args)
    node = ResetAndSpawnXacroCLI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




        
