#!/usr/bin/env python3
import os
import time
import xacro
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from pathlib import Path
from subprocess import Popen
from ament_index_python.packages import get_package_share_directory

import os
os.environ['ROS_DOMAIN_ID'] = '0'


class KumiBringup(Node):
    def __init__(self):
        super().__init__('kumi_bringup')

        self.subscription_bringup_signal = self.create_subscription(
            Float64,
            '/bringup_signal',
            self.bringup_callback,
            10,
        )
    
    def bringup_callback(self, msg: Float64):
        if msg.data == 1:
            # === Percorsi principali ===
            pkg_share = get_package_share_directory('kumi')
            xacro_file = os.path.join(pkg_share, 'description/xacro', 'kumi.xacro')

            # === Processa lo xacro ===
            self.get_logger().info(f'Processing xacro: {xacro_file}')
            doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
            robot_desc = doc.toprettyxml(indent='  ')

            # === Avvia robot_state_publisher ===
            self.get_logger().info('Launching robot_state_publisher...')
            self.rsp = Popen([
                'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                '--ros-args', '-p', f'robot_description:={robot_desc}'
            ])

            # === Avvia Gazebo spawn ===
            self.get_logger().info('Spawning Kumi in Gazebo...')
            self.gz_spawn = Popen([
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-string', robot_desc,
                '-x', '0.0', '-y', '0.0', '-z', '0.5',
                '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                '-name', 'kumi', '-allow_renaming', 'false'
            ])

            # === Bridge ROS-Gazebo ===
            self.get_logger().info('Launching ros_gz_bridge...')
            self.bridge = Popen([
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/frontCamera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/rearCamera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/frontCamera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/rearCamera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/world/stairs/control@ros_gz_interfaces/srv/ControlWorld'
            ])

            # === Foxglove Bridge ===
            self.get_logger().info('Launching foxglove_bridge...')
            self.foxglove = Popen([
                'ros2', 'run', 'foxglove_bridge', 'foxglove_bridge',
                '--ros-args',
                '-p', 'port:=8765',
                '-p', 'address:=0.0.0.0',
                '-p', 'use_compression:=false'
            ])

            # === Controller nodes ===
            self.get_logger().info('Loading controllers...')
            self.pid_effort = Popen(['ros2', 'run', 'kumi', 'PID_effort_controller'])
            #self.com_calc = Popen(['ros2', 'run', 'kumi', 'com_calculator'])

            # === carica controller ros2_control ===
            time.sleep(3.0)
            self.get_logger().info('Activating controllers...')
            Popen(['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'])
            Popen(['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_group_effort_controller'])

            self.get_logger().info('✅ Kumi bringup completo!')



def main(args=None):
    rclpy.init(args=args)
    node = KumiBringup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Chiusura richiesta...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




        