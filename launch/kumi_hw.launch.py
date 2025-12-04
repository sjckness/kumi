import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    #ros2 + gz - foxglove bridge for data plot and analisys
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'use_compression': False
        }]
    )

    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        output='screen',
        remappings=[
            ('depth', '/d435_rgbd/depth_image'),
            ('depth_camera_info', '/d435_rgbd/camera_info'),
            ('scan', '/camera/scan')
        ],
        parameters=[{
            'output_frame_id': 'camera_link',   # o il tuo optical frame
            'range_min': 0.2,
            'range_max': 8.0,
            'scan_height': 10,                  # quante righe verticali usare
        }]
    )

    front_distance = Node(
        package='kumi',                      # il tuo package
        executable='front_distance',
        output='screen',
        parameters=[{
            'front_angle_width_deg': 10.0
        }]
    )

    bt = Node(
        package='kumi',                      # il tuo package
        executable='bt',
        output='screen',
    )

    servo_controller = Node(
        package='kumi',                      # il tuo package
        executable='servo_control',
        output='screen',
    )

    return LaunchDescription([
        foxglove_bridge,
        depth_to_scan,
        front_distance,
        bt,
        servo_controller
    ]
    )



        
