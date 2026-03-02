import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_name = 'empty'
    screenOn = ' '
    screenOff = ' -s '


    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    pkg_share= os.path.join(
        get_package_share_directory('kumi'))

    pkg_share = get_package_share_directory('kumi')

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'worlds')
    )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value=world_name,     #name of the world.sdf file in /worlds folder
                          description='Gz sim World'),
           ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 5',
                                 screenOff,
                                 '-r']
                    )
                ]
             )
    
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression([
                "'",
                LaunchConfiguration('world'),
                ".sdf -v 4 -r'"
            ])
        }.items()
    )
    
    xacro_file = os.path.join(pkg_share,
                              'description',
                              'kumi.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

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

    #spawn of the robot
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '-1.0',
                   '-y', '0.0',
                   '-z', '0.5',     #spawn at .5 meters from the ground
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '3.14159',
                   '-name', 'kumi',
                   '-allow_renaming', 'false'],
    )

    #joint state broadcaster for feedback on joints positions (no sensors used)
    load_joint_state_broadcaster= ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    #multi effort controller
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'multi_joint_trajectory_controller'],
        output='screen'
    )

    #pid effort controller -> controlled by /target_position 
    trajectory_controller = Node(
        package='kumi',
        executable='kumi_seq_traj_controller',
        output='screen'
    )

    #multi effort controller
    load_joint_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_group_effort_controller'],
        output='screen'
    )

    #ros2 - gz bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/frontCamera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rearCamera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/frontCamera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rearCamera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/bodyImu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/collisionFlag/frontFoot@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/collisionFlag/rearFoot@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/d435_rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435_rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435_rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            PythonExpression([
                "'/world/' + '",
                LaunchConfiguration('world'),
                "' + '/control@ros_gz_interfaces/srv/ControlWorld'"
            ])
        ],
        output='screen'
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

    #simple center of mass computation
    com_calculator = Node(
        package='kumi',
        executable='com_calculator',
        output='screen'

    )

    #pid effort controller -> controlled by /target_position 
    pid_effort_controller = Node(
        package='kumi',
        executable='PID_effort_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo_resource_path,
        arguments,
        #gazebo,
        gz_sim,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        #foxglove_bridge,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        #depth_to_scan,
        #front_distance,
        #load_joint_effort_controller,
        #trajectory_controller,
        #pid_effort_controller
        #com_calculator,
    ]
    )



        
