import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction,
)
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_name = 'stairs'

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_start_delay = DeclareLaunchArgument(
        'gz_start_delay',
        default_value='5.0',
        description='Seconds to let Gazebo start before launching bridges and nodes'
    )
    world_load_delay = DeclareLaunchArgument(
        'world_load_delay',
        default_value='3.0',
        description='Extra seconds to wait for the world to settle before spawning the robot'
    )

    pkg_share = get_package_share_directory('kumi')

    # absolute path to the folder that contains the world sdf files
    gz_env = [
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=pkg_share, separator=':'),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(pkg_share, 'models'), separator=':'),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(pkg_share, 'worlds'), separator=':'),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(pkg_share, 'description'), separator=':'),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(pkg_share, 'description', 'mesh'), separator=':'),
    ]

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text=world_name),
        description='World name (without .sdf) located in kumi/worlds'
    )
    
    world_file = PathJoinSubstitution([
        FindPackageShare('kumi'),
        'worlds',
        LaunchConfiguration('world')
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            # passiamo il PATH ASSOLUTO + estensione
            'gz_args': [world_file, TextSubstitution(text='.sdf'), TextSubstitution(text=' -v 5 -r')]
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
        arguments=['-world', LaunchConfiguration('world'),
                   '-string', robot_desc,
                   '-x', '-0.15',
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

    # Timers to give Gazebo time to open and the world to load before spawning the robot.
    delayed_spawn_and_controllers = TimerAction(
        period=LaunchConfiguration('world_load_delay'),
        actions=[
            gz_spawn_entity,
            load_joint_state_broadcaster,
            load_joint_trajectory_controller,
            # Uncomment if/when you need them after spawn
            #load_joint_effort_controller,
            #trajectory_controller,
            #pid_effort_controller,
        ],
    )

    delayed_nodes = TimerAction(
        period=LaunchConfiguration('gz_start_delay'),
        actions=[
            bridge,
            node_robot_state_publisher,
            #foxglove_bridge,
            #depth_to_scan,
            #front_distance,
            #com_calculator,
            delayed_spawn_and_controllers,
        ],
    )

    return LaunchDescription([
        *gz_env,
        world_arg,
        gz_start_delay,
        world_load_delay,
        gz_sim,
        delayed_nodes,
    ])



        
