import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 


def generate_launch_description():
    # Launch Arguments
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Flag to enable use_sim_time'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='stairs.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_share= os.path.join(
        get_package_share_directory('kumi'))

    pkg_share = get_package_share_directory('kumi')

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'worlds')
    )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='stairs',     #name of the world.sdf file in /worlds folder
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
                                 ' -r']
                    )
                ]
             )
    
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 stairs.sdf']
        }.items()
    )

    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_share,
            'worlds',
            LaunchConfiguration('world')
        ]),
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        TextSubstitution(text=' -r -v 4')],
        'on_exit_shutdown': 'true'}.items()
    )
    
    xacro_file = os.path.join(pkg_share,
                              'description/xacro',
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

    #ros2 - gz bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/stairs/control@ros_gz_interfaces/msg/WorldControl[gz.msgs.WorldControl'
        ],
        output='screen'
    )

    #spawn of the robot
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.5',     #spawn at .5 meters from the ground
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'kumi',
                   '-allow_renaming', 'false'],
    )

    return LaunchDescription([
        sim_time_arg,
        gazebo_resource_path,
        world_arg,
        #arguments,
        #gazebo,
        gazebo_cmd,
        gz_spawn_entity,
        #gz_sim,
        bridge,
        node_robot_state_publisher,
    ]
    )



        