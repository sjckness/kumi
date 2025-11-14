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

    return LaunchDescription([
        sim_time_arg,
        gazebo_resource_path,
        world_arg,
        gazebo_cmd,
    ]
    )



        