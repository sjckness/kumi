from setuptools import find_packages, setup
from glob import glob

package_name = 'kumi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (f'share/{package_name}/launch', glob('launch/*.py')),

        # URDF / xacro
        (f'share/{package_name}/description/', glob('description/*.xacro')),

        # meshes
        (f'share/{package_name}/description/mesh/', glob('description/mesh/*')),
        
        # Config files (YAML)
        (f'share/{package_name}/config', glob('config/*.yaml')),

        # Worlds (for Gazebo)
        (f'share/{package_name}/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PID_effort_controller = kumi.controllers.PID_effort_controller:main',
            'kumi_seq_traj_controller = kumi.controllers.kumi_seq_traj_controller:main',
            'kumi_seq_traj_controller_keyboard = kumi.controllers.kumi_seq_traj_controller_keyboard:main',
            'kumi_trajectory_controller = kumi.controllers.kumi_trajectory_controller:main',

            'kumi_signal_bringup = kumi.matlab_interface.kumi_signal_bringup:main',
            'kumi_single_move = kumi.matlab_interface.single_move:main',
            'reset = kumi.matlab_interface.reset:main',

            'com_calculator = kumi.phisics.com_calculator:main',
            'orientation = kumi.phisics.orientation:main',

            'console_input = kumi.testing.console_input:main',
            'test_move = kumi.testing.test_move:main',
            'hard_reset = kumi.testing.hard_reset:main',
            'publish_active = kumi.testing.publish_active:main',

            'real_imu = kumi.external.real_imu:main',
            
            'bt = kumi.behaviors.behavior_tree:main',

            'front_distance = kumi.stereocamera.front_distance:main',

            'servo_control_serial = kumi.hw.servo_serial_control:main',
            'servo_control = kumi.hw.servo_direct_control:main'

            
        ]
    },

)
