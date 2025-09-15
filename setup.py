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
        (f'share/{package_name}/description', glob('description/*.xacro')),
        
        # Config files (YAML)
        (f'share/{package_name}/config', glob('config/*.yaml')),

        # Worlds (for Gazebo)
        (f'share/{package_name}/worlds', glob('worlds/*')),
        
        # Worlds (for Gazebo)
        (f'share/{package_name}/cntr_files', glob('cntr_files/*')),
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
        'controller = kumi.kumi_controller:main',
        'PID_effort_controller = kumi.PID_effort_controller:main',
        'com_calculator = kumi.com_calculator:main',
        'PID_tuner_GUI = kumi.PID_tuner_GUI:main',
        'visualizer = kumi.visualizer_node:main',
    ]
},

)
