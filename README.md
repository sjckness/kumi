# KUMI (aka Bruno)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?) 
![ROS](https://img.shields.io/badge/ROS-2_Jazzy-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic_8.9-6C3AB2?logo=gazebo)

## Table of content
[Installation](#install)

[Build](#build)

[Launch](#launch)

[Controller](#controller)

[Usefull commands](#commands)

[Foxglove](#foxglove)

## Installation
[ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

[Gazebo Harmonic 8.9 installation guide](https://gazebosim.org/docs/harmonic/install_ubuntu/)

### Package installation
First of all you need to create your ROS2 workspace (usually named dev_ws).
Inside of the workspace a directory named src has to be created and here you'll have all you packages.

For our case you can clone the main branch of this repository in ~/dev_ws/src/ using:

 ```bash
git clone https://github.com/sjckness/kumi.git
```

## Build
inside /dev_ws use the following command to build the package (this step is mandatory every time you open a new terminal): 
```bash
source install/setup.bash
```
Then you have to source the workspace, and now ros knows where your files are. Use:
```bash
source install/setup.bash
```
## Launch
```bash 
ros2 launch kumi kumi_gz_sim.launch.py
```
## Controller
To activate the walk controller, after the simulation is started and the robot is spawned, open a new terminal in ~/dev_ws and use:
```bash 
ros2 run kumi kumi_seq_traj_controller
```
Is also possible to use a version that send a point every time you press 'SPACE':
```bash 
ros2 run kumi kumi_seq_traj_controller_keyboard
```

## Commands
to kill gazebo:
```bash
pkill -9 -f 'gz-sim|gz sim|gz'
```
## Foxglove
Foxglove is a visualization and debugging tool for robotics that allows you to inspect, analyze, and replay ROS data (topics, messages, and logs) in real time or from recorded bag files.

In a new terminal:
```bash
foxglove-studio
```
Foxglove session info:
 - address: ws://localhost
 - port: 8765 (defined in launch file)