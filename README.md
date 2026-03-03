# KUMI (aka Bruno)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?) 
![ROS](https://img.shields.io/badge/ROS-2_Jazzy-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic_8.9-6C3AB2?logo=gazebo)

## Table of content
[Vai a Installazione](#install)

## install
[ROS2 Jazzy installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
## launch
in /dev_ws  
```bash
colcon build --symlink-install --packages-select kumi
source install/setup.bash  
ros2 launch kumi_controller kumi_gz_sim.launch.py
```

## to kill gazebo
```bash
killall -9 gazebo gzserver gzclient
```
## to open foxglove
in an empty terminal
```bash
foxglove-studio
```
address: ws://localhost
port: 8765 (defined in launch file)

## Control
### keyboard controller
```bash
ros2 run kumi kumi_seq_traj_controller_keyboard
```
### effort controller
PIDs not tuned, not recomended
## Descrizione delle giunzioni e link

| Link         | Parent joint     | Tipo Joint | Note                    |
|--------------|------------------|------------|-------------------------|
| body         | -                | -          | -                       |
| front_leg    | body             | revolute   |                         |
| front_foot   | front_leg        | revolute   |                         |
| back_leg     | body             | revolute   |                         |
| back_foot    | back_leg_d       | revolute   |                         |