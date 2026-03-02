# kumi (aka Bruno)
ros2 + gazebo model and control


## Issues / ToDo  


## Warnings
(+) no real time kernel

## Solved issues (not updated)
(+++) (1)controller_manager not working             15/07    
(+++) (2)joint_trajectory_controller not loaded     21/07  
(+++) (3)control the joint via python script        22/07  
(+++) (4)robot goes around without permission!      27/08  
(++)  (5)configure IMU plugin                       29/08 
(++)  (6)configured effort controller               06/09

## About
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?) 
![ROS](https://img.shields.io/badge/ROS-2_Jazzy-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic_8.9-6C3AB2?logo=gazebo)

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