# kumi
ros2 + gazebo model and control


## Issues / ToDo  
(++)  set and define phisics limits
(++)  pid tuning
(++)  new model update  

 

## Warnings
(+) no real time kernel

## Solved issues
(+++) (1)controller_manager not working             15/07    
(+++) (2)joint_trajectory_controller not loaded     21/07  
(+++) (3)control the joint via python script        22/07  
(+++) (4)robot goes around without permission!      27/08  
(++)  (5)configure IMU plugin                       29/08 
(++)  (6)configured effort controller               06/09

## About
ubuntu version: 24.04.3 LTS  
ros2 version: ros2 jazzy  
gazebo version: harmonic 8.9

## launch
in /dev_ws  
-> colcon build --symlink-install 
-> source install/setup.bash  
-> ros2 launch kumi_controller kumi_gz_sim.launch.py

to kill gazebo
-> killall -9 gazebo gzserver gzclient

to open foxglove
-> foxglove-studio
address: ws://localhost
port: 8765 (defined in launch file)


## Descrizione delle giunzioni e link

| Link         | Parent joint     | Tipo Joint | Note                    |
|--------------|------------------|------------|-------------------------|
| body         | -                | -          | -                       |
| front_leg    | body             | revolute   |                         |
| front_foot   | front_leg        | revolute   |                         |
| back_leg     | body             | revolute   |                         |
| back_foot    | back_leg_d       | revolute   |                         |