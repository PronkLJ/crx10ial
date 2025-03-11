# Fanuc CRX10iA/L packages for ROS2 simulation
The goal of these packages is to plan and execute paths in MoveIt 2, then displaying these movements in Gazebo (Classic).

Tested in ROS2 Humble Hawksbill.
For physical control of the robot, the robot controller must have ROS 2 Package (S568).

## Content
This repository currently includes two packages:
* The robot_bringup package includes launch files for the Gazebo and MoveIt simulation & path planning.
* The robot_motion_planning package contains scripts for trajectory planning and execution of the robot. 

## Package installation

To install the four packages from inside your workspace:
```console
cd src
git clone https://github.com/PronkLJ/crx10ial
```

The FANUC ROS2 drivers for the hardware interface are supplied by FANUC.

## Simulation - main launch file
### Launch MoveIt2
```console
ros2 launch robot_bringup main.launch.py sim:=true
```

## Physical control
### Launch FANUC ROS interface and MoveIt2
Terminal 1:
```console
ros2 launch fanuc_ros2_driver fanuc_interface.launch.py robot_type:="crx10ia_l" robot_ip:="[IP address]" 
```
Terminal 2:
```console
ros2 launch robot_bringup main.launch.py sim:=false
```

### Moving the robot
```console
ros2 run robot_motion_planning dynamic_move_program [x] [y] [z]
```
```console
ros2 run robot_motion_planning move_to_home
```

## To-do
* Add functionality to the end effector

## Known bugs
* Gazebo Sim: joints are not receiving enough effort to maintain correct position (might be ROS2 Humble related?)

