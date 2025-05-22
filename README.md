# Fanuc CRX10iA/L packages for ROS2 simulation
The packages in this repository allow for the planning and execution of trajectory paths for the FANUC CRX10iA/L. 
In combination with the FANUC ROS2 node, these packages can control the physical robot as well.

Tested in ROS2 Humble Hawksbill.
> [!NOTE]
> For physical control of the robot, the robot controller must have ROS 2 Package (S568).

> [!NOTE]
> To include the softgripper fingers, [its repository](https://github.com/PronkLJ/softgripper) also has to be cloned and sourced.

## Content
This repository currently includes the following packages:
* The robot_description package describes the CRX10iA/L robot.
* The robot_bringup package includes launch files for the Gazebo and MoveIt simulation & path planning.
* The robot_motion_planning package contains scripts for trajectory planning and execution of the robot. 
* The robot_moveit_config package holds all the configuration files that are used for MoveIt path planning 

## Package installation

To install the packages from inside your workspace:
```console
cd src
git clone https://github.com/PronkLJ/crx10ial
```

The FANUC ROS2 drivers for the hardware interface are supplied by FANUC.

Make sure that the following are properly installed in the ROS2 environment:
* MoveIt Humble

## Simulation - main launch file
### Launch MoveIt2 + Gazebo for simulation control
```console
ros2 launch robot_bringup main.launch.py sim:=true
```

## Physical control
### Launch FANUC ROS interface and MoveIt2 for physical control
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
- [ ] Dummy UR/AR4 robot
- [ ] Gazebo environment with 2 different robots
- [ ] Cartesian path planning