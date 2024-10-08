# Fanuc CRX10iA/L packages for ROS2 simulation
The goal of these packages is to plan and execute paths in MoveIt 2, then displaying these movements in Gazebo (Classic).

Tested in ROS2 Humble Hawksbill.

## Content
This repository currently includes four packages:
* The robot_description package describes the crx10ial robot. Most of the URDF is from [the work of Paulo Franceshi](https://github.com/paolofrance/crx_description).
* The robot_bringup package includes launch files for the Gazebo and MoveIt simulation & path planning.
* The robot_moveit_config package holds all the configuration files that are used for MoveIt path planning.

## Package installation

To install the four packages from inside your workspace:
```console
cd src
git clone https://github.com/PronkLJ/crx10ial
```

## Main Launch File
### Launch Gazebo (Classic) and MoveIt2
```console
ros2 launch robot_bringup simulation.launch.py
```

## Other Launch files

### Launch Gazebo (Ignition) and MoveIt2
```console
ros2 launch robot_bringup simulation2.launch.py
```
### RViz with manual control of the joints
```console
ros2 launch robot_description view_robot.launch.py
```
### MoveIt motion planner
```console
ros2 launch robot_bringup moveit.launch.py
```
### Displaying model in Gazebo Classic (no control)
```console
ros2 launch robot_bringup gazebo_classic.launch.py
```

### Displaying model in Gazebo (Ignition) (no control)
```console
ros2 launch robot_bringup gazebo_sim.launch.py
```

## To-do
* Create input variable for a position that the robot arm moves to (automated MoveIt)
* Connect to physical robot
* Add functionality to the end effector

## Known bugs
* Gazebo Sim: joints are not receiving enough effort to maintain correct position
