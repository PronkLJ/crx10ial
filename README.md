# Fanuc CRX10iA/L packages for ROS2 simulation
The goal of these packages is to plan and execute paths in MoveIt 2, then displaying these movements in Gazebo (Classic).

Tested in ROS2 Humble Hawksbill.
For physical control of the robot, the robot controller must have ROS 2 Package (S568).

## Content
This repository currently includes four packages, of which only robot_bringup and robot_motion are actively used. The other two only function as reference.
* The robot_description package describes the crx10ial robot. Most of the URDF is from [the work of Paulo Franceshi](https://github.com/paolofrance/crx_description) (Depricated, now uses URDF from FANUC).
* The robot_bringup package includes launch files for the Gazebo and MoveIt simulation & path planning.
* The robot_moveit_config package holds all the configuration files that are used for MoveIt path planning (Depricated, now uses MoveIt configurations from FANUC).

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

## Other Launch files (Depricated or under development)

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
* Add functionality to the end effector

## Known bugs
* Gazebo Sim: joints are not receiving enough effort to maintain correct position (might be ROS2 Humble related?)

