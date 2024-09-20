# Fanuc CRX10iA/L packages for ROS2 simulation
Tested in ROS2 Humble Hawksbill.

## Content
This repository currently includes four packages:
* The robot_description package describes the crx10ial robot. Most of the URDF is from [the work of Paulo Franceshi](https://github.com/paolofrance/crx_description).
* The robot_bringup package includes launch files for the Gazebo simulation.
* The robot_moveit_config package holds all the MoveIt files, generated by the MoveIt Setup Assistant.

## Package installation

To install the four packages from inside your workspace:
```console
$ cd src
$ git clone https://github.com/PronkLJ/crx10ial
```

## Launch files
### RViz with manual control of the joints
```console
$ ros2 launch robot_description view_robot.launch.py
```
### MoveIt motion planner
```console
$ ros2 launch robot_bringup moveit.launch.py
```
### Displaying model in Gazebo (no control yet)
```console
$ ros2 launch robot_bringup gazebo.launch.py
```

### Launch MoveIt and Gazebo
```console
$ ros2 launch robot_bringup test.launch.py
```

## To-do
* Bridge positions between MoveIt and Gazeb
