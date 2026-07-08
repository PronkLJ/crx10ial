# Fanuc CRX10iA/L packages for ROS2 simulation
The packages in this repository allow for the planning and execution of trajectory paths for the FANUC CRX10iA/L. 
In combination with the FANUC ROS2 node, these packages can control the physical robot as well.

Tested in ROS2 Jazzy Jalisco.
> [!NOTE]
> For physical control of the robot, the robot controller must have ROS 2 Package (S568).

## Content
This repository currently includes the following packages:
* The crx10ial_description package describes the CRX10iA/L robot.
* The crx10ial_bringup package includes launch files for simulation & physical control.
* The crx10ial_moveit_config package holds all the configuration and launch files that are used for MoveIt path planning.

## Package installation

To install the packages from inside your workspace:
```console
cd src
git clone https://github.com/IdPDE/...
```
Example:
Make sure that the following are properly installed in the ROS2 environment:
* MoveIt (main branch for Jazzy)

## Execution of the project
Here, you can add descriptions on how users can run the program/project.
For example:
### Launch MoveIt2 for simulation control
```console
ros2 launch crx10ial_bringup simulation.launch.py
```

## To-do
- [ ] First thing I still want to do
- [ ] Second thing I still want to do
- [x] A thing I finished doing
