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
git clone https://github.com/PronkLJ/crx10ial
```

The FANUC ROS2 drivers for the hardware interface are supplied by FANUC.

Make sure that the following are properly installed in the ROS2 environment:
* MoveIt (main branch for Jazzy)

## Simulation control
### Launch MoveIt2 for simulation control
```console
ros2 launch crx10ial_bringup simulation.launch.py
```

## Physical control
### Launch FANUC ROS interface and MoveIt2 for physical control
Terminal 1:
```console
ros2 launch crx10ial_bringup hardware_interface.launch.py robot_ip:="[IP address]" 
```
Terminal 2:
```console
ros2 launch crx10ial_bringup control.launch.py
```

## To-do
- [ ] Re-enable motion control package
- [ ] Cartesian path planning through scripting
- [ ] Integrate with 2FG7 gripper
