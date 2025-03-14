# Robot_motion_planning package
This package includes multiple scripts to control the movement of the robotic arm.
The script can roughly be divided into constraints for the movement, and planning & execution scripts.

## Planning & executing trajectories
Currently, there are three different scripts for planning & executing trajectories.

### dynamic_move_program
Moves to a position based on given arguments
```console
ros2 run robot_motion_planning dynamic_move_program [x] [y] [z]
```

### move_to_home
Moves to a predefined homing position by setting joint angles
```console
ros2 run robot_motion_planning move_to_home
```

### static_move_program
Moves to a position predefined in the script, mostly used for testing
```console
ros2 run robot_motion_planning static_move_program
```

## Constraining scripts
There are two scripts that constrain movement, to ensure movement is valid in the real world.
Their variables can be adjusted in add_ground_plane.cpp and add_ceiling_plane.cpp.
Additionally, they can be disabled by commenting the node in robot_bringup/launch/control.launch.py and robot_bringup/launch/simulation.launch.py
If desired, the dimensions of these constraints can be edited in their respective .cpp files. 

## Known bugs
* Planning the path to the same position can lead to different trajectories, of which not all are possible with the physical robot (near-collission terminates the rest of the trajectory).