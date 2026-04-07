# ReloPush-BOSS: Optimization-Guided Nonmonotone Rearrangement Planning for a Car-Like Robot Pusher

![Demo_vid](doc/img/o13_gif_optimized.gif)


## Overview
ReloPush-BOSS is multi-object rearrangement task planner incorporated with motion planners for a non-holonomic robot within a limited workspace region enforced with optimization-based relocation planning. It is an extension work of the original ReloPush with heuristic `Prerelocation`. 

## Dependency
- The planner is built with C++ STL libraries, Eigen, and OMPL.
- Qt is required for visualization UI.
- ZeroMQ is required to send the planning result to other programs.

### Predefined Input
- Size of the workspace in rectangular shape
- Physical dimensions and the initial pose of the robot
- Physical size, possible pushing directions, the initial pose, and the goal position of each movable object (i.e. a box has four pushing directions)
- Parameters required for motion planning (i.e. nominal driving velocity, maximum turning radius, etc)

### Output
- A sequence of trajectories/actions to rearrange all movable objects to their designated positions.
- Visualization of resulting relocation paths.

### Parameter files
To separate parameters for motion planning and others, we use two different files to contain necessary parameters:
- `include/PathPlanningTools.h`: Contains parameters for motion planning
- `include/Parameters.cpp`: Contains other parameters

## Running a planning instance
After building the CMake project, run `ReloPush-BOSS` binary with the following arguments:

```
# arg1: instance file name in input folder
# arg2: instance index in the file
# arg3: mode (f: ReloPush-BOSS, u: ReloPush-BO, d: ReloPush-B, o: ReloPush)

# example run (with ReloPush-BOSS, 10 objects, index 0)
./ReloPush-BOSS ReloPush-BOSS_10_objects.txt 0 f
```

Alternatively, when the binary is started without an argument, it will use hard-coded planning context (shown on the top of the main function).


## Citing this work
```
@ARTICLE{11397135,
  author={Ahn, Jeeho and Mavrogiannis, Christoforos},
  journal={IEEE Robotics and Automation Letters}, 
  title={ReloPush-BOSS: Optimization-Guided Nonmonotone Rearrangement Planning for a Car-Like Robot Pusher}, 
  year={2026},
  volume={},
  number={},
  pages={1-8},
  keywords={Robots;Planning;Kinematics;Collision avoidance;Search problems;Physics;Optimization;Robot kinematics;Manipulators;Clutter},
  doi={10.1109/LRA.2026.3665070}}

```
