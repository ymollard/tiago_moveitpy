# Tiago MoveItPy boilerplate for Gazebo Harmonic

This packge is a MoveItPy demonstrator for Tiago simulated in Tiago Harmonic / ROS Jazzy.

## Quickstart

```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place
ros2 launch tiago_moveitpy plan.launch.py use_sim_time:=True
```

You will observe Tiago performingg 3 sequential motions:
- **Plan 1**: Give a joint-space goal to MoveItPy 
- **Plan 2**: Give a cartesian-space goal to MoveItPy 
- **Plan 3**: Command opening/closure of the gripper

Checkout the code in `pick.py` to (un)comment the motion plans that you need and customize joint or cartesian coordinates.

