# Tiago MoveItPy boilerplate for Gazebo Harmonic

This package is a MoveItPy demonstrator for Tiago simulated in Tiago Harmonic / ROS Jazzy.

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


## Troubleshooting

### `RuntimeError: Failed to load planning pipelines from parameter server`
Do NOT run `pick.py` out of the launch file, as it would not load the planning parameters in that case.

To better understand, read the [MoveItPy documentation about planning parameters](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html#understanding-planning-parameters).

### `Failed to read controllers from [...] within 1 seconds`
If motion planning and successfuly but only motion execution fails, check that the controller is ready to execute motions by running `ros2 control list_controllers`.

If it is ready but you still get this error message, try re-compiling MoveIt with a higher service call timeout. See https://github.com/moveit/moveit2/pull/3382

