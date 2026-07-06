# ego_2d_planner_pkg

EGO-style 2D planner for FAST-LIO2 + PX4.

This version intentionally follows the EGO-Planner workflow more strictly than the first lightweight implementation:

```text
/cloud_registered_filtered
  -> plan_env/GridMap2D: 2D occupancy + inflation + distance field
  -> path_searching/AStar2D: front-end guide path
  -> bspline_opt/UniformBspline2D: clamped B-spline control points
  -> bspline_opt/BsplineOptimizer2D: smoothness + collision + feasibility + fitness costs
  -> plan_manage/PlannerManager2D: EGO-style rebound optimization retries
  -> ego_replan_fsm_2d_node: INIT / WAIT_TARGET / GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ / EMERGENCY_STOP
  -> /simple_2d_planner/local_goal
```

Important behavior:

- An optimized B-spline must pass post-optimization collision checking.
- If optimized trajectory is unsafe, the planner retries rebound optimization with higher collision cost and larger safety distance.
- If all rebound attempts fail, it enters emergency hover and retries later.
- Raw A* path is published for visualization/debug only. It is not treated as a normal success path.

## Build

```bash
cd ~/ros2_ws_px4/src
unzip ego_2d_planner_pkg_ego_style.zip
cd ~/ros2_ws_px4
colcon build --packages-select ego_2d_planner_pkg
source install/setup.bash
```

## Run

```bash
ros2 launch ego_2d_planner_pkg ego_2d_planner.launch.py
```

## Send goal

```bash
ros2 topic pub --once /simple_2d_planner/goal geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'camera_init'
pose:
  position:
    x: 2.0
    y: 1.0
    z: 1.0
  orientation:
    w: 1.0
"
```

## Outputs

```text
/ego_2d_planner/occupancy_grid
/ego_2d_planner/raw_path
/ego_2d_planner/smooth_path
/ego_2d_planner/selected_path
/simple_2d_planner/local_goal
/ego_2d_planner/local_goal_marker
```
