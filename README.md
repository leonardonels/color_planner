# planning

Purpose
-------
Compute a centerline or trajectory between mapped cones. Uses cone color to
classify left/right and produces a line suitable for vehicle guidance.

Key files
- `src/color_planner.cpp`, `src/color_planner_node.cpp` — planner logic and
  node.
- `include/color_planner/color_planner.hpp` — helper utilities.

Topics
- Subscribes: global cone map (from `mapping/`).
- Publishes: planned line/trajectory (custom or standard ROS message), and
  visualization markers.

Run (example)
--------------
```bash
ros2 launch planning launch.py
```

Notes
- The planner assumes the map provides left/right color labels for cones.
- Add smoothing and collision checks as needed for vehicle integration.