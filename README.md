# Stage BFS Navigation (ROS 2 Humble)

ROS 2 workspace for:
- Stage simulator
- Mapping with SLAM Toolbox
- Navigation with BFS path planning
- RViz visualization

## Workspace structure
- `src/` – ROS 2 packages
- `maps/` – saved maps (PGM + YAML)

## Build
```bash
colcon build --symlink-install
source install/setup.bash
