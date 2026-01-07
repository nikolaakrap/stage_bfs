# Stage BFS Navigation in ROS 2 Humble

This repository contains a ROS 2 workspace for mobile robot navigation in the Stage simulator.
The project demonstrates:

- 2D simulation using Stage
- Path planning using a custom Breadth-First Search (BFS) algorithm
- Reactive obstacle avoidance and recovery behavior
- Visualization of explored nodes, planned path, goals, and robot motion in RViz2

---

## Prerequisites

### 1. Ubuntu 22.04
This project is designed and tested on Ubuntu 22.04.

### 2. ROS 2 Humble
The project uses ROS 2 Humble Hawksbill.

Install ROS 2 Humble by following the official guide:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

After installation, source the ROS environment:
```bash
source /opt/ros/humble/setup.bash
```

### 3. Required ROS 2 Packages

Install the following packages:
```bash
sudo apt update
sudo apt install   ros-humble-nav2-map-server   ros-humble-nav2-lifecycle-manager   ros-humble-slam-toolbox   ros-humble-teleop-twist-keyboard   ros-humble-tf-transformations   ros-humble-rviz2
```

Additional system dependencies:
```bash
sudo apt install git python3-opencv
```

---

## Creating the Project

### 1. Create the Workspace
```bash
mkdir -p ~/stage_bfs_ws/src
cd ~/stage_bfs_ws
```

### 2. Clone the Repository (with submodules)
This repository uses git submodules for Stage and stage_ros2.

```bash
git clone --recurse-submodules https://github.com/<your_username>/stage_bfs_ws.git
```

If you already cloned without submodules:
```bash
git submodule update --init --recursive
```

### 3. Install Dependencies
```bash
cd ~/stage_bfs_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build and Source the Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Project

### 1. Launch the Simulation and Navigation
This launch file starts:
- Stage simulator
- BFS navigation node

```bash
ros2 launch stage_bfs_navigation stage_bfs_nav.launch.py
```

---

### 2. RViz Configuration
In RViz2, ensure the following:

- Fixed Frame: map
- Add the following displays:
  - Map (/map)
  - TF
  - LaserScan (/base_scan)
  - MarkerArray (/explored_nodes)
  - Marker (/path_marker)
  - MarkerArray (/goal_markers)
  - Marker (/target_waypoint)

For the Map display, set QoS:
  - Reliability: Reliable
  - Durability: Transient Local

---

### 3. Setting Start and Goal
Use the Publish Point tool in RViz:

1. First click sets the START position
2. Second click sets the GOAL position and triggers BFS planning

Markers:
- Green sphere: Start
- Red sphere: Goal
- Cyan spheres: Explored BFS nodes
- Green line: Planned path
- Yellow sphere: Current target waypoint

---

### 4. Navigation Behavior
- The robot follows the BFS-generated path using a simple proportional controller
- LaserScan data is used to detect obstacles in front of the robot
- If an obstacle is detected:
  - The robot stops
  - Moves backward
  - Rotates in place
  - Automatically replans from its new position

---