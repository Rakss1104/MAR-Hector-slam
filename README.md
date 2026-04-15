# ROS 2 Hector SLAM Simulation Workspace

A complete ROS 2 Humble workspace for **Hector SLAM** simulation in Gazebo, running on **Ubuntu 22.04 (WSL2)**.

---

## File Structure

```
ros2_hector_ws/
├── setup_env.sh                                        # WSL2 GUI environment variables
├── README.md                                           # This file
└── src/
    ├── hector_slam/                                    # ← Clone here (see below)
    └── my_robot_description/
        ├── CMakeLists.txt                              # ament_cmake build file
        ├── package.xml                                 # ROS 2 package manifest
        ├── config/
        │   └── hector_mapping_params.yaml              # Hector mapping parameters
        ├── launch/
        │   └── sim_hector.launch.py                    # Main launch file
        ├── rviz/
        │   └── hector_slam.rviz                        # RViz2 display config
        ├── urdf/
        │   └── diff_drive_robot.urdf.xacro             # Robot description
        └── worlds/
            └── classroom.world                         # 10m × 8m classroom
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| WSL2 (Ubuntu) | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Classic 11 (via `ros-humble-gazebo-ros-pkgs`) |
| X Server | VcXsrv / X410 / WSLg |

### Install ROS 2 dependencies (one-time)

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-ros \
  ros-humble-rviz2
```

---

## Setup

### 1. Clone Hector SLAM (ROS 2 port)

```bash
cd ~/ros2_hector_ws/src
git clone https://github.com/Rakss1104/MAR-Hector-slam.git
```


### 2. Install remaining dependencies

```bash
cd ~/ros2_hector_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build & Run (One-Liner)

```bash
source setup_env.sh && source /opt/ros/humble/setup.bash && cd ~/ros2_hector_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch my_robot_description sim_hector.launch.py
```

### Broken down:

```bash
# 1. Configure WSL2 display
source setup_env.sh

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Build workspace
cd ~/ros2_hector_ws
colcon build --symlink-install

# 4. Source workspace overlay
source install/setup.bash

# 5. Launch simulation
ros2 launch my_robot_description sim_hector.launch.py
```

---

## Drive the Robot

In a **new terminal** (with the workspace sourced):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_hector_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use `i`, `j`, `l`, `k` keys to drive. Watch the map build in RViz2!

---

## Key Configuration

| Parameter | Value | Purpose |
|---|---|---|
| `base_frame` | `base_link` | Robot base frame |
| `odom_frame` | `base_link` | SLAM without odometry |
| `map_update_distance_thresh` | `0.1` m | Map update distance |
| `map_update_angle_thresh` | `0.04` rad | Map update angle |
| `map_resolution` | `0.05` m/cell | Grid resolution |
| `map_size` | `2048` cells | Grid dimensions |

---

## Verify Topics

```bash
ros2 topic list
# Expected: /scan  /map  /tf  /cmd_vel  /odom  /robot_description
```

## Troubleshooting

| Issue | Fix |
|---|---|
| Black screen in Gazebo | Re-run `source setup_env.sh` and ensure X server is running |
| `hector_mapping` not finding `/scan` | Check `ros2 topic echo /scan` — LiDAR may not have spawned |
| No map in RViz2 | Ensure Fixed Frame is set to `map` |
| Build errors for `hector_slam` | Try the community fork (see Setup step 1) |
