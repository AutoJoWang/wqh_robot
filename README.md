# Axioma 4WD Autonomous Mobile Robot

## Quick Start

```bash
# 1. Install ROS2 Humble (Ubuntu 22.04)
sudo apt update && sudo apt install ros-humble-desktop

# 2. Install project dependencies
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete \
                     ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
                     ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
                     ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
                     ros-humble-rviz2 ros-humble-xacro ros-humble-tf2-tools

# 3. Clone and build
mkdir -p ~/ros2/axioma_ws/src
cd ~/ros2/axioma_ws/src
git clone https://github.com/MrDavidAlv/Axioma_robot.git .
cd ~/ros2/axioma_ws
colcon build --symlink-install
source install/setup.bash

# 4. Launch SLAM (mapping)
ros2 launch axioma_bringup slam_bringup.launch.py

# Or launch autonomous navigation (requires a saved map)
ros2 launch axioma_bringup navigation_bringup.launch.py
```

See [Installation](#installation) and [Usage](#usage) for detailed instructions.

---


## Description

This project implements autonomous navigation software using ROS2 for the **Axioma.io** mobile robot platform. The system integrates SLAM (Simultaneous Localization and Mapping) for real-time map construction, AMCL (Adaptive Monte Carlo Localization) for pose estimation, and Nav2 for trajectory planning and obstacle avoidance. The robot is designed for autonomous material transport in industrial production lines.

### Objectives

- Design a 3D simulation environment that replicates real workspaces with static and dynamic obstacles
- Equip the simulated robot with navigation sensors (LiDAR, encoders, IMU)
- Implement the ROS2 ecosystem for localization (AMCL), control (differential drive), and navigation (Nav2)
- Develop trajectory planning under kinematic constraints and obstacle avoidance
- Integrate the software stack with the physical Axioma.io robot

### Keywords

Mobile robot, autonomous navigation, industrial logistics, trajectory planning, ROS2 Humble, Gazebo Harmonic, Nav2, SLAM, differential drive, skid-steering


## System Architecture

### Transform Tree (TF)

<div align="center">
<img src="images/URDF-TF.png" width="800"/>
</div>

Spatial transform tree: `map -> odom -> base_footprint -> base_link -> sensors`. The `odom_to_tf` node publishes the `odom -> base_link` transform from Gazebo odometry. AMCL publishes `map -> odom` to correct odometric drift during navigation.

### SLAM System

<div align="center">
<img src="images/SLAM.png" width="800"/>
</div>

SLAM Toolbox runs in asynchronous mode, building graph-based 2D occupancy grid maps in real time. It processes LiDAR scans at 5.5 Hz and odometry at 50 Hz with pose-graph optimization and loop closure detection.

### Navigation System

<div align="center">
<img src="images/Navigation.png" width="800"/>
</div>

The Nav2 stack integrates the NavFn global planner (Dijkstra), the DWB local controller (Dynamic Window Approach), dynamic costmaps with inflation and obstacle layers, and recovery behaviors (spin, backup, wait).

---

### Key Parameters

| Parameter | Value |
|-----------|-------|
| Wheel radius | $r = 0.0381$ m |
| Wheel separation | $W = 0.1725$ m |
| Total mass | $m = 5.525$ kg |
| Max linear velocity | $v_{max} = 0.26$ m/s |
| Max angular velocity | $\omega_{max} = 1.0$ rad/s |
| Max linear acceleration | $a_{max} = 2.5$ m/s² |
| Max angular acceleration | $\alpha_{max} = 3.2$ rad/s² |

**Differential kinematics:**

$$v = \frac{r(\omega_R + \omega_L)}{2}, \quad \omega = \frac{r(\omega_R - \omega_L)}{W}$$

---

## Requirements

### Software

- **Operating System**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Gazebo**: Harmonic (gz-sim 8)
- **Python**: 3.8+
- **CMake**: 3.16+

### ROS2 Dependencies

```
ros-humble-ros-gz              # Gazebo Harmonic integration
ros-humble-navigation2         # Full Nav2 stack
ros-humble-slam-toolbox        # SLAM mapping
ros-humble-rviz2               # Visualization
ros-humble-teleop-twist-keyboard   # Keyboard teleoperation
ros-humble-robot-state-publisher   # URDF TF publishing
ros-humble-tf2-tools           # TF debugging utilities
```


## Installation

### 1. Install ROS2 Humble

```bash
# Configure locale and repository
sudo apt update && sudo apt install locales curl
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 2. Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic ros-humble-ros-gz
```

### 3. Install Project Dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-tf2-tools

sudo rosdep init && rosdep update
```


## Build

```bash
cd ~/ros2/axioma_ws
colcon build --symlink-install
source install/setup.bash
```

To automatically source the workspace on every new terminal:

```bash
echo "source ~/ros2/axioma_ws/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### SLAM (Mapping)

Launch the simulation with SLAM Toolbox and RViz:

```bash
ros2 launch axioma_bringup slam_bringup.launch.py
```

In a separate terminal, control the robot to explore the environment:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or use the graphical teleoperation interface:

```bash
ros2 launch axioma_teleop_gui teleop_gui.launch.py
```

Save the map once the environment has been fully explored:

```bash
ros2 launch axioma_slam save_map.launch.py
```

### Autonomous Navigation

Launch the simulation with Nav2 and RViz (requires a previously saved map):

```bash
ros2 launch axioma_bringup navigation_bringup.launch.py
```

In RViz2:

1. Use **2D Pose Estimate** to set the robot initial pose
2. Use **2D Goal Pose** to send a navigation goal
3. Monitor the global/local costmaps, planned paths, and AMCL particle distribution

### Useful Commands

```bash
# Monitoring
ros2 node list                           # Active nodes
ros2 topic list                          # Active topics
ros2 topic hz /scan                      # LiDAR frequency
ros2 topic echo /cmd_vel                 # Velocity commands
ros2 run tf2_ros tf2_echo map base_link  # TF lookup
ros2 run tf2_tools view_frames           # TF tree diagram

# Debugging
ros2 node info /slam_toolbox
ros2 param list /controller_server
ros2 bag record -a -o navigation_data
```

---

## Project Structure

```
Axioma_robot/
├── src/
│   ├── axioma_bringup/            # Top-level launch orchestrators
│   │   └── launch/
│   │       ├── slam_bringup.launch.py
│   │       └── navigation_bringup.launch.py
│   │
│   ├── axioma_description/        # URDF model, meshes, RViz configs
│   │   ├── urdf/
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── launch/
│   │
│   ├── axioma_gazebo/             # Gazebo Harmonic simulation
│   │   ├── axioma_gazebo/
│   │   │   └── odom_to_tf.py      # Odometry to TF broadcaster
│   │   ├── models/axioma_v2/      # SDF model with meshes
│   │   ├── worlds/                # Simulation worlds
│   │   └── launch/
│   │       └── simulation.launch.py
│   │
│   ├── axioma_slam/               # SLAM Toolbox configuration
│   │   ├── config/
│   │   │   └── slam_params.yaml
│   │   ├── rviz/
│   │   └── launch/
│   │       ├── slam.launch.py
│   │       └── save_map.launch.py
│   │
│   ├── axioma_navigation/         # Nav2 configuration and maps
│   │   ├── config/
│   │   │   └── nav2_params.yaml
│   │   ├── maps/
│   │   ├── rviz/
│   │   └── launch/
│   │       └── navigation.launch.py
│   │
│   └── axioma_teleop_gui/         # PyQt5 teleoperation interface
│       ├── axioma_teleop_gui/
│       │   ├── main.py
│       │   ├── main_window.py
│       │   ├── ros_node.py
│       │   └── widgets/
│       │       ├── keyboard_mode.py
│       │       ├── joystick_mode.py
│       │       └── slider_mode.py
│       └── launch/
│           └── teleop_gui.launch.py
│
├── documentacion/
│   └── modelo-matematico/         # Kinematic and control documentation
│
├── images/                        # Documentation images
└── README.md
```
