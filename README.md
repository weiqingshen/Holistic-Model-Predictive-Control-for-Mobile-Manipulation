# Holistic Model Predictive Control for Mobile Manipulation

This repository contains the source code for our work on holistic model predictive control for mobile manipulation.

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 20.04 LTS
- **ROS Version**: ROS2 Humble
- **Python**: 3.8+
- **Compiler**: GCC 9.4+

## Installation

### Step 1: Install ROS2 Humble

If you haven't installed ROS2 Humble yet, follow the official installation guide:

```bash
# Install ROS2 Humble
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

### Step 2: Install Dependencies

```bash
# Install system dependencies
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update

# Install additional dependencies
sudo apt install python3-pip
pip3 install -r requirements.txt
```

### Step 3: Build the Workspace

Create a new workspace, configure it to build in release mode, and build the project:

```bash
mkdir -p ~/hmpc_ws/src
cd ~/hmpc_ws/src
git clone https://github.com/weiqingshen/Holistic-Model-Predictive-Control-for-Mobile-Manipulation.git
cd ~/hmpc_ws

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "source ~/hmpc_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Quick Start

### Task 1: Active Obstacle Avoidance

<div align="center">
 <img src="assets/task1.gif" alt="Task 1 - Active Obstacle Avoidance" width="80%">
</div>

```bash
# Terminal 1: Launch Rviz
ros2 launch mybot hmpc_demo.launch.py

# Terminal 2: Start Task 1 controller
ros2 launch mybot task1_hmpc.launch.py

# Terminal 3: Send random pose commands
ros2 run pose_publisher pose_publisher_task1_random
```

*Task 1 demonstrates active obstacle avoidance capabilities where the mobile manipulator navigates through dynamic environments while avoiding obstacles in real-time.*

### Task 2: Multi-Task Modular Demonstration

<div align="center">
 <img src="assets/task2.gif" alt="Task 2 - Multi-Task Modular Demonstration" width="80%">
</div>

```bash
# Terminal 1: Launch Rviz
ros2 launch mybot hmpc_demo.launch.py

# Terminal 2: Start Task 2 controller
ros2 launch mybot task2_hmpc.launch.py

# Terminal 3: Send task commands
ros2 run pose_publisher pose_publisher_task2
```

*Task 2 showcases multi-task modular capabilities, demonstrating how the system can seamlessly switch between different manipulation tasks while maintaining coordination between the mobile base and manipulator.*

### Task 3: Dynamic Target Following

<div align="center">
 <img src="assets/task3_v1.gif" alt="Task 3 - Dynamic Target Following" width="80%">
</div>

```bash
# Terminal 1: Launch Rviz
ros2 launch mybot hmpc_demo.launch.py

# Terminal 2: Start Task 3 controller
ros2 launch mybot task3_hmpc.launch.py

# Terminal 3: Start dynamic following
ros2 run pose_publisher pose_publisher_task3_follow
```

*Task 3 demonstrates dynamic target following where the mobile manipulator tracks and follows moving targets while maintaining precise end-effector positioning.*
