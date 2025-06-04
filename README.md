# Holistic Model Predictive Control for Mobile Manipulation

This repository contains the source code for our work holistic model predictive control for mobile manipulation.

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

## ## Quick Start

### Running the Basic Example

```bash
# Terminal 1: Launch Rviz
ros2 launch mybot hmpc_demo.launch.py

# Terminal 2: 
ros2 launch mybot task1_hmpc.launch.py

# Terminal 3: Send test commands
ros2 run pose_publisher pose_publisher_task1_random
```

#### Demo

### Task Demonstrations

Our holistic model predictive control approach is demonstrated across multiple manipulation tasks, comparing simulation results with real robot performance.

#### Task 1: Basic Mobile Manipulation

<div align="center">
 <img src="assets/task1.gif" alt="Task 1 - Basic Mobile Manipulation" width="80%">
</div>

*Task 1: Fundamental mobile manipulation scenario demonstrating coordinated base and arm motion.*

#### Task 2: Advanced Manipulation

<div align="center">
 <img src="assets/task2.gif" alt="Task 2 - Advanced Manipulation" width="80%">
</div>

*Task 2: Complex manipulation task showcasing advanced coordination between mobile base and manipulator.*

#### Task 3: Precision Control (Version 1)

<div align="center">
 <img src="assets/task3_v1.gif" alt="Task 3 v1 - Precision Control" width="80%">
</div>

*Task 3 Version 1: High-precision manipulation demonstrating the accuracy of our holistic MPC approach.*

#### Task 3: Precision Control (Version 2)

<div align="center">
 <img src="assets/task3_v2.gif" alt="Task 3 v2 - Precision Control" width="80%">
</div>

*Task 3 Version 2: Enhanced precision control with improved performance metrics.*
