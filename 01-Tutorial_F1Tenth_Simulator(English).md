# 🏎️ Running the F1Tenth AV Simulator

## 1. Prerequisites

To run the F1Tenth Simulator, the following environment is required.

### Step 1: Install Ubuntu or WSL (Ubuntu)

* **Recommended version:** Ubuntu 22.04 or 20.04

* **For Windows users:**
  You can configure an Ubuntu environment using **WSL (Windows Subsystem for Linux)** to run Linux commands and development tools.

#### WSL Installation

1. **Run PowerShell as Administrator**

   * Search for `PowerShell` → Select **Run as administrator**

2. **Install WSL and Ubuntu**

   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

---

### Step 2: Install ROS2

* **Required version:** ROS2 Foxy (Humble supported as well)
* ROS2 is the core framework for running the F1Tenth simulator and ROS nodes.
* Follow the official guide: [ROS2 Foxy Installation Guide](https://docs.ros.org/en/foxy/Installation.html)

---

### Step 3: Install Docker

* **Ubuntu:** [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
* **Windows 11:** [Install Docker Desktop](https://docs.docker.com/desktop/install/windows-install/) and run in Linux container mode.

---

## 2. Build F1Tenth Docker Image

### 1. Create Workspace

```bash
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src
```

### 2. Clone Repositories

```bash
git clone https://github.com/jinkimh/f1tenth_gym_ros.git
git clone https://github.com/jinkimh/f1tenth-software-stack.git
```

### 3. Build Docker Image

```bash
cd f1tenth_gym_ros
docker build -t f1tenth_gym_ros -f Dockerfile .
```

---

## 3. Run Docker Container

### Option 1: Run with Command

```bash
cd ~/f1tenth_ws
xhost +local:docker

docker run -it \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/f1tenth_ws/src/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros" \
  --volume="$HOME/f1tenth_ws/src/f1tenth-software-stack:/sim_ws/src/f1tenth-software-stack" \
  --name f110_gym_docker \
  f1tenth_gym_ros:latest
```

Prompt example after container starts:

```
root@12ff300395b4:/sim_ws#
```

---

### Option 2: Run with Script

Create a script `docker_run_f1tenth_sim.sh`:

```bash
#!/bin/bash
# F1Tenth Simulator Docker Run Script

cd ~/f1tenth_ws || { echo "f1tenth_ws does not exist."; exit 1; }

xhost +local:docker

docker run -it \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/f1tenth_ws/src/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros" \
  --volume="$HOME/f1tenth_ws/src/f1tenth-software-stack:/sim_ws/src/f1tenth-software-stack" \
  --name f110_gym_docker \
  f1tenth_gym_ros:latest
```

Grant execution and run:

```bash
chmod +x docker_run_f1tenth_sim.sh
./docker_run_f1tenth_sim.sh
```

---

## 4. Run Simulator and `wall_follow`

### Step 1: Launch Simulator

```bash
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

A GUI simulator window will appear:

<img src="https://github.com/jinkimh/f1tenth_bootcamp/blob/main/figures/sim_rviz.png" alt="F110th Simulator" width="600"/>

---

### Step 2: Connect to Container in a New Terminal

```bash
docker ps -a  # check container ID
docker exec -it <container_id> /bin/bash
```

---

### Step 3: Run `wall_follow`

```bash
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
ros2 run wall_follow wall_follow_node.py
```

---

## 5. Verification

### Check ROS2 Topics

```bash
ros2 topic list
```

Verify key topics: `scan`, `odom`, `cmd_vel`.

### Check Logs

Inspect messages from `wall_follow_node.py` to confirm status.

---

## ✅ Result

* The vehicle follows the wall in the simulator.
* Once verified, you can extend with custom algorithms.

---

# 🧭 ROS2 Teleop Twist Keyboard Guide

This guide explains how to set up the **ROS2 environment inside Docker** and control the vehicle manually using the `teleop_twist_keyboard` node.

---

## 🚪 0. Re-enter Docker Container

If the container `f110_gym_docker` already exists:

```bash
docker start f110_gym_docker
docker exec -it f110_gym_docker bash
```

For GUI-enabled mode:

```bash
docker exec -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 f110_gym_docker bash
```

---

## ⚙️ 1. Setup ROS2 Environment

Inside the container:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

---

## 🕹️ 2. Run teleop\_twist\_keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🎮 3. Keyboard Controls

| Action         | Key        |
| -------------- | ---------- |
| Forward        | `i`        |
| Backward       | `,`        |
| Turn Left      | `j`        |
| Turn Right     | `l`        |
| Increase Speed | `u` / `m`  |
| Decrease Speed | `o` / `.`  |
| Stop           | `k`        |
| Quit           | `Ctrl + C` |

---

## 📝 Notes

* Always source ROS2 and workspace before running:

  ```bash
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  ```
* Keep the terminal **active and focused** when using teleop.

---

## 📄 Example Workflow

```bash
# 1. Start container
docker start f110_gym_docker
docker exec -it f110_gym_docker bash

# 2. Source environment
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 3. Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 📄 License

```
© 2025 Jin Kim, Gyeongsang National University

This document is prepared for ROS2 and F1Tenth practice.
Unauthorized reproduction or redistribution beyond educational/research use is prohibited.
```
 
