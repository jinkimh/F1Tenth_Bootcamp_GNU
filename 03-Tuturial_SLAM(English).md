# 🗺️ F1TENTH SLAM Toolbox Installation and Map Creation

This document provides instructions on installing `slam_toolbox` on the F1TENTH platform, generating maps via real-time SLAM, visualizing them in RViz2, and saving them for later use.

---

## 1. Install slam\_toolbox

```bash
sudo apt install ros-foxy-slam-toolbox
```

> ⚠️ Properly tuned odometry is required to achieve accurate SLAM performance.

---

## 2. Network and Device Setup

* **NoMachine** must be connected between the host PC and the F1Tenth AV.
* IPs `192.168.0.10` and `192.168.0.15` are used for **Hokuyo LiDAR Ethernet**; ensure network separation so they are not blocked.
* Even with remote access via NoMachine, an **HDMI dongle must be connected to the F1Tenth AV** for keyboard and display interaction.

---

## 3. SLAM Execution Steps

### 3.1. Setup ROS2 and Workspace

```bash
cd ~/f1tenth_ws
colcon build
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

### 3.2. Launch Vehicle System

```bash
ros2 launch f1tenth_gym_ros bringup_launch.py
```

> Starts vehicle sensors and driving-related nodes.

### 3.3. Launch RViz2 (in another terminal)

```bash
source /opt/ros/foxy/setup.bash
rviz2
```

---

## 4. Run SLAM Toolbox

```bash
source /opt/ros/foxy/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/home/yourid/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
```

> Replace `"yourid"` with your actual Linux username.

---

## 5. RViz2 Visualization Setup

In RViz2, add the following to enable SLAM visualization:

* **Add by Topic**:

  * `/map`
  * `/graph_visualization`

* **Add Panel**:

  * Top menu → `Panels` → `Add New Panel`
  * Select `SlamToolBoxPlugin`

---

## 6. Save Map in RViz

1. In the `SlamToolBoxPlugin` panel, click `"Save Map"`
2. Enter a map name (e.g., `my_map`)
3. The map will be saved in the directory where SLAM Toolbox was launched.

### Example generated files:

* `my_map.pgm`
* `my_map.yaml`
* (optionally `.csv` or `.txt` files)

> Default save location: `~/f1tenth_ws`

---

## 7. NoMachine Connection Summary

**To remotely connect to the F1Tenth AV:**

1. Run **NoMachine** on the host PC
2. **Add new connection** → Enter the F1Tenth AV IP
3. **Protocol**: SSH
4. Enter **username/password** to connect

> HDMI dongle is required for GUI and keyboard input.

---

## 8. Full Execution Summary

```bash
# 1. Launch bringup (Terminal 1)
ros2 launch f1tenth_gym_ros bringup_launch.py

# 2. Launch SLAM Toolbox (Terminal 2)
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/home/yourid/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml

# 3. Launch RViz (Terminal 3)
rviz2
```

---

## 🔎 Notes

* Accurate odometry and LiDAR alignment are essential for reliable SLAM results.
* Along with `.pgm` and `.yaml`, `.csv` graph files may also be generated.
* RViz performance may degrade during SLAM; reduce active panels in resource-limited environments.

---

## 📄 License

```text
© 2025 Jin Kim, Gyeongsang National University

This document is prepared for experimentation and education using the F1Tenth autonomous driving platform.  
Unauthorized reproduction or commercial use is prohibited.
```
