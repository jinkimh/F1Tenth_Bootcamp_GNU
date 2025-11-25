
# F1TENTH Particle Filter Installation and Execution Guide

This document explains how to install and run the **Particle Filter** on the **F1TENTH** platform.
Optionally, you can install **range_libc** to use a fast laser range model.

---

## 1. (Optional) Install `range_libc`

Install `range_libc` if you want to use a faster laser sensor model.

```bash
cd ~
git clone https://github.com/f1tenth/range_libc.git
cd range_libc/pywrapper
sudo WITH_CUDA=ON python setup.py install
```

⚠️ If your system **does not have CUDA**, set `WITH_CUDA=OFF`.

---

## 2. Install the Particle Filter Package

```bash
# Move to the ROS workspace source directory and clone the repo
cd ~/f1tenth_ws/src
git clone https://github.com/f1tenth/particle_filter.git

# Install dependencies
cd ~/f1tenth_ws
rosdep install -r --from-paths src --ignore-src --rosdistro foxy -y

# Build the workspace
colcon build
source install/setup.bash
```

---

## 3. Run the Particle Filter

### 3.1. Run Teleop (in another terminal)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3.2. Run the Particle Filter Node

```bash
ros2 launch particle_filter localize_launch.py
```

---

## 4. Visualization in RViz2

Start RViz:

```bash
rviz2
```

### Additional RViz Configuration

#### Display the Map

* Add → By topic → **/map**
* Set **Durability Policy** to **Transient Local**

#### Display Localization Pose

* Topic: **/pf/viz/inferred_pose**

#### Display Particles (optional)

* Topic: **/pf/viz/particles**

---

## 5. Check Publishing Frequency

Verify that `/pf/viz/inferred_pose` runs at **30 Hz or higher**:

```bash
ros2 topic hz /pf/viz/inferred_pose
```

---

## 6. Change the Map

1. Copy your map files (`.pgm`, `.yaml`) into:

```
particle_filter/maps
```

2. Modify the map file name in:

```
particle_filter/config/localize.yaml
```

Example:

```yaml
map_yaml_file: maps/my_custom_map.yaml
```

---

## 7. Set the Initial Pose

In RViz, click **“2D Pose Estimate”** on the top toolbar and manually set the initial robot pose.
This initializes the Particle Filter.

---

## Summary Commands

```bash
# 1. Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 2. Run Particle Filter
ros2 launch particle_filter localize_launch.py

# 3. Run RViz visualization
rviz2
```

---

## Notes

* This guide is based on **ROS2 Foxy**.
* If RViz settings are incomplete, you may not see localization results.
  Always check **Topics** and **Durability settings**.

---

## 📄 Copyright

© 2025 Jin Kim, Gyeongsang National University
This document is created for experimental and educational use on the F1TENTH autonomous driving platform.
Unauthorized reproduction or commercial use is prohibited.

