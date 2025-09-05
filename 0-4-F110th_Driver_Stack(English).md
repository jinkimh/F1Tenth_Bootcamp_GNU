# 🚗 F1Tenth Driver Stack Setup and LiDAR Test

This guide explains how to **install the ROS2-based driver stack**, **test the LiDAR connection**, and **visualize data using RViz** for the F1TENTH autonomous vehicle.

---

## 📂 1. Create Workspace

Create a ROS2 workspace (`f1tenth_ws`) for the driver stack.

```bash
cd $HOME
mkdir -p f1tenth_ws/src
```

Initialize workspace:

```bash
cd f1tenth_ws
colcon build
```

---

## 📥 2. Clone Driver Stack Repository & Init Submodules

Clone the repository:

```bash
cd src
git clone https://github.com/f1tenth/f1tenth_system.git
```

Initialize and update submodules:

```bash
cd f1tenth_system
git submodule update --init --force --remote
```

---

## ⚙️ 3. Install Dependencies

Update `rosdep` and install dependencies:

```bash
cd $HOME/f1tenth_ws
rosdep update
rosdep install --from-paths src -i -y
```

---

## 🛠 4. Build the Workspace

```bash
colcon build
```

> 📎 For more details, see the [f1tenth\_system GitHub repository](https://github.com/f1tenth/f1tenth_system).

---

## 🎮 5. Teleop & LiDAR Test

Make sure LiDAR is connected to the vehicle.
(For **Hokuyo 10LX**, Ethernet connection must be configured first.)

---

## 📝 6. Configure LiDAR Parameters

Config file location:

```bash
$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/sensors.yaml
```

### Ethernet-based LiDAR (e.g., Hokuyo 10LX)

```yaml
ip_address: "192.168.x.x"  # Set LiDAR IP address
# serial_port should be commented out
```

### USB-based LiDAR

```yaml
# ip_address should be commented out
serial_port: "/dev/ttyUSB_LIDAR"  # Defined by udev rules
```

---

## 🚀 7. Setup ROS2 Environment & Run Bringup

Source ROS2 and workspace environment:

```bash
source /opt/ros/foxy/setup.bash
cd $HOME/f1tenth_ws
source install/setup.bash
```

Run bringup:

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

✅ This launches all required nodes (VESC driver, LiDAR driver, joystick driver, etc.).

---

## 👀 8. Visualize LiDAR in RViz

Run in a new terminal:

```bash
source /opt/ros/foxy/setup.bash
cd $HOME/f1tenth_ws
source install/setup.bash
rviz2
```

RViz Setup:

1. Add **LaserScan** display to visualize `/scan` topic.
2. View LiDAR distance data in real time.

---

## 📊 Summary

| Item            | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| Workspace       | `f1tenth_ws`                                                 |
| Main Repository | [f1tenth\_system](https://github.com/f1tenth/f1tenth_system) |
| Config Files    | `vesc.yaml`, `sensors.yaml`                                  |
| Run Command     | `ros2 launch f1tenth_stack bringup_launch.py`                |
| Visualization   | `rviz2`                                                      |

---

## 📄 License

```text
MIT License

Copyright (c) 2025 Jin Kim

This software and documentation are provided "as is", without warranty of any kind.  
Permission is granted, free of charge, to use, copy, modify, merge, publish, distribute, sublicense,  
and/or sell copies of the software, subject to the following conditions:

1. The above copyright notice and this permission notice shall be included  
   in all copies or substantial portions of the software.

2. The software is provided without warranty of any kind, including but not limited to  
   merchantability or fitness for a particular purpose. The authors are not liable for any damages.  
```
 
