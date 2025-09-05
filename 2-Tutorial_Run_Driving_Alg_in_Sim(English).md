# 🏎️ F1Tenth Simulator Usage – Running Driving Algorithms (Wall Following & Pure Pursuit)

This document explains how to run **Wall Following** and **Pure Pursuit** algorithms in the F1Tenth simulator environment.

---

## 1. Setup ROS2 Environment and Build

In the simulator workspace (`/sim_ws`), run:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash   # Setup ROS2 Foxy
/sim_ws$ colcon build                      # Build workspace
/sim_ws$ source install/setup.bash         # Source built packages
```

* `setup.bash`: Load ROS2 environment variables
* `colcon build`: Build ROS2 workspace

---

## 2. Run Wall Following

### (1) Launch Simulator

In a new terminal inside the Docker container:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

* `gym_bridge_launch.py`: Launches the GUI simulator and connects to ROS2

---

### (2) Run Wall Following Node

In another terminal inside the container:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 run wall_follow wall_follow_node.py
```

* `wall_follow_node.py`: Runs the wall-following driving algorithm

---

## 3. Run Pure Pursuit

The Pure Pursuit algorithm runs from its own node or package, similar to Wall Following.

### (1) Launch Simulator

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

---

### (2) Run Pure Pursuit Node

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 run pure_pursuit pure_pursuit_node.py
```

* `pure_pursuit_node.py`: Follows a given waypoint path

---

## 4. Key Files and Directories

### (1) Map Files

* Extensions: `.png`, `.pgm`, `.yaml`
* Purpose: Map images (obstacles, paths) + metadata (resolution, origin, etc.)
* Location:

```bash
src/f1tenth_gym_ros/maps/
```

---

### (2) Waypoint Files

* Extension: `.csv`
* Purpose: Coordinate data for Pure Pursuit paths
* Location:

```bash
src/f1tenth-software-stack/csv_data/
```

* Recommended: Use same naming as map file (`map_xxx.png` ↔ `map_xxx.csv`)

---

## 5. Running with a New Map

1. Add map file to `src/f1tenth_gym_ros/maps/`
2. Add corresponding waypoint file to `src/f1tenth-software-stack/csv_data/`
3. Launch simulator → Run Pure Pursuit or Wall Following node

---

## 6. Conclusion

By configuring custom maps and waypoints, you can perform diverse simulations and compare/improve the performance of **Wall Following** and **Pure Pursuit** algorithms.

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

--- 
