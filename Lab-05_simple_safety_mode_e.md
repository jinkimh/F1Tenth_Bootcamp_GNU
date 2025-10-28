## Lab-05: Implementation of simple_safety_node in the F1TENTH Simulator**

---

### Assignment Overview

This individual lab focuses on implementing and running the **simple_safety_node** in the **F1TENTH simulator** under the **ROS 2 Foxy** environment.
The goal is to understand the basic safety logic of autonomous vehicles by writing a node that subscribes to LiDAR `/scan` data and issues deceleration or stop commands based on a safety-distance rule.

---

### Assignment Requirements

| Category                    | Description                                                                     |
| --------------------------- | ------------------------------------------------------------------------------- |
| **Required Implementation** | Create the `simple_safety_node` package, write the node code, build, and run it |
| **Environment**             | ROS 2 Foxy + F1TENTH simulator integration                                      |
| **Functionality**           | Input `/scan` → evaluate safety distance → output `/drive`                      |
| **Validation**              | Verify `/drive` deceleration/stop commands and confirm structure in `rqt_graph` |

---

### Evaluation Criteria (Total = 100 points)

| Item                                  | Points | Evaluation Basis                                 |
| ------------------------------------- | ------ | ------------------------------------------------ |
| Simulator operation                   | 20 pts | `/scan` and `/drive` topics appear correctly     |
| Code implementation                   | 25 pts | Structure, comments, Foxy API compatibility      |
| `package.xml` and `setup.py` accuracy | 15 pts | Correct dependencies and entry points            |
| Build and execution                   | 20 pts | Successful `colcon build` and node execution     |
| Safety logic behavior                 | 20 pts | Proper deceleration/stop output and log analysis |

---

### Submission Materials

1. Entire directory `/sim_ws/src/simple_safety_node` (including `package.xml` and source code)
2. Screenshots or log captures (simulator + node running)
3. Short execution summary or PDF report of results

---

## Environment (ROS 2 Foxy)

| Item              | Description                                           |
| ----------------- | ----------------------------------------------------- |
| **OS**            | Ubuntu 20.04 LTS                                      |
| **Python**        | 3.8 (default in Foxy)                                 |
| **ROS 2 Version** | Foxy Fitzroy                                          |
| **Workspace**     | `/sim_ws` (structure: `/sim_ws/src` → `colcon build`) |

Setup:

```bash
$ source /opt/ros/foxy/setup.bash
```

Dependency:

```bash
$ sudo apt install ros-foxy-ackermann-msgs
```

---

## Step 1 – Check F1TENTH Simulator Operation

Reference:
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/1-Tutorial_F1Tenth_Simulator(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/1-Tutorial_F1Tenth_Simulator%28English%29.md)

```bash
# (new terminal)
$ source /opt/ros/foxy/setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Foxy compatibility check**

* The simulator should start without errors and open RViz or Gazebo.
* `ros2 topic list` should include:

  * `/scan` (sensor_msgs/LaserScan)
  * `/drive` (ackermann_msgs/AckermannDriveStamped)
  * `/ego_racecar/odom` (may vary by repository)
* If errors occur, verify:

  * The repository was built for Foxy
  * Workspace sourcing: `source ~/f1tenth_ws/install/setup.bash`

---

## Step 2 – Create the simple_safety Package

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ cd src
/sim_ws/src$ ros2 pkg create --build-type ament_python simple_safety_node
```

Resulting structure:

```
simple_safety_node/
  package.xml
  setup.py
  setup.cfg
  resource/simple_safety_node
  simple_safety_node/__init__.py
```

`ament_python` build type is officially supported in Foxy.

---

## Step 3 – Implement the Node

```bash
/sim_ws/src$ cd simple_safety_node/simple_safety_node
/sim_ws/src/simple_safety_node/simple_safety_node$ nano simple_safety_node.py
```

Copy and paste the code from:
[https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py)

**Foxy compatibility checklist**

```python
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
```

* Input topic: `/scan`
* Output topic: `/drive`
  (use remapping if `/ego_racecar/scan` is required)
* Confirm use of `declare_parameter()` and `get_parameter()`—both valid in Foxy.

---

## Step 4 – Edit setup.py

```bash
/sim_ws/src/simple_safety_node$ cd ..
/sim_ws/src/simple_safety_node$ nano setup.py
```

---

## Step 5 – Modify setup.py

Reference:
[https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/setup.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/setup.py)

Essential section:

```python
entry_points={
    'console_scripts': [
        'simple_safety_node = simple_safety_node.simple_safety_node:main',
    ],
},
```

Keep:

```python
install_requires=['setuptools']
zip_safe=True
packages=['simple_safety_node']
```

---

## package.xml Dependencies

```xml
<!-- /sim_ws/src/simple_safety_node/package.xml -->
<package format="3">
  <name>simple_safety_node</name>
  <version>0.0.1</version>
  <description>Simple safety node for F1TENTH (Foxy)</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>ackermann_msgs</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export/>
</package>
```

If missing:

```bash
$ sudo apt install ros-foxy-ackermann-msgs
```

---

## Step 6 – Build and Source

```bash
/sim_ws/src/simple_safety_node$ cd /sim_ws
/sim_ws$ colcon build
/sim_ws$ source install/setup.bash
```

**Test run**

```bash
$ ros2 run simple_safety_node simple_safety_node
```

If errors appear:

* Check `entry_points` spelling
* Verify `package.xml` dependencies
* Ensure sourcing order:
  `source /opt/ros/foxy/setup.bash` → `source /sim_ws/install/setup.bash`

---

## Execution Procedure (ROS 2 Foxy)

### Step 1 – Run the Simulator (Terminal 1)

```bash
$ source /opt/ros/foxy/setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

RViz or Gazebo should start, and `/scan` and `/drive` topics should appear.
Keep this terminal open.

### Step 2 – Run simple_safety_node (Terminal 2)

```bash
$ source /opt/ros/foxy/setup.bash
$ cd ~/sim_ws
$ source install/setup.bash
$ ros2 run simple_safety_node simple_safety_node
```

The node subscribes to `/scan` and publishes `/drive` commands according to safety-distance or TTC thresholds.

**Verification**

```bash
$ ros2 topic echo /drive
```

If deceleration or stop commands appear, the node is functioning properly.

---

### Operation Checklist

| Item              | Command                  | Expected Result                                       |
| ----------------- | ------------------------ | ----------------------------------------------------- |
| `/scan` reception | `ros2 topic echo /scan`  | Continuous distance data                              |
| `/drive` output   | `ros2 topic echo /drive` | AckermannDriveStamped messages                        |
| Graph structure   | `rqt_graph`              | Connection: `/scan` → `simple_safety_node` → `/drive` |

---
 
