# ** Lab-05 (Team Project) – Implementation of simple_safety_node on the F1TENTH Car (ROS 2 Foxy)**

---

## **1. Project Overview**

The goal of this project is to **implement and validate the simple_safety_node** on a real **F1TENTH autonomous vehicle (AV)**.
The team will use the **ROS 2 Foxy** environment to bring up the actual F1TENTH car, implement a **LiDAR-based safety control node**, and ensure that the safety logic for deceleration and stopping operates correctly.

The project involves writing, modifying, building, running, and testing a custom ROS 2 package that enforces a safety distance policy in real-time.

---

## **2. Team Structure and Roles (Example)**

| Role                            | Responsibilities                                 | Deliverables                          |
| ------------------------------- | ------------------------------------------------ | ------------------------------------- |
| **Team Leader**                 | Manage timeline, integrate code, finalize report | Execution logs, final report          |
| **Developer A (ROS)**           | Implement and modify `simple_safety_node`        | Python code (`simple_safety_node.py`) |
| **Developer B (System/Launch)** | Handle car bringup and run-time testing          | Launch logs, topic capture            |
| **Developer C (Validation)**    | Verify `/drive` command and analyze safety logic | Validation summary, graphs            |
| **Documentation Manager**       | Compile documentation and organize deliverables  | Final PDF report, screenshots         |

---

## **3. Development Environment**

| Item                 | Details                                    |
| -------------------- | ------------------------------------------ |
| **OS**               | Ubuntu 20.04 LTS                           |
| **Python**           | 3.8                                        |
| **ROS 2 Version**    | Foxy Fitzroy                               |
| **Workspace**        | `/f1tenth_ws`                              |
| **Required Package** | `sudo apt install ros-foxy-ackermann-msgs` |

### **ROS Environment Setup**

```bash
source /opt/ros/foxy/setup.bash
```

### **Vehicle Safety Guidelines**

* Place the vehicle **on a maintenance stand** (wheels should not touch the ground) for initial testing.
* Prepare the **E-stop** and start with **minimal speed parameters**.
* Perform all tests in an **indoor or safe environment**.

---

## **4. Implementation Steps**

### **Step 1. Create the simple_safety_node Package**

(Skip this step if the package already exists.)

```bash
cd ~/f1tenth_ws/src
ros2 pkg create --build-type ament_python simple_safety_node
```

Package structure:

```
simple_safety_node/
  package.xml
  setup.py
  setup.cfg
  resource/simple_safety_node
  simple_safety_node/__init__.py
```

---

### **Step 2. Node Implementation and Code Modification**

```bash
cd ~/f1tenth_ws/src/simple_safety_node/simple_safety_node
nano simple_safety_node.py
```

Base code:
[https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py)

#### **Key Modification – Remove Topic Namespace**

**Before (around line 34):**

```python
self.sub_odom = self.create_subscription(
    Odometry, '/ego_racecar/odom', self.odom_callback, 10
)
```

**After (namespace removed):**

```python
self.sub_odom = self.create_subscription(
    Odometry, '/odom', self.odom_callback, 10
)
```

#### **Topic Configuration**

* Input: `/scan`
* Output: `/drive`
* Optional remapping:
  `ros2 run ... --ros-args -r <from>:=<to>`

#### **Foxy-Compatible APIs**

```python
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
```

(`declare_parameter` and `get_parameter` are fully supported in Foxy.)

---

### **Step 3. Modify setup.py**

```bash
cd ~/f1tenth_ws/src/simple_safety_node
nano setup.py
```

```python
entry_points={
    'console_scripts': [
        'simple_safety_node = simple_safety_node.simple_safety_node:main',
    ],
},
install_requires=['setuptools'],
zip_safe=True,
packages=['simple_safety_node'],
```

---

### **Step 4. Update package.xml**

`~/f1tenth_ws/src/simple_safety_node/package.xml`

```xml
<package format="3">
  <name>simple_safety_node</name>
  <version>0.0.1</version>
  <description>Simple safety node for F1TENTH (Foxy)</description>
  <maintainer email="team@example.com">F1TENTH Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>ackermann_msgs</exec_depend>
</package>
```

---

### **Step 5. Build and Source**

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
```

Test run:

```bash
ros2 run simple_safety_node simple_safety_node
```

---

## **5. Vehicle Bringup**

**Terminal 1:**

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws/
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

Check topics:

```bash
ros2 topic list
```

Expected topics:

* `/scan` (sensor_msgs/LaserScan)
* `/odom` (nav_msgs/Odometry)
* `/drive` (ackermann_msgs/AckermannDriveStamped)

---

## **6. Run and Validate simple_safety_node**

**Terminal 2:**

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws/
source install/setup.bash
ros2 run simple_safety_node simple_safety_node
```

**Terminal 3 (optional):**

```bash
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /drive
```

---

## **7. Verification and Checklist**

| Item              | Description                                                  |
| ----------------- | ------------------------------------------------------------ |
| Topic Consistency | `/scan`, `/odom`, `/drive` are correctly connected           |
| Safety Logic      | Deceleration or stop command triggers near obstacles         |
| Topic Remapping   | Use `--ros-args -r <from>:=<to>` if topic mismatch           |
| Test Phases       | Conduct tests in order: lift test → low-speed run → full run |

---

## **8. Team Deliverables**

### **Shared Deliverables**

1. `~/f1tenth_ws/src/simple_safety_node` directory
   (including code, `package.xml`, `setup.py`)
2. Screenshots or logs of bringup and node execution
3. `/drive` topic output log (`ros2 topic echo /drive`)
4. Final PDF report (see structure below)

---

### **Report Structure (PDF Format)**

1. **Project Overview** (goal, environment, car setup)
2. **Team Members and Roles**
3. **Code Modification Summary** (`/ego_racecar/odom` → `/odom`)
4. **Build and Execution Procedure**
5. **Execution Results and Topic Snapshots**
6. **Analysis and Conclusion** (functionality, findings, improvements)

---

## **9. Evaluation Criteria**

| Category                      | Points | Evaluation Basis                            |
| ----------------------------- | ------ | ------------------------------------------- |
| Team Collaboration and Roles  | 10 pts | Teamwork, role execution, coordination      |
| Code Implementation           | 25 pts | Accuracy, structure, comments               |
| Package Configuration         | 15 pts | Correct `package.xml` and `setup.py`        |
| Vehicle Bringup and Execution | 20 pts | Successful topic verification and operation |
| Safety Logic Validation       | 20 pts | Correct `/drive` command output             |
| Report Quality                | 10 pts | Clarity, completeness, and insights         |

**Total: 100 points**

---

## **10. Summary**

* The simulator setup is **not required** for this project.
* Modify `simple_safety_node.py` to replace `/ego_racecar/odom` with `/odom`.
* Run the **bringup_launch.py** to start the F1TENTH car and sensors.
* Execute the node and verify `/drive` commands to confirm safety logic.
* Submit all deliverables as a **team report** including code, logs, and analysis.

---

📘 **Final Objective:**
The team will successfully develop and validate a **LiDAR-based safety control node** for the F1TENTH car in ROS 2 Foxy,
demonstrating a functioning **Safety Layer** in a real autonomous vehicle system.
