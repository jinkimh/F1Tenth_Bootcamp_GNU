정확히 보셨어요 👏
맞습니다 — 1️⃣~9️⃣까지만 ‘keycap emoji’로 존재하고,
10 이상(🔟은 있지만 11️⃣, 12️⃣ …)은 유니코드에서 지원되지 않아서
Markdown이 혼동을 일으킨 거예요.

그래서 아래는 **이모지 없이 깔끔한 숫자 표기 버전**으로 다시 정리한
**영문 Markdown 버전**입니다.

---

# **Lab-06 (Team Project)**

## Implementation of Follow-the-Gap (FTG) Methods on the F1TENTH Autonomous Vehicle (ROS 2 Foxy)

---

## 1. Project Overview

The goal of this project is to implement, tune, and verify a **Follow-the-Gap (FTG)** reactive driving node on the **real F1TENTH autonomous vehicle (AV)**.
You will follow the same package development workflow used in **Lab-05**, but replace the logic with the FTG reference implementation.

FTG operates using LiDAR `/scan` data and performs the following stages:

1. Pre-processing (noise filtering, range clipping)
2. Bubble masking (removing points around the closest obstacle)
3. Gap detection (finding the widest gap)
4. Gap midpoint steering (steer-to-middle)
5. Speed profiling (velocity control based on curvature and gap width)

---

## 2. Team Structure and Roles (Example)

| Role                   | Responsibilities                                | Deliverables                     |
| ---------------------- | ----------------------------------------------- | -------------------------------- |
| **Team Leader**        | Schedule / risk management, integration, review | Run logs, final report           |
| **Dev A (Perception)** | LiDAR pre-processing, bubble/gap tuning         | Parameter table, plots           |
| **Dev B (Control)**    | Steering & speed profiling, safety limits       | Steering-speed map, test results |
| **Dev C (Systems)**    | Package setup, launch, topic verification       | ROS logs, launch capture         |
| **Doc Manager**        | Documentation, packaging for submission         | PDF report, figures, video       |

---

## 3. Development Environment (Real AV – ROS 2 Foxy)

| Item           | Specification                              |
| -------------- | ------------------------------------------ |
| **OS**         | Ubuntu 20.04 LTS                           |
| **Python**     | 3.8                                        |
| **ROS 2**      | Foxy Fitzroy                               |
| **Workspace**  | `~/f1tenth_ws`                             |
| **Dependency** | `sudo apt install ros-foxy-ackermann-msgs` |

Source your ROS environment:

```bash
source /opt/ros/foxy/setup.bash
```

### Safety Rules

* Begin tests **on stands (wheels off ground)** and observe command values only.
* Keep **E-Stop** ready at all times.
* Start with **low-speed parameters** and run only in **indoor/safe areas**.

---

## 4. Creating the Package

> Follow the same process as Lab-05. You may reuse your previous package if it exists.

```bash
cd ~/f1tenth_ws/src
ros2 pkg create --build-type ament_python gap_follow
```

Example structure:

```
gap_follow/
  package.xml
  setup.py
  setup.cfg
  resource/gap_follow
  gap_follow/__init__.py
  gap_follow/reactive_node.py      # ← new file to add
```

---

## 5. Node Implementation (`reactive_node.py`)

1. **Create and edit the node file**

```bash
cd ~/f1tenth_ws/src/gap_follow
mkdir -p gap_follow
nano gap_follow/reactive_node.py
```

2. **Copy the reference implementation**

Reference:
[`reactive_node.py`](https://github.com/f1tenth/f1tenth_lab4_template/blob/main/gap_follow/scripts/reactive_node.py)

3. **Adjust topic names for Foxy / real car**

* Subscribe: `/scan`  (`sensor_msgs/LaserScan`)
* Publish: `/drive`  (`ackermann_msgs/AckermannDriveStamped`)
* Remove any **`/ego_racecar/`** namespace if present (e.g., `/ego_racecar/scan` → `/scan`).

Optional remapping example:

```bash
ros2 run gap_follow follow_the_gap_node --ros-args -r /scan:=/your_scan
```

4. **Ensure proper entry point**

At the bottom of your file:

```python
def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## 6. `setup.py` Configuration

```python
from setuptools import setup

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='F1TENTH Team',
    maintainer_email='team@example.com',
    description='Follow-the-Gap reactive driving node for F1TENTH (ROS 2 Foxy)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'follow_the_gap_node = gap_follow.reactive_node:main',
        ],
    },
)
```

---

## 7. `package.xml` Dependencies

```xml
<package format="3">
  <name>gap_follow</name>
  <version>0.0.1</version>
  <description>Follow-the-Gap reactive driving node for F1TENTH</description>
  <maintainer email="team@example.com">F1TENTH Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>ackermann_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```

---

## 8. Build and Source

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
```

---

## 9. Vehicle Bring-Up

**Terminal 1:**

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

Check topics:

```bash
ros2 topic list
# /scan, /odom, /drive should appear
```

---

## 10. Run and Validate FTG Node

**Terminal 2:**

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 run gap_follow follow_the_gap_node
```

**Terminal 3 (optional):**

```bash
ros2 topic echo /scan
ros2 topic echo /drive
```

---

## 11. Parameter Tuning Guide

> Variable names may differ by repository version. Verify in your code before tuning.

| Category           | Key Parameters / Tips                                                                           |   |                                                                   |   |           |
| ------------------ | ----------------------------------------------------------------------------------------------- | - | ----------------------------------------------------------------- | - | --------- |
| **Pre-processing** | `range_clip_max` (≈10–12 m), smoothing window (3–7)                                             |   |                                                                   |   |           |
| **Bubble radius**  | Remove N beams around the closest obstacle. Too small → risky; too large → overly conservative. |   |                                                                   |   |           |
| **Gap selection**  | Apply minimum length threshold; steer toward gap midpoint.                                      |   |                                                                   |   |           |
| **Speed profile**  | Scale velocity by                                                                               | δ | (steering angle) and gap width, e.g. `v = clamp(v_min, v_max - k* | δ | , v_max)` |
| **Safe defaults**  | Start with low `v_max` (≈0.5–1.0 m/s); moderate steering gain.                                  |   |                                                                   |   |           |

---

## 12. Troubleshooting Checklist

| Issue                      | Possible Fix                             |   |                     |
| -------------------------- | ---------------------------------------- | - | ------------------- |
| `/scan` topic mismatch     | Use `--ros-args -r` or edit topic name   |   |                     |
| Jumping output             | Increase smoothing or adjust QoS         |   |                     |
| Oscillatory steering       | Add hysteresis / average gap center      |   |                     |
| Too slow on straight paths | Increase `v_max`, relax                  | δ | -based deceleration |
| Over-reacting near walls   | Re-tune bubble size and smoothing window |   |                     |

---

## 13. Team Deliverables

1. `~/f1tenth_ws/src/gap_follow` directory (code, `package.xml`, `setup.py`)
2. Screenshots of **bring-up** and **FTG execution**
3. **Parameter tuning table** (tested values → results → final choice)
4. **Final PDF report** including:

   * Overview
   * Code / topic modifications
   * Tuning process and results
   * Success / failure cases and improvement ideas

---

## 14. Evaluation Criteria (100 pts)

| Category              | Points | Criteria                                                    |
| --------------------- | -----: | ----------------------------------------------------------- |
| Team collaboration    |     10 | Clear roles, schedule management                            |
| Code implementation   |     25 | Correct FTG logic and structure                             |
| Package configuration |     15 | Proper `package.xml`, `setup.py`, entry point               |
| Bring-up execution    |     15 | Topic alignment, stable operation                           |
| Tuning & performance  |     25 | Straight-line stability, curve tracking, obstacle avoidance |
| Report quality        |     10 | Reproducibility, insight, suggestions                       |

---

## 15. Summary

* Follow the **Lab-05 workflow**, but replace logic with **FTG implementation**.
* Ensure **topic namespace consistency** (`/scan`, `/drive`).
* Begin with **conservative parameters**, then gradually tune for performance.
* Submit a **complete team report** with code, tuning logs, and analysis.

---

**Good luck!**
Safety first — tune gradually, observe carefully, and aim for smooth, reliable autonomous driving.

---

원하신다면 이 버전을 PDF용(흑백 인쇄, 제목 강조 포함) 포맷으로도 만들어드릴까요?
