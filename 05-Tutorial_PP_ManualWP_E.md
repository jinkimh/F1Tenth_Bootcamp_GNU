
# Running Manually Generated Waypoints (WP) with Pure Pursuit in the Simulator

This document explains how to run the Pure Pursuit algorithm inside the F1TENTH simulator using user-generated `(x, y)` waypoints, along with required code modifications.

---

## 1. Modify the Pure Pursuit Node

**File Location:**
`/sim_ws/src/f1tenth-software-stack/pure_pursuit/scripts/pure_pursuit_node.py`

---

### 🔄 Differences Compared to Tutorial-2

| Item           | Manual Waypoint Method | Tutorial-2 Method                                       |
| -------------- | ---------------------- | ------------------------------------------------------- |
| Format         | Contains only `(x, y)` | Includes multiple columns: `(x, y, speed, steering...)` |
| Purpose        | Simple path following  | Precise, dynamic path control                           |
| Data Structure | Simple 2-column CSV    | Multi-column CSV                                        |

---

### 🗂 Example of Manual Waypoint CSV

```csv
x, y
1.0, 2.0
2.0, 3.5
3.2, 4.8
```

* Columns: `x`, `y` (with header)
* Comma-separated

---

## 2. Code Modification Steps

### ✅ 1) Set Map Name (Line 27)

```python
self.map_name = 'levine_2nd'  # Replace with your map name
```

---

### ✅ 2) Modify CSV Loading Logic (Lines 44–45)

**Before:**

```python
csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=';', skiprows=0)
self.waypoints = csv_data[:, 1:3]
```

**After:**

```python
csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=',', skiprows=1)
self.waypoints = csv_data[:, :]  # Use all columns (x, y)
```

**Summary of Changes:**

* Delimiter changed from `';'` → `','`
* Skip header row: `skiprows=0` → `1`
* Use full columns: `[:, 1:3]` → `[:, :]`

---

### ✅ 3) Modify Speed Control Logic (Line 96)

**Before:**

```python
self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * self.ref_speed[self.closest_index]
```

**After:**

```python
self.drive_msg.drive.speed = 2.0  # Set constant speed (e.g., 2.0 m/s)
```

---

### ✅ 4) Disable Speed-Related Code (Line 49)

**Original Code:**

```python
# self.ref_speed = csv_data[:, 5] * 0.6
self.ref_speed = csv_data[:, 5]
```

**Updated Version:**

```python
# self.ref_speed = csv_data[:, 5] * 0.6
# self.ref_speed = csv_data[:, 5]
```

* Comment out `ref_speed` entirely to avoid referencing nonexistent columns.

---

## 3. Running Pure Pursuit

Run the following steps inside the F1TENTH simulator environment.

---

### 🖥️ (1) Launch the Simulator Bridge

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

---

### 🚗 (2) Run the Pure Pursuit Node

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 run pure_pursuit pure_pursuit_node.py
```

* The node will follow waypoints stored in:
  `/csv_data/<map_name>.csv`

---

## 4. Final Checklist ✅

| Item               | What to Verify                                                                 |
| ------------------ | ------------------------------------------------------------------------------ |
| **CSV Path**       | Located at `~/sim_ws/src/f1tenth-software-stack/csv_data/<map_name>.csv`       |
| **CSV Format**     | Two columns `(x, y)`, comma-separated, header included                         |
| **Code Location**  | `/sim_ws/src/f1tenth-software-stack/pure_pursuit/scripts/pure_pursuit_node.py` |
| **Speed Setting**  | Constant speed set to `2.0` m/s or adjusted as needed                          |
| **Node Execution** | Use `ros2 run pure_pursuit pure_pursuit_node.py`                               |

---

## 📄 Copyright

```
© 2025 Jin Kim, Gyeongsang National University

This document was created for experimental and educational use  
on the F1TENTH autonomous driving platform.  
Unauthorized reproduction or commercial use is prohibited.
```
