# 🧪 Odometry Tuning

To improve the driving accuracy of the F1Tenth autonomous vehicle, odometry-related parameters must be tuned properly.
In this practice, you will learn how to adjust **ERPM gain**, **steering angle offset**, and **steering angle gain** to improve straight-line driving and turning accuracy.

> ✅ Source: [Mushr Odometry Tuning Guide (ROS1)](https://mushr.io/tutorials/tuning/)

---

## ⚙️ Config File Location

```bash
$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
```

### Key Parameters

```yaml
/**:
  ros__parameters:
    speed_to_erpm_gain: 4614.0
    speed_to_erpm_offset: 0.0
    steering_angle_to_servo_gain: -1.2135
    steering_angle_to_servo_offset: 0.5304
```

---

## ⚠️ Required Procedure After Editing `vesc.yaml`

Whenever you modify the `vesc.yaml` file, you **must** follow these steps:

### 🔄 Reapply After Editing

```bash
cd ~/f1tenth_ws

# 1. Rebuild
colcon build

# 2. Source ROS2 environment
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 3. Relaunch bringup
ros2 launch f1tenth_stack bringup_launch.py
```

> ⚠️ Simply restarting bringup without running `colcon build` will **not apply changes**.

---

## 🛞 1. Steering Angle Offset (`steering_angle_to_servo_offset`)

### 🎯 Purpose

Correct the neutral steering position so the vehicle drives straight.

### 🛠 Tuning Steps

1. Run bringup:

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

2. Drive straight and check deviation.

3. Adjust offset based on deviation:

   * Drifts left → Increase offset
   * Drifts right → Decrease offset

4. Apply changes:

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

5. Repeat until optimal value is found.

> 📌 Typical range: `0.4 ~ 0.6`

---

## ⚡ 2. ERPM Gain (`speed_to_erpm_gain`)

### 🎯 Purpose

Reduce error between commanded speed and actual traveled distance.

### 🛠 Tuning Steps

1. Prepare a measuring tool (≥ 3m).
2. Align vehicle at zero position and run bringup:

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

3. Drive and check odometry:

```bash
ros2 topic echo --no-arr /odom
```

4. Adjust gain based on error:

   * Reported distance < actual → Increase gain
   * Reported distance > actual → Decrease gain

5. Apply changes:

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

> 📌 Recommended step size: `500`
> 📌 Typical range: `2000 ~ 5000`

---

### ❗ When `/odom` Values Are **Negative**

In some vehicles, `/odom` outputs negative x values.
This means velocity calculation is reversed. Edit the following code:

### 🔧 File Location

```bash
$HOME/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp
```

### ✏️ Edit Near Line 102

**Before:**

```cpp
double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
```

**After:**

```cpp
double current_speed = -(-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
```

✅ Wrap with `-()` to flip the sign.

Rebuild:

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

---

## 🌀 3. Steering Angle Gain (`steering_angle_to_servo_gain`)

### 🎯 Purpose

Tune steering commands to match the expected turning radius.

### 🛠 Tuning Steps

1. Align vehicle on measuring tape.

2. Drive with max steering command.

3. Measure turning radius (target: `1.722m`).

4. Adjust gain based on error:

   * Too wide → Decrease gain
   * Too tight → Increase gain

5. Apply changes:

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

> 📌 Typical range: `1.1 ~ 1.3`

---

## 📑 Summary

| Item            | Parameter                            | Typical Range | Required Action After Edit   |
| --------------- | ------------------------------------ | ------------- | ---------------------------- |
| Steering Offset | `steering_angle_to_servo_offset`     | 0.4 \~ 0.6    | ✅ Rebuild + Relaunch bringup |
| ERPM Gain       | `speed_to_erpm_gain`                 | 2000 \~ 5000  | ✅ Rebuild + Relaunch bringup |
| Steering Gain   | `steering_angle_to_servo_gain`       | 1.1 \~ 1.3    | ✅ Rebuild + Relaunch bringup |
| Odom Correction | `vesc_to_odom.cpp` speed calculation | -             | ✅ Rebuild                    |

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
