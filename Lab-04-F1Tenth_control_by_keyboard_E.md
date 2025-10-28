# **Lab-04 (Team Project) : F1TENTH Control by Keyboard (ROS 2 Foxy)**

This lab demonstrates how to **control a F1TENTH vehicle (real or simulated)** using a keyboard in the ROS 2 Foxy environment.
Keyboard input is handled by the `ros2-keyboard` package, which uses **SDL 1.2**.
It receives events **only from a focused window** and publishes messages of type `keyboard_msgs/Key` to `/keydown` and `/keyup` topics.
([GitHub – ros2-keyboard][1])

---

## **Step 1 – Download and Build `ros2-keyboard`**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/cmower/ros2-keyboard.git
sudo apt install -y libsdl1.2-dev
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash   # For Foxy: ${ROS_DISTRO}=foxy
colcon build
```

> The package publishes to `/keydown` and `/keyup` and supports parameters such as `allow_repeat`. ([GitHub][1])

---

## **Step 2 – Run the Keyboard Node**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run keyboard keyboard --ros-args -p allow_repeat:=true
```

> The node only receives input when its **small SDL window** is focused.
> If the window loses focus, no key events are captured. ([GitHub][1])

---

## **Step 3 – Check the Up Arrow Key Code**

In another terminal:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo /keydown
```

Click the input window, press the **↑ (Up Arrow)** key,
and observe the `code` field in the terminal output.

`ros2-keyboard` (SDL 1.2) typically reports **Up Arrow = 273**,
but it may vary depending on your system. ([GitHub][1])

---

## **Step 4 – Create `key_teleop` Package and Add Commander Node**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python key_teleop
cd key_teleop/key_teleop
# Option A: Download example file
wget https://raw.githubusercontent.com/mina1134/AIRobotSystem/refs/heads/main/Lab-01/commander_node.py
# Option B: Create manually
gedit commander_node.py
```

### **`commander_node.py` (Complete Example)**

This node publishes Ackermann drive commands to `/drive`
based on keyboard inputs – forward, reverse, left/right steering, and stop.
(F1TENTH vehicles use `/drive` → `AckermannDriveStamped` messages.) ([F1TENTH Docs][2])

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from keyboard_msgs.msg import Key
from ackermann_msgs.msg import AckermannDriveStamped
import math

# SDL 1.2 keycodes
K_UP = 273
K_DOWN = 274
K_RIGHT = 275
K_LEFT = 276
K_SPACE = 32

class Commander(Node):
    """Keyboard teleoperation node for F1TENTH Ackermann control"""
    def __init__(self):
        super().__init__('keyboard_commander')

        # Publisher: /drive → AckermannDriveStamped
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Subscriber: /keydown → Key input
        self.key_sub = self.create_subscription(Key, '/keydown', self.key_callback, 10)

        # Default parameters
        self.speed_forward = 1.0     # m/s
        self.speed_reverse = -0.5
        self.steer_angle_deg = 20.0  # degrees

        self.get_logger().info("Keyboard Commander Node started. Waiting for key input...")

    def key_callback(self, key_msg: Key):
        """Callback for key input"""
        drive_msg = AckermannDriveStamped()
        cmd = drive_msg.drive
        steer_rad = math.radians(self.steer_angle_deg)

        # Key-to-command mapping
        if key_msg.code == K_UP:
            cmd.speed = self.speed_forward
            cmd.steering_angle = 0.0
            self.get_logger().info("Forward")
        elif key_msg.code == K_DOWN:
            cmd.speed = self.speed_reverse
            cmd.steering_angle = 0.0
            self.get_logger().info("Reverse")
        elif key_msg.code == K_LEFT:
            cmd.speed = self.speed_forward * 0.6
            cmd.steering_angle = +steer_rad
            self.get_logger().info("Turn Left")
        elif key_msg.code == K_RIGHT:
            cmd.speed = self.speed_forward * 0.6
            cmd.steering_angle = -steer_rad
            self.get_logger().info("Turn Right")
        elif key_msg.code == K_SPACE:
            cmd.speed = 0.0
            cmd.steering_angle = 0.0
            self.get_logger().info("Stop")
        else:
            return  # Ignore all other keys

        # Publish drive command
        self.drive_pub.publish(drive_msg)
        self.get_logger().info(
            f"→ Published /drive: speed={cmd.speed:.2f} m/s, steer={cmd.steering_angle:.2f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## **Step 5 – Configure `setup.py`**

```bash
gedit ~/ros2_ws/src/key_teleop/setup.py
```

Clean configuration example (corrected and simplified):

```python
from setuptools import find_packages, setup

package_name = 'key_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mina',
    maintainer_email='k1b2c508600@gmail.com',
    description='Keyboard teleop to publish AckermannDriveStamped messages to /drive',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'commander_node = key_teleop.commander_node:main',
        ],
    },
)
```

### **`package.xml` (optional)**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>key_teleop</name>
  <version>0.0.1</version>
  <description>Keyboard teleop for F1TENTH Ackermann drive</description>
  <maintainer email="k1b2c508600@gmail.com">mina</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>ackermann_msgs</exec_depend>
  <exec_depend>keyboard_msgs</exec_depend>
</package>
```

---

## **Step 6 – Build and Run**

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### **Vehicle / Simulator Bringup (separate terminal)**

* **Real Car:**

  ```bash
  source /opt/ros/foxy/setup.bash
  # Make sure the vehicle bringup and /drive subscriber (e.g., VESC driver) are running.
  ```

* **Simulator (e.g., f1tenth_gym_ros):**
  Adjust or remap topics accordingly.

### **Run Keyboard Input + Commander Nodes**

**Terminal A:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run keyboard keyboard --ros-args -p allow_repeat:=true
```

**Terminal B:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run key_teleop commander_node
```

---

## **Verification**

```bash
# Check published drive messages
ros2 topic echo /drive

# Optional: view key events
ros2 topic echo /keydown
```

F1TENTH uses the `/drive` topic of type `AckermannDriveStamped`,
where `drive.speed` and `drive.steering_angle` control vehicle motion. ([F1TENTH Docs][2])

---

## **Troubleshooting**

* **No key input:**
  Ensure the SDL window is focused; unfocused windows receive no key events. ([GitHub][1])

* **Up Arrow code ≠ 273:**
  Keycodes differ between SDL 1 and SDL 2.
  Run `ros2 topic echo /keydown` to find actual values for your environment. ([GitHub][3])

* **No vehicle motion:**
  Verify that a `/drive` subscriber (e.g., VESC driver) is active and the message types match. ([F1TENTH Docs][2])

---

[1]: https://github.com/cmower/ros2-keyboard "GitHub – cmower/ros2-keyboard: Keyboard driver for ROS 2."
[2]: https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_autonomous.html?utm_source=chatgpt.com "Autonomous Control — RoboRacer Docs"
[3]: https://github.com/pygame/pygame/issues/1128?utm_source=chatgpt.com "SDL2 key constants differ – Pygame Issue #1128"
