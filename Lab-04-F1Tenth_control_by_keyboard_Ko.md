# **Lab-04: F1TENTH Control by Keyboard (ROS 2 Foxy)**

이 실습은 **F1TENTH 실차와 시뮬레이터 모두**에서 동일하게 동작합니다.
키 입력은 `ros2-keyboard` 패키지를 통해 **SDL 1.2** 기반으로 수신되며, **포커스된 작은 입력창**에서만 이벤트가 전달됩니다.
이 노드는 `keydown` 및 `keyup` 토픽으로 `keyboard_msgs/Key` 메시지를 발행합니다.
(참고: [GitHub – cmower/ros2-keyboard][1])

---

## ✅ Step 1. `ros2-keyboard` 설치 및 빌드

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/cmower/ros2-keyboard.git
sudo apt install -y libsdl1.2-dev
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash   # Foxy 기준: ${ROS_DISTRO}=foxy
colcon build
```

> `ros2-keyboard` 노드는 `keydown`, `keyup`을 퍼블리시하며
> `allow_repeat` 등 여러 파라미터를 지원합니다.

---

## ✅ Step 2. 키보드 입력 노드 실행

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run keyboard keyboard --ros-args -p allow_repeat:=true
```

> 입력창이 **포커스 상태일 때만** 키 이벤트가 인식됩니다.

---

## ✅ Step 3. ↑ (Up Arrow) 키 코드 확인

별도 터미널에서 다음 명령을 실행합니다.

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo /keydown
```

입력창을 클릭한 뒤 **↑ 키**를 누르면 `code` 필드에 숫자가 표시됩니다.
SDL 1 계열에서는 보통 **273**으로 보고되지만, 환경에 따라 다를 수 있습니다.
(직접 `ros2 topic echo /keydown`으로 확인하는 것이 정확합니다.)

---

## ✅ Step 4. `key_teleop` 패키지 생성 및 커맨더 노드 작성

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python key_teleop
cd key_teleop/key_teleop
wget https://raw.githubusercontent.com/mina1134/AIRobotSystem/refs/heads/main/Lab-01/commander_node.py
# 또는 직접 작성
gedit commander_node.py
```

### 📄 `commander_node.py` 예시

아래 코드는 Up/Down/Left/Right/Space 키로 전진·후진·조향·정지를 제어하는 간단한 예시입니다.
F1TENTH은 `/drive` 토픽으로 `AckermannDriveStamped` 메시지를 구독합니다.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from keyboard_msgs.msg import Key
from ackermann_msgs.msg import AckermannDriveStamped
import math

# SDL1.2 기본 키코드
K_UP = 273
K_DOWN = 274
K_RIGHT = 275
K_LEFT = 276
K_SPACE = 32

class Commander(Node):
    """Keyboard teleoperation node for F1TENTH Ackermann control"""
    def __init__(self):
        super().__init__('keyboard_commander')

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Subscriber
        self.key_sub = self.create_subscription(Key, '/keydown', self.key_callback, 10)

        # 기본 파라미터
        self.speed_forward = 1.0
        self.speed_reverse = -0.5
        self.steer_angle_deg = 20.0
        self.get_logger().info("Keyboard Commander Node started. Ready for key input.")

    def key_callback(self, key_msg: Key):
        drive_msg = AckermannDriveStamped()
        cmd = drive_msg.drive
        steer_rad = math.radians(self.steer_angle_deg)

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
            cmd.steering_angle = steer_rad
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
            return

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

## ✅ Step 5. `setup.py` 및 `package.xml`

`setup.py` (엔트리포인트 확인)

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
    description='Keyboard teleop for F1TENTH Ackermann control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'commander_node = key_teleop.commander_node:main',
        ],
    },
)
```

`package.xml`

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

## ✅ Step 6. 빌드 및 실행

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 차량 또는 시뮬레이터 준비

* **실차**

  ```bash
  source /opt/ros/foxy/setup.bash
  # 차량 bringup 후 /drive 구독 드라이버(VESC)가 실행 중인지 확인
  ```

* **시뮬레이터(f1tenth_gym_ros)**
  토픽 이름과 브리지 매핑을 환경에 맞게 조정하세요.

### 실행 절차

터미널 A:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run keyboard keyboard --ros-args -p allow_repeat:=true
```

터미널 B:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run key_teleop commander_node
```

터미널 C (차량 bringup)

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

> 조이스틱 오른쪽의 **자율주행 제어 버튼을 누른 뒤** 키보드 제어를 실행합니다.

---

## ✅ Step 7. 동작 확인

```bash
# /drive 토픽으로 발행되는 제어 명령 확인
ros2 topic echo /drive

# 키 이벤트 확인 (선택)
ros2 topic echo /keydown
```

`/drive`의 `AckermannDriveStamped` 메시지 중
`speed`와 `steering_angle` 값이 실제 차량 제어에 사용됩니다.

---

## ⚙️ Troubleshooting

| 문제            | 원인                   | 해결 방법                                                          |
| ------------- | -------------------- | -------------------------------------------------------------- |
| 키 입력이 동작하지 않음 | 입력창에 포커스가 없음         | 작은 SDL 입력창을 클릭 후 키 입력                                          |
| 키 코드가 다르게 출력됨 | SDL 버전 또는 환경 차이      | 실제 `ros2 topic echo /keydown`으로 확인 후 코드 수정                     |
| 차량이 반응하지 않음   | `/drive` 구독 드라이버 미실행 | bringup 상태 및 토픽 타입 확인 (`ackermann_msgs/AckermannDriveStamped`) |

---


[1]: https://github.com/cmower/ros2-keyboard
[2]: https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_autonomous.html
[3]: https://github.com/pygame/pygame/issues/1128

---

원하신다면 위 내용을 **TTS 강의 스크립트 버전**으로도 만들어드릴까요? (각 Step별 설명 포함)
