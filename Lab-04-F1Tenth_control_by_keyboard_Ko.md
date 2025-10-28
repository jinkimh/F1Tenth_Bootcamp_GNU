# **Lab-04: F1TENTH Control by Keyboard (ROS 2 Foxy)**

아래 절차는 **F1TENTH 실차/시뮬레이터 공통**으로 동작합니다. 키 입력은 `ros2-keyboard` 패키지가 **SDL 1.2**로 수신하며, 포커스된 작은 입력 창에서만 이벤트가 전달됩니다. ([GitHub][1])
해당 노드는 `keydown`/`keyup` 토픽에 `keyboard_msgs/Key` 메시지를 발행합니다. ([GitHub][1])

---

## Step 1. `ros2-keyboard` 다운로드 & 빌드

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/cmower/ros2-keyboard.git
sudo apt install -y libsdl1.2-dev
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash   # Foxy라면 ${ROS_DISTRO}=foxy
colcon build
```

> 패키지 노드는 `keydown`, `keyup`을 퍼블리시하고, `allow_repeat` 등의 파라미터를 지원합니다. ([GitHub][1])

---

## Step 2. 키보드 입력 노드 실행

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run keyboard keyboard --ros-args -p allow_repeat:=true
```

> 이 노드는 **포커스된 작은 윈도우**에서만 키를 받습니다(포커스 없으면 이벤트 미수신). ([GitHub][1])

---

## Step 3. ↑(Up Arrow) 키 코드 확인

별도 터미널에서:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo /keydown
```

작은 입력창을 클릭한 뒤 **↑ 키**를 누르면 터미널에 `code` 필드가 표시됩니다.
`ros2-keyboard`는 SDL 1.2를 사용하며, **SDL 1 계열에서 Up Arrow는 보통 `273`**으로 보고됩니다(환경에 따라 다를 수 있음). ([GitHub][1])

---

## Step 4. `key_teleop` 패키지 생성 & 커맨더 노드 추가

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python key_teleop
cd key_teleop/key_teleop
# 예시 커맨더 노드 내려받기(원문 예시 경로 준수)
wget https://raw.githubusercontent.com/mina1134/AIRobotSystem/refs/heads/main/Lab-01/commander_node.py
# 또는 직접 편집
gedit commander_node.py
```

### `commander_node.py` (정리된 예시)

아래는 **Up Arrow(273)**에 대해 전진, 좌/우 방향키로 조향, Space로 정지하는 간단한 예시입니다.
F1TENTH의 주행 명령은 `/drive` 토픽의 `AckermannDriveStamped` 메시지를 사용합니다. ([f1tenth.readthedocs.io][2])

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from keyboard_msgs.msg import Key
from ackermann_msgs.msg import AckermannDriveStamped
import math

# SDL1.2 기본 키코드 (ros2-keyboard 기준)
K_UP = 273
K_DOWN = 274
K_RIGHT = 275
K_LEFT = 276
K_SPACE = 32

class Commander(Node):
    """Keyboard teleoperation node for F1TENTH Ackermann control"""
    def __init__(self):
        super().__init__('keyboard_commander')

        # Publisher: /drive → AckermannDriveStamped 메시지 발행
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Subscriber: /keydown → 키 입력 수신
        self.key_sub = self.create_subscription(Key, '/keydown', self.key_callback, 10)

        # 기본 파라미터
        self.speed_forward = 1.0       # 전진 속도 (m/s)
        self.speed_reverse = -0.5      # 후진 속도 (m/s)
        self.steer_angle_deg = 20.0    # 조향 각도 (degrees)
        self.get_logger().info("Keyboard Commander Node started. Ready to receive key input.")

    def key_callback(self, key_msg: Key):
        """키 입력 콜백 함수: 키코드에 따라 /drive 명령 발행"""
        drive_msg = AckermannDriveStamped()
        cmd = drive_msg.drive

        # 각도 변환
        steer_rad = math.radians(self.steer_angle_deg)

        # 키코드별 동작 매핑
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
            return  # 다른 키는 무시

        # 메시지 발행
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

## Step 5. `setup.py` 구성

```bash
gedit ~/ros2_ws/src/key_teleop/setup.py
```

아래처럼 **깨끗한 엔트리포인트**를 등록하세요(원문 스니펫의 누락·오타 정정).

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
    description='Keyboard teleop to publish AckermannDriveStamped to /drive',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'commander_node = key_teleop.commander_node:main',
        ],
    },
)
```

### `package.xml`(필요 시)

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

## Step 6. 빌드 & 실행

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 차량/시뮬레이터 구동(별도 터미널)

* 실차:

  ```bash
  source /opt/ros/foxy/setup.bash
  # 차량 bringup 후 /drive 구독 드라이버(VESC)가 떠 있어야 함
  ```
* 시뮬레이터(예: f1tenth_gym_ros) 사용 시 브릿지/토픽 정합 필요.

### 키보드 입력 노드 + 커맨더 노드 실행

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

---

## 동작 점검

```bash
# 발행되는 주행 명령 확인
ros2 topic echo /drive
# 키 이벤트 확인(선택)
ros2 topic echo /keydown
```

F1TENTH은 `/drive`의 `AckermannDriveStamped` 메시지의 `speed`와 `steering_angle`을 사용합니다. ([f1tenth.readthedocs.io][2])

---

## 트러블슈팅

* **키 입력이 안 잡힘**: 입력 창에 포커스를 두세요. 창이 비활성화면 이벤트 미수신. ([GitHub][1])
* **↑ 키 코드가 273이 아님**: SDL 버전/환경에 따라 상수값이 달라질 수 있습니다(SD L1=273, SDL2는 다른 값). 실제 `ros2 topic echo /keydown`로 코드 확인 후 매핑하세요. ([GitHub][3])
* **차량이 반응 안 함**: `/drive`를 실제로 구독하는 드라이버(예: VESC)가 떠 있는지, 메시지 타입/토픽명이 일치하는지 확인하세요. ([f1tenth.readthedocs.io][2])

---

필요하면 위 예시 코드를 **개행·주석 포함 완전본**으로도 준비해 드릴게요.

[1]: https://github.com/cmower/ros2-keyboard "GitHub - cmower/ros2-keyboard: Keyboard driver for ROS 2."
[2]: https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_autonomous.html?utm_source=chatgpt.com "Autonomous Control — RoboRacer- Build latest documentation"
[3]: https://github.com/pygame/pygame/issues/1128?utm_source=chatgpt.com "SDL2: constant keyboard IDs different. · Issue #1128"
