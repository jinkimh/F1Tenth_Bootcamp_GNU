# **Lab-05 (AV 버전): simple_safety_node on F1TENTH AV**

---

## **1. 사전 조건 (ROS 2 Foxy · 실차 환경)**

| 항목       | 내용                                         |
| -------- | ------------------------------------------ |
| OS       | Ubuntu 20.04 LTS                           |
| Python   | 3.8 (ROS 2 Foxy 기본)                        |
| ROS 2 버전 | Foxy Fitzroy                               |
| 워크스페이스   | `/f1tenth_ws`                              |
| 필수 패키지   | `sudo apt install ros-foxy-ackermann-msgs` |

### **ROS 2 환경 설정**

```bash
source /opt/ros/foxy/setup.bash
```

### **차량 구동 전 안전수칙**

* 차량을 정비 스탠드 위에 두거나, 바퀴가 지면에 닿지 않도록 설정
* E-Stop 스위치 준비, 속도 파라미터를 최소값으로 시작
* 반드시 안전한 실내 또는 전용 테스트 구역에서 수행

---

## **2. simple_safety_node 패키지 생성**

(패키지가 이미 있다면 이 단계는 건너뛰고, 코드만 수정)

```bash
cd ~/f1tenth_ws/src
ros2 pkg create --build-type ament_python simple_safety_node
```

생성 구조:

```
simple_safety_node/
  package.xml
  setup.py
  setup.cfg
  resource/simple_safety_node
  simple_safety_node/__init__.py
```

---

## **3. 노드 구현 및 코드 수정**

### (1) 코드 작성

```bash
cd ~/f1tenth_ws/src/simple_safety_node/simple_safety_node
nano simple_safety_node.py
```

아래 링크의 코드를 복사 후 붙여넣습니다.
👉 [https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py)

---

### (2) 핵심 수정 사항 – 토픽 네임스페이스 제거

차량의 `bringup`은 일반적으로 `/odom`, `/scan`, `/drive` 등 **루트 토픽**을 사용하므로
아래 코드에서 **`/ego_racecar` 접두사를 제거**해야 합니다.

**수정 전 (약 34 라인 부근):**

```python
self.sub_odom = self.create_subscription(
    Odometry, '/ego_racecar/odom', self.odom_callback, 10
)
```

**수정 후:**

```python
self.sub_odom = self.create_subscription(
    Odometry, '/odom', self.odom_callback, 10
)
```

**참고**

* 입력 토픽: `/scan`
* 출력 토픽: `/drive` (ackermann_msgs/AckermannDriveStamped)
* 필요 시 `ros2 run ... --ros-args -r <from>:=<to>` 로 remap 가능

---

### (3) Foxy 호환성 확인

```python
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
```

* `declare_parameter()` 및 `get_parameter()` API는 Foxy 버전에서 정상 지원됩니다.

---

## **4. setup.py 작성 및 수정**

```bash
cd ~/f1tenth_ws/src/simple_safety_node
nano setup.py
```

필수 항목:

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

참고 예시:
[https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/setup.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/setup.py)

---

## **5. package.xml 의존성 추가**

`~/f1tenth_ws/src/simple_safety_node/package.xml`

```xml
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

---

## **6. 빌드 및 환경설정**

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
```

**테스트 실행**

```bash
ros2 run simple_safety_node simple_safety_node
```

에러 시 점검:

* `entry_points` 오타 여부
* `package.xml` 의존성 누락
* `source /opt/ros/foxy/setup.bash` → `source install/setup.bash` 순서 확인

---

## **7. 차량 Bringup 실행**

터미널 1 (센서 및 제어 bringup):

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**정상 토픽 확인**

```bash
ros2 topic list
```

아래 토픽들이 보이면 정상입니다.

* `/scan` (sensor_msgs/LaserScan)
* `/odom` (nav_msgs/Odometry)
* `/drive` (ackermann_msgs/AckermannDriveStamped)

---

## **8. simple_safety_node 실행**

터미널 2:

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 run simple_safety_node simple_safety_node
```

터미널 3 (확인용, 선택):

```bash
# LiDAR 입력 확인
ros2 topic echo /scan

# ODOM 입력 확인
ros2 topic echo /odom

# 안전 로직 출력 확인
ros2 topic echo /drive
```

---

## **9. 체크리스트 (AV 전용)**

| 항목           | 점검 내용                                             |
| ------------ | ------------------------------------------------- |
| 토픽 이름        | bringup과 동일한 이름인지 확인 (`/scan`, `/odom`, `/drive`) |
| remap 사용     | 필요 시 `ros2 run ... --ros-args -r <from>:=<to>`    |
| 속도/브레이크 파라미터 | 초기에는 저속·단거리로 설정                                   |
| 안전 시험 순서     | **허공 시험 → 저속 주행 → 실제 주행** 순으로 단계적 진행              |

---

## **10. 제출 (개인 과제)**

1. `~/f1tenth_ws/src/simple_safety_node` 전체 디렉터리
   (코드, `package.xml`, `setup.py` 포함)
2. 차량 bringup + 노드 실행 화면 캡처 또는 로그
3. 결과 요약 PDF (변경된 토픽, 테스트 환경, /drive 출력 예시 기술)

---

## **요약**

* 시뮬레이터 단계를 **모두 생략**
* 코드 내 `/ego_racecar/odom` → `/odom` 으로 수정
* 차량에서 `bringup_launch.py` 실행 후
  **같은 방식으로 simple_safety_node 를 실행**하면 됩니다.

---
