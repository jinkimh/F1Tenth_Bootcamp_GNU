## Lab-05: simple_safety_node 시뮬레이터 및 F1TENTH 구현**

---

### 과제 개요

이 개인 실습의 목적은 **ROS 2 Foxy** 환경에서 **F1TENTH 시뮬레이터**와 **simple_safety_node**를 직접 구현하고 실행해 보는 것입니다.
LiDAR 센서로부터 수신한 `/scan` 데이터를 기반으로 **안전 거리 판단 및 감속·정지 명령**을 수행하는 노드를 작성함으로써, 자율주행 차량의 기본적인 **안전 제어 로직**을 이해하는 것이 목표입니다.

---

### 과제 요구사항

| 구분        | 내용                                          |
| --------- | ------------------------------------------- |
| **필수 구현** | `simple_safety_node` 패키지 생성, 코드 작성, 빌드 및 실행 |
| **환경 요건** | ROS 2 Foxy + F1TENTH 시뮬레이터 연동               |
| **기능**    | `/scan` 입력 → 안전 거리 판단 → `/drive` 출력         |
| **결과 검증** | `/drive` 감속·정지 명령 확인 및 `rqt_graph` 구조 검증    |

---

### 평가 항목 (총점 100점)

| 항목                            | 배점  | 평가 기준                      |
| ----------------------------- | --- | -------------------------- |
| 시뮬레이터 실행 정상 여부                | 20점 | `/scan`, `/drive` 토픽 정상 출력 |
| 코드 작성                         | 25점 | 코드 구조, 주석, Foxy API 호환성    |
| `package.xml`, `setup.py` 정확성 | 15점 | 의존성 및 entry_points 설정 정확성  |
| 빌드 및 실행 성공                    | 20점 | `colcon build` 후 노드 정상 실행  |
| 안전 로직 동작                      | 20점 | 감속/정지 명령 정상 출력 및 로그 분석     |

---

### 제출 항목

1. `/sim_ws/src/simple_safety_node` 전체 디렉터리 (코드 및 `package.xml` 포함)
2. 실행 화면 캡처 또는 로그 (시뮬레이터 + 노드 실행 화면)
3. 간단한 실행 설명서 또는 결과 요약 PDF

---

## 환경 (ROS 2 Foxy 전용)

| 항목           | 내용                                             |
| ------------ | ---------------------------------------------- |
| **운영체제**     | Ubuntu 20.04 LTS                               |
| **Python**   | Python 3.8 (Foxy 기본)                           |
| **ROS 2 버전** | Foxy Fitzroy                                   |
| **워크스페이스**   | `/sim_ws` (구조: `/sim_ws/src` → `colcon build`) |

설정 명령:

```bash
$ source /opt/ros/foxy/setup.bash
```

필수 의존성 설치:

```bash
$ sudo apt install ros-foxy-ackermann-msgs
```

---

## Step 1 – F1TENTH 시뮬레이터 실행 확인

참고 문서:
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/1-Tutorial_F1Tenth_Simulator(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/1-Tutorial_F1Tenth_Simulator%28English%29.md)

```bash
# (새 터미널)
$ source /opt/ros/foxy/setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Foxy 적합성 점검**

* `gym_bridge_launch.py` 실행 시 에러 없이 RViz 또는 Gazebo가 실행되어야 함.
* `ros2 topic list` 출력에 다음 토픽들이 포함되어야 함:

  * `/scan` (sensor_msgs/LaserScan)
  * `/drive` (ackermann_msgs/AckermannDriveStamped)
  * `/ego_racecar/odom` (버전에 따라 네임스페이스가 다를 수 있음)
* 에러 발생 시 확인할 사항:

  * 해당 리포지토리가 Foxy 버전으로 빌드되었는지
  * `source ~/f1tenth_ws/install/setup.bash` 등 시뮬레이터 워크스페이스를 올바르게 소싱했는지

---

## Step 2 – simple_safety_node 패키지 생성

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ cd src
/sim_ws/src$ ros2 pkg create --build-type ament_python simple_safety_node
```

생성된 디렉터리 구조:

```
simple_safety_node/
  package.xml
  setup.py
  setup.cfg
  resource/simple_safety_node
  simple_safety_node/__init__.py
```

`ament_python` 빌드타입은 Foxy에서 공식 지원됩니다.

---

## Step 3 – 노드 구현

```bash
/sim_ws/src$ cd simple_safety_node/simple_safety_node
/sim_ws/src/simple_safety_node/simple_safety_node$ nano simple_safety_node.py
```

아래 링크의 코드를 복사하여 붙여넣습니다:
[https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py)

**Foxy 호환성 점검**

```python
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
```

* 입력 토픽: `/scan`
* 출력 토픽: `/drive`
  (필요 시 `/ego_racecar/scan`으로 remap 가능)
* `declare_parameter()` 및 `get_parameter()` API는 Foxy에서 정상 동작합니다.

---

## Step 4 – setup.py 작성

```bash
/sim_ws/src/simple_safety_node$ cd ..
/sim_ws/src/simple_safety_node$ nano setup.py
```

---

## Step 5 – setup.py 수정

참고 링크:
[https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/setup.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/setup.py)

필수 항목:

```python
entry_points={
    'console_scripts': [
        'simple_safety_node = simple_safety_node.simple_safety_node:main',
    ],
},
```

다음 항목도 포함해야 합니다:

```python
install_requires=['setuptools']
zip_safe=True
packages=['simple_safety_node']
```

---

## package.xml 의존성 추가

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

`ackermann_msgs` 패키지가 없다면 설치:

```bash
$ sudo apt install ros-foxy-ackermann-msgs
```

---

## Step 6 – 빌드 및 환경 설정

```bash
/sim_ws/src/simple_safety_node$ cd /sim_ws
/sim_ws$ colcon build
/sim_ws$ source install/setup.bash
```

**빌드 후 테스트:**

```bash
$ ros2 run simple_safety_node simple_safety_node
```

**문제 발생 시 확인:**

* `entry_points` 오타 여부
* `package.xml` 의존성 누락 여부
* 소싱 순서:
  `source /opt/ros/foxy/setup.bash` → `source /sim_ws/install/setup.bash`

---

## 실행 절차 (ROS 2 Foxy)

### 1단계 – 시뮬레이터 실행 (터미널 1)

```bash
$ source /opt/ros/foxy/setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

RViz 또는 Gazebo 실행 후 `/scan`, `/drive` 토픽이 표시되면 정상입니다.
이 창은 닫지 말고 그대로 둡니다.

### 2단계 – simple_safety_node 실행 (터미널 2)

```bash
$ source /opt/ros/foxy/setup.bash
$ cd ~/sim_ws
$ source install/setup.bash
$ ros2 run simple_safety_node simple_safety_node
```

이 노드는 `/scan`을 구독하고, 안전 거리 또는 TTC 기준에 따라 `/drive` 명령을 발행합니다.

**동작 확인**

```bash
$ ros2 topic echo /drive
```

→ 감속 또는 정지 명령이 출력되면 정상 동작입니다.

---

### 정상 동작 확인 체크리스트

| 항목          | 명령어                      | 기대 결과                                           |
| ----------- | ------------------------ | ----------------------------------------------- |
| `/scan` 수신  | `ros2 topic echo /scan`  | 거리 데이터가 지속적으로 출력                                |
| `/drive` 발행 | `ros2 topic echo /drive` | AckermannDriveStamped 메시지 출력                    |
| 그래프 구조 확인   | `rqt_graph`              | `/scan` → `simple_safety_node` → `/drive` 연결 표시 |

---

### 요약

본 Lab-05는 ROS 2 Foxy 환경에서 F1TENTH 시뮬레이터와 연동하여 **LiDAR 기반 안전 제어 노드**를 구현하는 실습입니다.
`simple_safety_node`를 통해 자율주행 차량의 안전 판단 로직, 토픽 통신 구조, 그리고 실제 제동 명령 발생 과정을 체험할 수 있습니다.
