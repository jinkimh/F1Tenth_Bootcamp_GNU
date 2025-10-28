# ** Lab-05 (Team Project) – Implementation of simple_safety_node on F1TENTH Car (ROS 2 Foxy)**

---

## **1. 프로젝트 개요**

이 프로젝트의 목적은 **F1TENTH 실차(Autonomous Vehicle)** 상에서
LiDAR 기반의 **simple_safety_node**를 구현하고 검증하는 것입니다.

팀은 ROS 2 Foxy 환경에서 실제 F1TENTH 차량을 bringup한 후,
안전 거리 기반 감속 및 정지 제어 로직이 정상적으로 동작하도록
패키지를 작성, 수정, 빌드, 실행 및 테스트합니다.

---

## **2. 팀 구성 및 역할 분담 (예시)**

| 역할                              | 담당 업무                              | 주요 산출물                              |
| ------------------------------- | ---------------------------------- | ----------------------------------- |
| **Team Leader**                 | 전체 일정 관리, 코드 통합, 보고서 최종 검토         | 실행 로그, 결과 보고서                       |
| **Developer A (ROS)**           | `simple_safety_node` 코드 작성 및 토픽 수정 | Python 코드 (`simple_safety_node.py`) |
| **Developer B (System/Launch)** | 차량 bringup 및 실행 테스트                | Launch 로그, 토픽 캡처                    |
| **Developer C (Validation)**    | `/drive` 명령 검증 및 안전 로직 분석          | 테스트 결과 요약, 그래프                      |
| **Documentation Manager**       | 결과 보고서 정리 및 제출 파일 구성               | PDF 보고서, 캡처 정리                      |

---

## **3. 개발 환경**

| 항목            | 내용                                         |
| ------------- | ------------------------------------------ |
| **OS**        | Ubuntu 20.04 LTS                           |
| **Python**    | 3.8                                        |
| **ROS 2 버전**  | Foxy Fitzroy                               |
| **Workspace** | `/f1tenth_ws`                              |
| **필수 패키지**    | `sudo apt install ros-foxy-ackermann-msgs` |

ROS 환경 설정:

```bash
source /opt/ros/foxy/setup.bash
```

### **차량 안전 수칙**

* 차량은 반드시 **정비 스탠드 위** 또는 **바퀴 미접지 상태**에서 초기 테스트
* **E-Stop 준비**, **속도 파라미터 최소값**으로 설정
* **실내/안전 구역**에서만 실험

---

## **4. 구현 절차**

### **Step 1. simple_safety_node 패키지 생성**

(이미 존재한다면 생략 가능)

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

### **Step 2. 코드 구현 및 수정**

```bash
cd ~/f1tenth_ws/src/simple_safety_node/simple_safety_node
nano simple_safety_node.py
```

기본 코드:
[https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/simple_safety_node/simple_safety_node/simple_safety_node.py)

#### **핵심 수정 사항 – 토픽 네임스페이스 제거**

기존:

```python
self.sub_odom = self.create_subscription(
    Odometry, '/ego_racecar/odom', self.odom_callback, 10
)
```

변경:

```python
self.sub_odom = self.create_subscription(
    Odometry, '/odom', self.odom_callback, 10
)
```

#### **토픽 확인**

* 입력: `/scan`
* 출력: `/drive`
* 필요 시 remap 사용 가능
  (`ros2 run ... --ros-args -r <from>:=<to>`)

#### **Foxy 호환 API**

```python
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
```

(`declare_parameter`, `get_parameter` 사용 가능)

---

### **Step 3. setup.py 수정**

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

### **Step 4. package.xml 수정**

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

### **Step 5. 빌드 및 환경설정**

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
```

빌드 확인:

```bash
ros2 run simple_safety_node simple_safety_node
```

---

## **5. 차량 Bringup 실행**

터미널 1:

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws/
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

정상 토픽 확인:

```bash
ros2 topic list
```

나타나야 할 주요 토픽:

* `/scan` (sensor_msgs/LaserScan)
* `/odom` (nav_msgs/Odometry)
* `/drive` (ackermann_msgs/AckermannDriveStamped)

---

## **6. simple_safety_node 실행 및 검증**

터미널 2:

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws/
source install/setup.bash
ros2 run simple_safety_node simple_safety_node
```

터미널 3 (확인용):

```bash
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /drive
```

---

## **7. 검증 및 점검 체크리스트**

| 항목          | 점검 내용                              |
| ----------- | ---------------------------------- |
| 토픽 이름 일치 여부 | `/scan`, `/odom`, `/drive` 가 정상인지  |
| 안전 로직 동작    | 장애물 접근 시 감속/정지 명령 발생 여부            |
| Remap 필요 시  | `--ros-args -r <from>:=<to>` 옵션 사용 |
| 시험 단계       | 허공 테스트 → 저속 주행 → 실제 주행 순으로 진행      |

---

## **8. 팀 보고서 및 제출물**

### **공동 제출물**

1. `~/f1tenth_ws/src/simple_safety_node` 전체 디렉터리
   (코드, `package.xml`, `setup.py` 포함)
2. bringup 및 노드 실행 화면 캡처
3. `/drive` 명령 출력 로그 (`ros2 topic echo /drive`)
4. 최종 PDF 보고서 (구성 아래 참조)

---

### **보고서 구성 (PDF 형식)**

1. **프로젝트 개요** (목적, 환경, 차량 정보)
2. **팀 구성 및 역할 분담**
3. **코드 수정 내용** (`/ego_racecar/odom` → `/odom` 변경 부분 명시)
4. **빌드 및 실행 절차**
5. **실행 결과 및 토픽 로그 스냅샷**
6. **분석 및 결론** (성공 동작, 개선점, 향후 계획)

---

## **9. 평가 기준**

| 항목              | 배점  | 평가 기준                        |
| --------------- | --- | ---------------------------- |
| 팀 구성 및 역할 수행    | 10점 | 역할별 책임 수행 및 협업 수준            |
| 코드 구현 및 수정      | 25점 | 수정 정확도, 구조, 주석               |
| 패키지 설정 정확성      | 15점 | `package.xml`, `setup.py` 구성 |
| 차량 bringup 및 실행 | 20점 | 토픽 확인 및 실행 성공 여부             |
| 안전 로직 동작 검증     | 20점 | `/drive` 출력의 정확성             |
| 보고서 품질          | 10점 | 문서 정리 및 결과 해석                |

총점: **100점**

---

## **10. 요약**

* 시뮬레이터 단계를 모두 제외
* `simple_safety_node.py`에서 `/ego_racecar/odom` → `/odom`으로 변경
* `bringup_launch.py` 실행 후 동일한 방식으로 노드 실행
* `/drive` 명령 발생 여부로 안전 로직 확인
* 팀 단위로 역할을 분담하여 코드, 실행, 보고서를 공동 제출

---

📘 **최종 목표:**
팀은 ROS 2 Foxy 기반 F1TENTH 차량에서 LiDAR 기반 안전 제어 노드를 완성하고,
자율주행 시스템의 **Safety Layer** 동작을 직접 검증한다.
