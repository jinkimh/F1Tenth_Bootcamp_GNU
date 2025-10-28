# ** Lab-06 (Team Project) ** 
## : Implementation of `follow_the_gap` on the F1TENTH Car (ROS 2 Foxy) 

---

## 1) 프로젝트 개요

본 프로젝트의 목표는 실제 **F1TENTH 차량(AV)** 상에서 **Follow-the-Gap (FTG)** 반응형 주행 노드를 구현·튜닝·검증하는 것입니다.
Lab-05와 동일한 패키지 제작 흐름을 따르되, 노드 로직은 아래 레퍼런스 코드를 활용합니다:

* 참고 코드: `reactive_node.py`
  [https://github.com/jinkimh/f1tenth-software-stack/blob/main/gap_follow/scripts/reactive_node.py](https://github.com/jinkimh/f1tenth-software-stack/blob/main/gap_follow/scripts/reactive_node.py)

FTG는 LiDAR `/scan`으로부터 전처리(노이즈 제거, 범위 클리핑), **버블(bubble) 마스킹**, **최대 갭 탐색**, **갭 중앙 조향**(steer-to-midpoint), **속도 프로파일링** 단계로 동작합니다.

---

## 2) 팀 구성 및 역할 (예시)

| 역할                 | 담당 업무                     | 산출물             |
| ------------------ | ------------------------- | --------------- |
| Team Leader        | 일정/리스크 관리, 통합, 최종 리뷰      | 실행 로그, 최종 보고서   |
| Dev A (Perception) | LiDAR 전처리/버블/갭 탐색 파라미터 튜닝 | 파라미터 표/비교 그래프   |
| Dev B (Control)    | 조향/속도 프로파일, 한계치 설정        | 속도-조향 맵, 실험 결과  |
| Dev C (Systems)    | 패키지/런치/토픽 검증, bringup 지원  | 토픽 캡처, 런치 로그    |
| Doc Manager        | 문서화/정리/제출물 패키징            | PDF 보고서, 이미지/영상 |

---

## 3) 개발 환경 (AV 실차·Foxy)

| 항목        | 내용                                         |
| --------- | ------------------------------------------ |
| OS        | Ubuntu 20.04 LTS                           |
| Python    | 3.8                                        |
| ROS 2     | Foxy Fitzroy                               |
| Workspace | `~/f1tenth_ws`                             |
| 의존 패키지    | `sudo apt install ros-foxy-ackermann-msgs` |

ROS 환경 설정:

```bash
source /opt/ros/foxy/setup.bash
```

### 안전 수칙

* 초기 테스트는 **정비 스탠드/바퀴 미접지** 상태에서 명령값만 확인
* **E-Stop** 준비, 초기 **저속 파라미터**
* **실내/안전 구역**에서 단계적으로 시범

---

## 4) 패키지 생성 (새로 만들 경우)

> Lab-05와 동일 흐름. 기존 패키지가 있다면 재사용 가능.

```bash
cd ~/f1tenth_ws/src
ros2 pkg create --build-type ament_python gap_follow
```

생성 구조(예):

```
gap_follow/
  package.xml
  setup.py
  setup.cfg
  resource/gap_follow
  gap_follow/__init__.py
  gap_follow/reactive_node.py      # ← 이 파일을 새로 추가
```

---

## 5) 코드 반영 (reactive_node.py)

1. 파일 생성 및 붙여넣기

```bash
cd ~/f1tenth_ws/src/gap_follow
mkdir -p gap_follow
nano gap_follow/reactive_node.py
```

* 위 링크의 `reactive_node.py` 내용을 **복사/붙여넣기** 합니다.

2. **Foxy/실차 토픽 확인 및 수정**

* 구독: `/scan` (sensor_msgs/LaserScan)
* 발행: `/drive` (ackermann_msgs/AckermannDriveStamped)
* **/ego_racecar/** 네임스페이스가 코드에 있다면 제거하세요.
  예) `/ego_racecar/scan` → `/scan`
* 필요 시 런치/실행에서 remap 사용 가능:

  ```bash
  ros2 run gap_follow follow_the_gap_node --ros-args -r /scan:=/your_scan
  ```

3. **rclpy 엔트리포인트 확인**

* 파일 말미에 `def main(args=None):` 함수와 `rclpy.init()` → `rclpy.spin()` → `rclpy.shutdown()` 흐름이 있어야 함.

---

## 6) setup.py 설정

```bash
nano ~/f1tenth_ws/src/gap_follow/setup.py
```

예시:

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

## 7) package.xml 의존성

```xml
<!-- ~/f1tenth_ws/src/gap_follow/package.xml -->
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

## 8) 빌드 & 소싱

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
```

---

## 9) 차량 Bringup

터미널 1:

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

토픽 확인:

```bash
ros2 topic list
# /scan, /odom, /drive 가 보여야 정상
```

---

## 10) FTG 노드 실행 & 검증

터미널 2:

```bash
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 run gap_follow follow_the_gap_node
```

터미널 3(옵션):

```bash
ros2 topic echo /scan
ros2 topic echo /drive
```

---

## 11) 파라미터/튜닝 가이드 (권장)

> 코드에 노출된 상수/파라미터 이름은 레포 버전에 따라 다를 수 있음—실제 변수명을 확인하세요.

* **전처리**

  * `range_clip_max` (예: 10~12 m) 이상 값 클리핑
  * 이동평균/가우시안 스무딩 윈도우 (예: 3–7)
* **버블 크기(bubble_radius)**

  * 가장 가까운 충돌 후보 전후로 N개 빔 제거
  * 너무 작으면 충돌 위험, 너무 크면 과도한 회피
* **최대 갭 선택**

  * 최소 길이 임계(갭이 너무 짧으면 무시)
  * 갭 중앙 인덱스 조향(steer-to-mid)
* **속도 프로파일**

  * 조향각 |δ|와 최대범위/갭길이로 속도 스케일
  * 예: `v = clamp(v_min, v_max - k*|δ|, v_max)`
* **보수적 초기값**

  * `v_max` 낮게 시작(예 0.5–1.0 m/s), 조향 민감도 완만

---

## 12) 문제해결 체크리스트

* `/scan` 프레임/토픽 불일치 → remap 또는 코드 토픽 수정
* 메시지 timestamp 지연으로 튐 → smoothing 윈도우/QoS 조정 고려
* 과도한 좌우 흔들림 → 갭 선택 히스테리시스, 중앙 평균화
* 직선로에서 저속 유지 → v_max 상향, |δ| 기반 감속 항 조정
* 근거리 장애물 과민 → bubble/clip/σ(스무딩) 재조정

---

## 13) 팀 제출물

1. `~/f1tenth_ws/src/gap_follow` 전체 디렉터리(코드, `package.xml`, `setup.py`)
2. bringup/FTG 실행 화면 캡처 및 `/drive` 로그
3. **파라미터 튜닝 표**(시도값/결과/선정값)
4. **최종 PDF 보고서**: 개요, 코드/토픽 변경, 튜닝 과정, 주행 결과 분석(성공/실패 케이스, 개선점)

---

## 14) 평가 기준 (총 100점)

| 항목         | 배점 | 기준                                |
| ---------- | -: | --------------------------------- |
| 팀 협업/역할 수행 | 10 | 역할 명확성, 일정 준수                     |
| 코드 구현 정확성  | 25 | FTG 로직 적용, 구조/주석                  |
| 패키지/설정 정확성 | 15 | `package.xml`, `setup.py`, 엔트리포인트 |
| Bringup·실행 | 15 | 토픽 일치, 안정 실행                      |
| 튜닝 및 주행 성능 | 25 | 직선 안정성, 커브 추종, 충돌 회피              |
| 보고서 품질     | 10 | 재현성, 인사이트, 개선 제안                  |

---

## 15) 요약

* Lab-05 흐름을 유지하되, **FTG 로직으로 교체**
* 실차 기본 토픽(`/scan`, `/drive`)에 맞게 **네임스페이스 제거/정합**
* **보수적 파라미터**로 시작 → 단계적 튜닝으로 안정 주행 달성
* 코드/튜닝/로그/분석을 **팀 보고서**로 제출

행운을 빕니다! 안전을 최우선으로, 천천히 정확하게 튜닝해 봅시다.
