# 🚗 F1TENTH 드라이버 스택 설정 및 LiDAR 테스트 매뉴얼

이 문서는 F1TENTH 자율주행 차량에서 ROS2 기반의 **드라이버 스택 설치**, **LiDAR 연결 테스트**, 그리고 **RViz를 통한 시각화** 절차를 안내합니다.

---

## 1. 드라이버 스택용 워크스페이스 생성

### 💡 설명  
드라이버 스택을 설치할 ROS2 워크스페이스(`f1tenth_ws`)를 생성합니다.

```bash
cd $HOME
mkdir -p f1tenth_ws/src
````

### ROS2 워크스페이스 초기화

```bash
cd f1tenth_ws
colcon build
```

---

## 2. 드라이버 스택 저장소 클론 및 서브모듈 초기화

### 💡 저장소 클론

```bash
cd src
git clone https://github.com/f1tenth/f1tenth_system.git
```

### 💡 Git 서브모듈 초기화 및 업데이트

```bash
cd f1tenth_system
git submodule update --init --force --remote
```

---

## 3. 패키지 의존성 설치

### 💡 rosdep 업데이트 및 의존성 설치

```bash
cd $HOME/f1tenth_ws
rosdep update
rosdep install --from-paths src -i -y
```

---

## 4. 드라이버 스택 포함 전체 워크스페이스 빌드

```bash
colcon build
```

> 📎 더 자세한 설명은 [f1tenth\_system GitHub 저장소](https://github.com/f1tenth/f1tenth_system)의 README를 참조하세요.

---

## 5. Teleop 및 LiDAR 테스트

이 절차는 LiDAR가 차량에 연결된 상태를 전제로 합니다.
(Hokuyo 10LX 사용 시 이더넷 연결이 선행되어야 함)

---

## 6. LiDAR 파라미터 설정

### 💡 설정 파일 위치

```bash
$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/sensors.yaml
```

### LiDAR 종류에 따른 설정 방법

#### ➤ Ethernet 기반 LiDAR (예: Hokuyo 10LX)

```yaml
ip_address: "192.168.x.x"  # 해당 LiDAR IP 주소로 설정
# serial_port 항목은 주석 처리
```

#### ➤ USB 기반 LiDAR

```yaml
# ip_address 항목은 주석 처리
serial_port: "/dev/ttyUSB_LIDAR"  # udev 규칙에 따라 지정된 이름 사용
```

---

## 7. ROS2 환경 설정 및 bringup 실행

### ROS2 및 워크스페이스 환경 불러오기

```bash
source /opt/ros/foxy/setup.bash
cd $HOME/f1tenth_ws
source install/setup.bash
```

### bringup 실행

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

> ✅ 이 명령은 VESC 드라이버, LiDAR 드라이버, 조이스틱 드라이버 등 차량 실행에 필요한 모든 노드를 시작합니다.

---

## 8. RViz를 통한 LaserScan 메시지 시각화

### 새로운 터미널에서 아래 명령 실행

```bash
source /opt/ros/foxy/setup.bash
cd $HOME/f1tenth_ws
source install/setup.bash
rviz2
```

### RViz 설정

1. `/scan` 토픽을 시각화할 수 있도록 **LaserScan** 디스플레이 추가
2. LiDAR 센서의 거리 데이터를 시각적으로 확인 가능

---

## 📝 요약

| 항목     | 설명                                                            |
| ------ | ------------------------------------------------------------- |
| 워크스페이스 | `f1tenth_ws`                                                  |
| 주요 저장소 | [`f1tenth_system`](https://github.com/f1tenth/f1tenth_system) |
| 설정 파일  | `vesc.yaml`, `sensors.yaml`                                   |
| 실행 명령  | `ros2 launch f1tenth_stack bringup_launch.py`                 |
| 시각화 도구 | `rviz2`                                                       |

---

## 📄 저작권

```
© 2025 Jin Kim, Gyeongsang National University

본 문서는 F1Tenth 자율주행 플랫폼의 ROS2 환경 설정 및 센서 테스트 실습용으로 제작되었습니다.  
연구 및 교육 목적 외의 무단 복제 및 배포를 금지합니다.
```

```

---
 
