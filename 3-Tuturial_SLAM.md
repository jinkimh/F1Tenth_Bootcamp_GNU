
# F1TENTH SLAM Toolbox 설치 및 맵 생성 가이드

이 문서는 F1TENTH 플랫폼에서 `slam_toolbox`를 설치하고, 실시간 SLAM을 통해 지도를 생성하며, RViz2에서 시각화 및 저장하는 방법을 안내합니다.

---

## 1. slam_toolbox 설치

```bash
sudo apt install ros-foxy-slam-toolbox
````

> ⚠️ 반드시 차량의 오도메트리(odometry)가 잘 튜닝되어 있어야 SLAM 성능이 정상적으로 나옵니다.

---

## 2. 네트워크 및 장치 연결 설정

* NoMachine이 **host PC와 F1Tenth AV**에 연결되어 있어야 합니다.
* IP `192.168.0.10`, `192.168.0.15`는 **Hokuyo LiDAR의 이더넷 연결에 사용**되므로 해당 IP로 접근하지 않도록 네트워크를 분리해야 합니다.
* NoMachine을 통해 원격 연결하더라도 **F1Tenth AV에는 HDMI 동글이 연결되어 있어야** 키보드 및 화면 조작이 가능합니다.

---

## 3. SLAM 실행 순서

### 3.1. ROS2 및 워크스페이스 설정

```bash
cd ~/f1tenth_ws
colcon build
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

### 3.2. 차량 시스템 실행

```bash
ros2 launch f1tenth_gym_ros bringup_launch.py
```

> 이 명령은 차량 센서 및 주행 관련 노드를 실행합니다.

### 3.3. RViz2 실행 (다른 터미널)

```bash
source /opt/ros/foxy/setup.bash
rviz2
```

---

## 4. SLAM Toolbox 실행

```bash
source /opt/ros/foxy/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/home/yourid/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
```

> `"yourid"`는 실제 사용자 계정으로 변경해야 합니다.

---

## 5. RViz2 시각화 설정

RViz2에서 다음 항목들을 추가하여 SLAM 시각화를 활성화합니다.

* **Add by topic**:

  * `/map`
  * `/graph_visualization`

* **패널 추가**:

  * 상단 메뉴 → `Panels` → `Add New Panel`
  * `SlamToolBoxPlugin` 패널 추가

---

## 6. 맵 저장 (RViz 내에서)

1. `SlamToolBoxPlugin` 패널에서 `"Save Map"` 버튼 클릭
2. 오른쪽 입력란에 맵 이름 입력 (예: `my_map`)
3. 맵은 SLAM Toolbox를 실행한 디렉토리에 저장됩니다.

### 생성되는 파일 예시:

* `my_map.pgm`
* `my_map.yaml`
* (경우에 따라 `.csv` 또는 `.txt` 파일 포함)

> 기본 저장 위치는 `~/f1tenth_ws`입니다.

---

## 7. NoMachine 연결 요약

**F1Tenth AV에 원격 접속하려면:**

1. Host PC에서 **NoMachine** 실행
2. **새 연결 추가** → F1Tenth AV의 IP 입력
3. **프로토콜 선택**: SSH
4. **사용자명/비밀번호 입력**하여 연결

> 키보드 및 GUI 사용을 위해 F1Tenth 차량에 **HDMI 동글 연결 필수**

---

## 전체 실행 요약

```bash
# 1. bringup 실행 (터미널 1)
ros2 launch f1tenth_gym_ros bringup_launch.py

# 2. SLAM Toolbox 실행 (터미널 2)
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/home/yourid/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml

# 3. RViz 실행 (터미널 3)
rviz2
```

---

## 참고 사항

* 오도메트리와 LiDAR 위치가 정확해야 SLAM 결과가 왜곡되지 않습니다.
* 맵 저장 시 `.pgm`, `.yaml` 파일 외에 `.csv` 그래프 정보도 함께 저장될 수 있습니다.
* SLAM 중 RViz가 느려질 수 있으므로, 리소스가 부족한 환경에서는 패널을 최소화하세요.

```

---
