# F1Tenth 시뮬레이터 활용 — 드라이빙 알고리즘 실행 (Wall Following & Pure Pursuit)

이 문서는 F1Tenth 시뮬레이터 환경에서 **Wall Following**과 **Pure Pursuit** 알고리즘을 실행하는 절차를 안내합니다.

---

## 1. ROS2 환경 설정 및 빌드

시뮬레이터 작업 디렉토리(`/sim_ws`)에서 다음 명령어를 실행합니다:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash   # ROS2 Foxy 환경 설정
/sim_ws$ colcon build                      # 작업 공간 빌드
/sim_ws$ source install/setup.bash         # 빌드된 ROS2 패키지 환경 설정
````

* `setup.bash` : ROS2 환경 변수 불러오기
* `colcon build` : ROS2 워크스페이스 빌드

---

## 2. Wall Following 실행

### (1) 시뮬레이터 실행

새로운 터미널을 열어 Docker 컨테이너에 접속 후:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

* `gym_bridge_launch.py` : 시뮬레이터 GUI 실행 및 ROS2 연결

### (2) Wall Following 노드 실행

다른 터미널에서 Docker 컨테이너에 접속 후:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 run wall_follow wall_follow_node.py
```

* `wall_follow_node.py` : 벽을 따라 주행하는 알고리즘 실행

---

## 3. Pure Pursuit 실행

Pure Pursuit 알고리즘은 별도의 노드나 패키지에서 실행할 수 있으며, 방법은 Wall Following 실행과 동일합니다.
예시:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 run pure_pursuit pure_pursuit_node.py
```

* `pure_pursuit_node.py` : 지정된 웨이포인트 경로를 따라 주행하는 알고리즘 실행

---

## 4. 주요 파일 및 디렉토리

### (1) 지도 파일 (Map Files)

* 확장자: `.png`, `.pgm`, `.yaml`
* 역할: 맵 이미지(장애물·주행 경로) + 메타데이터(해상도, 원점 등)
* 위치:

```bash
src/f1tenth_gym_ros/maps/
```

### (2) 웨이포인트 파일 (Waypoint Files)

* 확장자: `.csv`
* 역할: Pure Pursuit 경로 좌표 데이터
* 위치:

```bash
src/f1tenth-software-stack/csv_data/
```

* 맵 파일과 동일한 이름 권장 (`map_xxx.png` ↔ `map_xxx.csv`)

---

## 5. 새로운 맵으로 실행하는 방법

1. 맵 파일을 `src/f1tenth_gym_ros/maps/`에 추가
2. 해당 맵에 맞는 웨이포인트 파일을 `src/f1tenth-software-stack/csv_data/`에 저장
3. 시뮬레이터 실행 → Pure Pursuit 또는 Wall Following 노드 실행

---

## 6. 결론

맵과 웨이포인트를 직접 설정해 다양한 시뮬레이션을 수행할 수 있으며, 이를 통해 Wall Following과 Pure Pursuit 알고리즘 성능을 비교·개선할 수 있습니다.

```

