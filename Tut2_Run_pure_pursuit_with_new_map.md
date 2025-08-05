# Pure Pursuit 알고리즘 실행 가이드

F1Tenth 시뮬레이터 환경에서 **Pure Pursuit 알고리즘**을 사용해 차량을 주행시키는 절차를 설명합니다.

---

## 1. ROS2 환경 설정 및 빌드

시뮬레이터 작업 디렉토리(`/sim_ws`)에서 다음 명령어를 실행합니다:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash   # ROS2 Foxy 환경 설정
/sim_ws$ colcon build                      # 작업 공간 빌드
/sim_ws$ source install/setup.bash         # 빌드된 ROS2 패키지 환경 설정
````

* `setup.bash` : ROS2 환경 변수를 불러옵니다.
* `colcon build` : ROS2 워크스페이스를 빌드합니다.

---

## 2. Pure Pursuit 실행

### (1) 시뮬레이터 실행

새로운 터미널을 열어 Docker 컨테이너에 접속 후, 아래 명령어를 입력합니다:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

* `gym_bridge_launch.py` : F1Tenth 시뮬레이터 GUI를 실행하고 ROS2와 연결하는 런치 파일입니다.

### (2) Pure Pursuit 노드 실행

또 다른 터미널을 열어 동일하게 Docker 컨테이너에 접속 후:

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 run wall_follow wall_follow_node.py
```

* `wall_follow_node.py` : Pure Pursuit 알고리즘 기반으로 차량이 경로를 따라 주행하도록 하는 노드입니다.

---

## 3. 주요 파일 및 디렉토리

### (1) 지도 파일 (Map Files)

* 확장자: `.png`, `.pgm`, `.yaml`
* **역할:** 맵 이미지는 장애물 및 주행 경로를, `.yaml` 파일은 해상도·좌표계 정보를 저장합니다.
* 저장 위치:

```bash
src/f1tenth_gym_ros/maps/
```

### (2) 웨이포인트 파일 (Waypoint Files)

* 확장자: `.csv`
* **역할:** Pure Pursuit 알고리즘이 따라갈 경로 좌표를 저장합니다.
* 저장 위치:

```bash
src/f1tenth-software-stack/csv_data/
```

* 맵 파일 이름과 일치하는 파일명 사용 권장 (`map_xxx.png` ↔ `map_xxx.csv`)

---

## 4. 새로운 맵으로 실행하는 방법

1. 맵 파일을 `src/f1tenth_gym_ros/maps/`에 추가
2. 해당 맵에 맞는 웨이포인트 파일을 `src/f1tenth-software-stack/csv_data/`에 저장
3. `gym_bridge_launch.py` 실행 후 Pure Pursuit 노드 실행

---

## 5. 결론

맵과 웨이포인트를 직접 설정하면 다양한 주행 시나리오를 실험할 수 있으며, 이를 통해 Pure Pursuit 알고리즘의 성능을 평가하고 최적화할 수 있습니다.

```

