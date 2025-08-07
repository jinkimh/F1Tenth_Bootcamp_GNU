# F1Tenth AV에서 Pure Pursuit 실행 
(Optimal Trajectory Generator를 활용한 WP를 사용하는 경우)

이 문서는 실제 F1Tenth AV 차량에서 optimal Trajectory Generator (wp_gen.py)를 활용한 WP를 사용하는 경우, Pure Pursuit(PP) 알고리즘을 실행하기 위한 전체 절차를 정리한 가이드입니다.

---

## 📦 사전 준비

### 1. 맵 및 맵 설정 파일 준비

- SLAM Toolbox를 통해 `.pgm`, `.yaml` 형식의 맵 파일을 생성합니다.

### 2. Particle Filter 설정

- 생성한 맵과 맵.yaml 파일을 다음 위치에 복사합니다:

```text
particle_filter/maps/
````

* 다음 설정 파일을 열어 사용할 맵 이름을 수정합니다:

```bash
particle_filter/config/localize.yaml
```

---

## 🚘 주행 알고리즘 다운로드
* ~/f1tenth_ws/src/f1tenth-software-stack 가 존재하면 이 과정 생략
```bash
cd ~/f1tenth_ws/src
git clone https://github.com/jinkimh/f1tenth-software-stack.git
```

---

## ⚙️ Pure Pursuit 노드 설정 변경

다음 파일을 엽니다:

```bash
~/f1tenth_ws/src/f1tenth-software-stack/pure_pursuit/scripts/pure_pursuit_node.py
```

### 주요 수정 사항:

| 라인 | 코드                                                                                   | 설명                           |
| -- | ------------------------------------------------------------------------------------ | ---------------------------- |
| 25 | `self.is_real = False`                                                                | 실제 차량에서 실행할 경우 `True`로 설정    |
---

## 📁 웨이포인트 CSV 파일 복사

```bash
cp your_waypoints.csv ~/f1tenth_ws/src/f1tenth-software-stack/csv_data/
```

> `your_waypoints.csv` 파일은 SLAM 기반 주행 경로를 기준으로 생성된 파일이어야 합니다.

---

## ▶️ 실행 순서

### 1. Teleop 실행 (1번 터미널)

```bash
cd ~/f1tenth_ws/
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch
```

### 2. RViz 실행 (2번 터미널)

```bash
source /opt/ros/foxy/setup.bash
rviz2
```

### 3. Particle Filter 실행 (3번 터미널)

```bash
cd ~/f1tenth_ws/
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch particle_filter localize_launch.py
```
이 과정이 끝나면 rviz2의 맵 상에, '2d Pose Estimate' 버튼을 이용, 자동차의 임의로 위치를 설정합니다. 

### 4. Pure Pursuit 실행 (4번 터미널)

```bash
cd ~/f1tenth_ws/
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run pure_pursuit pure_pursuit_node.py
```

---

## ✅ 참고 사항

* `pure_pursuit_node.py`는 실제 차량 주행 모드 여부(`is_real`)에 따라 동작이 다르므로 반드시 설정을 확인하세요.
* Particle Filter의 초기 위치는 RViz에서 **"2D Pose Estimate"** 도구로 지정해야 합니다.
* 맵 이름과 Waypoint 파일 이름은 일치해야 하며, `.csv` 형식은 Pure Pursuit 코드와 호환되어야 합니다.

---
