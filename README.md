# F1TENTH 시뮬레이션 및 자율주행 튜토리얼

이 저장소는 F1TENTH 시뮬레이터 및 실제 자율주행 차량(F1Tenth AV)에서 다양한 자율주행 알고리즘을 실행하기 위한 실습 튜토리얼을 제공합니다.

## 📚 튜토리얼 개요

본 튜토리얼은 다음과 같은 주요 알고리즘 및 기능의 설치 및 실행 방법을 단계별로 안내합니다:

---

### 1️⃣ **Docker 기반 시뮬레이터 환경 구축**

- GUI를 위한 X11 설정 포함
- `docker_run_f1tenth_sim.sh` 스크립트 제공
- 시뮬레이터 내 ROS2 환경 구성 및 실행

---

### 2️⃣ **Particle Filter 기반 Localization**

- SLAM으로 생성한 맵 활용
- `particle_filter` 패키지 설치 및 실행
- RViz를 통한 위치 추정 시각화
- 웨이포인트 기반 경로 주행을 위한 초기 위치 설정

---

### 3️⃣ **SLAM Toolbox를 활용한 지도 생성**

- `slam_toolbox` 설치 및 실행
- 실시간 맵 생성
- RViz에서 SLAM 시각화 및 맵 저장
- NoMachine을 통한 실제 F1Tenth AV 연결 및 제어

---

### 4️⃣ **Pure Pursuit (PP) 기반 자율주행**

- 시뮬레이터 또는 실제 차량에서 PP 알고리즘 실행
- `f1tenth-software-stack`의 Pure Pursuit 노드 사용
- 웨이포인트 `.csv` 파일 기반 경로 추종 주행
- 시뮬레이터 전용 설정 (`is_real = False`) 및 실제 차량용 설정 (`is_real = True`)

---

### 5️⃣ **수동 웨이포인트 기반 주행**

- `(x, y)` 포맷의 수동 웨이포인트 파일 활용
- `pure_pursuit_node.py` 내부 코드 수정 (CSV 파싱, 속도 설정 등)
- 고정 속도 주행 설정 (`self.drive_msg.drive.speed = 2.0`)

---

## 🛠️ 실행 환경

- ROS2 Foxy
- Ubuntu 20.04 (권장)
- F1Tenth 시뮬레이터 Docker 이미지# F1Tenth Bootcamp  
- 실제 F1Tenth AV 또는 시뮬레이터 환경에서 실행 가능

---

## 📂 디렉토리 구성

```bash
f1tenth_ws/
├── src/
│   ├── f1tenth_gym_ros/
│   ├── f1tenth-software-stack/
│   ├── particle_filter/
│   └── ...
├── install/
└── build/ 
````

---

## 🔒 라이선스 및 저작권

본 튜토리얼 및 관련 스크립트의 저작권은 다음과 같습니다:

```
© 2025 Jin Kim, Gyeongsang National University

본 자료는 교육 및 연구 목적으로 자유롭게 활용할 수 있으나,
사전 동의 없이 상업적 목적의 사용 또는 2차 배포는 금지됩니다.
```

---

## 📬 문의

문의 사항이나 피드백은 다음 이메일로 보내주세요:

**[jin.kim@gnu.ac.kr](mailto:jin.kim@gnu.ac.kr)**

```




