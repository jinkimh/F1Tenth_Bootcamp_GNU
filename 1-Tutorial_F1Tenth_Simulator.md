# F1Tenth AV 시뮬레이터 실행 튜토리얼

## 1. 사전 준비

F1Tenth Simulator를 실행하기 위해 다음 환경이 필요합니다.

### Step 1: Ubuntu 또는 WSL(Ubuntu) 설치
- **권장 버전:** Ubuntu 22.04 또는 20.04

- **Windows 사용자의 경우:**  
  Windows 환경에서도 리눅스 명령어와 개발 도구를 사용하기 위해 **WSL(Windows Subsystem for Linux)**를 설치하여 Ubuntu 환경을 구성할 수 있습니다.

```
#### WSL 설치 방법
1. **PowerShell 관리자 모드 실행**  
   - Windows 검색창에 `PowerShell` 입력 → "관리자 권한으로 실행" 선택

2. **WSL 및 Ubuntu 설치 명령어 입력**
   ```powershell
   wsl --install -d Ubuntu-22.04
```

### Step 2: ROS2 설치
- **필수 버전:** ROS2 Foxy (Humble도 지원 가능)
- ROS2는 F1Tenth 시뮬레이터와 ROS 노드 실행의 핵심 프레임워크입니다.
- ROS2 Foxy 설치 방법은 공식 문서에서 확인할 수 있습니다:  
  [ROS2 Foxy Installation Guide](https://docs.ros.org/en/foxy/Installation.html)


### Step 3: Docker 설치
- **Ubuntu:** 리눅스 Docker 설치 가이드 참고  
  [Docker Engine on Ubuntu 설치 가이드](https://docs.docker.com/engine/install/ubuntu/)
- **Windows 11:** Docker Desktop 설치 후 Linux 컨테이너 모드로 실행  
  [Docker Desktop for Windows 설치 가이드](https://docs.docker.com/desktop/install/windows-install/)

---

## 2. F1Tenth Docker 이미지 빌드

### 1. 작업 디렉토리 생성
```bash
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src
````

### 2. Git 저장소 클론

```bash
git clone https://github.com/jinkimh/f1tenth_gym_ros.git
git clone https://github.com/jinkimh/f1tenth-software-stack.git
```

### 3. Docker 이미지 빌드

```bash
cd f1tenth_gym_ros
docker build -t f1tenth_gym_ros -f Dockerfile .
```

---

## 3. Docker 컨테이너 실행

### 옵션 1: 명령어 직접 실행

```bash
cd ~/f1tenth_ws
xhost +local:docker

docker run -it \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/f1tenth_ws/src/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros" \
  --volume="$HOME/f1tenth_ws/src/f1tenth-software-stack:/sim_ws/src/f1tenth-software-stack" \
  --name f110_gym_docker \
  f1tenth_gym_ros:latest
```
컨테이너 실행 후 프롬프트 예시:

```
root@12ff300395b4:/sim_ws#
```
### 옵션 2: 실행 스크립트 사용

### 1. 실행 스크립트 만들기

다음 내용을 `docker_run_f1tenth_sim.sh` 파일로 저장합니다:

```bash
#!/bin/bash

# F1Tenth 시뮬레이터용 Docker 실행 스크립트

cd ~/f1tenth_ws || { echo "f1tenth_ws 디렉토리가 존재하지 않습니다."; exit 1; }

xhost +local:docker

docker run -it \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/f1tenth_ws/src/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros" \
  --volume="$HOME/f1tenth_ws/src/f1tenth-software-stack:/sim_ws/src/f1tenth-software-stack" \
  --name f110_gym_docker \
  f1tenth_gym_ros:latest
```

### 2. 실행 권한 부여

```bash
chmod +x docker_run_f1tenth_sim.sh
```

### 3. 스크립트 실행


```bash
chmod +x docker_run_f1tenth_sim.sh
./docker_run_f1tenth_sim.sh
```

컨테이너 실행 후 프롬프트 예시:

```
root@12ff300395b4:/sim_ws#
```

---

## 4. 시뮬레이터와 `wall_follow` 실행

### Step 1: 시뮬레이터 실행

```bash
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

* 실행 후 GUI 시뮬레이터 창이 표시됨

### Step 2: 새로운 터미널에서 컨테이너 접속

```bash
docker ps -a  # 컨테이너 ID 확인
docker exec -it <container_id> /bin/bash
```

### Step 3: `wall_follow` 실행

```bash
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
ros2 run wall_follow wall_follow_node.py
```

---

## 5. 실행 확인

### ROS2 토픽 확인

```bash
ros2 topic list
```

* `scan`, `odom`, `cmd_vel` 등 주요 토픽 연결 여부 확인

### 로그 확인

* `wall_follow_node.py` 실행 시 출력되는 메시지로 동작 상태 점검

---

## 결과

* 시뮬레이터에서 차량이 벽을 따라 주행하는 동작 확인 가능
* 정상 동작 후 알고리즘 수정 및 주행 방식 확장 가능

---

# 🧭 ROS2 Teleop Twist Keyboard 실행 가이드

이 문서는 **Docker 컨테이너 내부에서 ROS2 환경을 설정하고**,  
`teleop_twist_keyboard` 노드를 통해 키보드 입력으로 자율주행 차량을 수동 조작하는 방법을 안내합니다.

---

## 🚪 0. Docker 컨테이너 재진입

이미 생성된 Docker 컨테이너(`f110_gym_docker`)에 접속하려면 다음 명령어를 사용합니다:

```bash
# 컨테이너 실행 (백그라운드)
docker start f110_gym_docker

# bash 셸로 진입
docker exec -it f110_gym_docker bash
````

> ⚠️ GUI를 사용하는 경우에는 아래 명령을 사용하세요:

```bash
docker exec -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 f110_gym_docker bash
```

---

## ⚙️ 1. ROS2 환경 설정

Docker 컨테이너 내부에서 ROS2를 사용하려면
ROS2 Foxy와 작업 공간(`sim_ws`) 환경을 아래와 같이 설정합니다:

```bash
# ROS2 Foxy 설정
source /opt/ros/foxy/setup.bash

# sim_ws 작업 공간 설정
source install/setup.bash
```

---

## 🕹️ 2. teleop\_twist\_keyboard 실행

키보드 입력으로 차량의 속도와 방향을 제어할 수 있도록
teleop\_twist\_keyboard 노드를 실행합니다:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🎮 3. 키보드 조작법 요약

실행 후 아래 키를 눌러 차량을 조작할 수 있습니다:

| 동작    | 키                 |
| ----- | ----------------- |
| 전진    | `i`               |
| 후진    | `,`               |
| 좌회전   | `j`               |
| 우회전   | `l`               |
| 속도 증가 | `u` (앞) / `m` (뒤) |
| 속도 감소 | `o` (앞) / `.` (뒤) |
| 정지    | `k`               |
| 종료    | `Ctrl + C`        |

---

## 📝 참고 사항

* 실행 전 반드시 다음 두 명령어로 ROS2 환경을 불러와야 합니다:

  ```bash
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  ```

* `teleop_twist_keyboard`는 터미널 상에서 키 입력을 직접 받아 처리하므로,
  실행 중인 터미널을 **항상 활성화 상태로 유지**해야 합니다 (즉, 창을 클릭한 상태).

---

## 📄 예시 실행 흐름

```bash
# 1. 컨테이너 접속
docker start f110_gym_docker
docker exec -it f110_gym_docker bash

# 2. 환경 설정
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 3. 노드 실행
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 📄 저작권

```
© 2025 Jin Kim, Gyeongsang National University

본 문서는 ROS2 및 F1Tenth 기반 실습용으로 제작되었으며,
교육 및 연구 목적 외의 무단 복제 및 재배포를 금지합니다.
```

