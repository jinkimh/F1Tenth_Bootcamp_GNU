# F1Tenth AV 시뮬레이터 실행 가이드

## 1장. 사전 준비

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

## 2장. F1Tenth Docker 이미지 빌드

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

## 3장. Docker 컨테이너 실행

### 옵션 1: 명령어 직접 실행

```bash
cd ~/f1tenth_ws
xhost +local:docker

docker run -it \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd)/src/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros" \
  --volume="$(pwd)/src/f1tenth-software-stack:/sim_ws/src/f1tenth-software-stack" \
  --name f110_gym_docker \
  f1tenth_gym_ros:latest
```

### 옵션 2: 실행 스크립트 사용

```bash
chmod +x docker_run_f1tenth_sim.sh
./docker_run_f1tenth_sim.sh
```

컨테이너 실행 후 프롬프트 예시:

```
root@12ff300395b4:/sim_ws#
```

---

## 4장. 시뮬레이터와 `wall_follow` 실행

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

## 5장. 실행 확인

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

```

---

이렇게 정리하면 한눈에 실행 순서와 필요한 명령어를 확인할 수 있고,  
GitHub README.md에 그대로 넣어도 가독성이 좋습니다.  

원하시면 제가 여기에 **환경 구성도**와 **전체 실행 흐름 다이어그램**까지 추가해서 Bootcamp 자료로 쓸 수 있게 만들어 드릴까요?  
그렇게 하면 참가자들이 설치-빌드-실행 관계를 한 번에 이해할 수 있습니다.
```
