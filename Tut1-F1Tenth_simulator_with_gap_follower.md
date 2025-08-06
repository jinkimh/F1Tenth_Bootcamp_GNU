# F1Tenth AV 시뮬레이터 실행 가이드

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
  --volume="$HOME/src/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros" \
  --volume="$HOME/src/f1tenth-software-stack:/sim_ws/src/f1tenth-software-stack" \
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

```
좋습니다. 그러면 \*\*"독커 컨테이너 사용법"\*\*이라는 제목으로,
2\. 컨테이너 종료부터 다시 정리한 `README.md` 예시를 드리겠습니다.

# 독커 컨테이너 사용법

---

## 2. 컨테이너 종료

컨테이너 실행 중 종료하려면 아래 방법 중 하나를 사용합니다.

- `exit` 명령어 입력
- `Ctrl + D` 단축키 사용

종료 후 컨테이너는 **정지 상태**가 되며, 삭제되지 않습니다.

---

## 3. 컨테이너 재실행 및 재접속

이미 생성된 컨테이너를 다시 실행하고 접속하려면 다음 명령을 사용합니다.

```bash
# 컨테이너 실행 (백그라운드 모드)
docker start f110_gym_docker

# 실행 중인 컨테이너에 bash로 접속
docker exec -it f110_gym_docker bash
````

> ⚠️ `docker run`은 **최초 생성 시 1회만** 사용합니다.
> 재접속 시에는 `docker start`와 `docker exec`만 사용합니다.

---

## 4. GUI/X11 환경 유지한 접속

X11 기반 GUI 프로그램을 실행하기 위해 DISPLAY 환경변수와
관련 설정을 전달하여 접속하려면 다음 명령을 사용합니다.

```bash
docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  f110_gym_docker bash
```

---

## 5. 유용한 명령어 모음

```bash
# 컨테이너 상태 확인
docker ps -a

# 컨테이너 중지
docker stop f110_gym_docker

# 컨테이너 삭제
docker rm f110_gym_docker

# 이미지 삭제
docker rmi f1tenth_gym_ros:latest
```

---

## 6. 팁: alias 설정 (선택 사항)

자주 사용하는 명령어를 단축어로 등록하면 편리합니다.
아래 내용을 `~/.bashrc` 또는 `~/.zshrc`에 추가 후 `source ~/.bashrc`로 반영하세요.

```bash
alias f1start="docker start f110_gym_docker && docker exec -it f110_gym_docker bash"
alias f1gui="docker start f110_gym_docker && docker exec -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 f110_gym_docker bash"
```

이제 `f1start` 또는 `f1gui` 명령만으로 컨테이너에 접속할 수 있습니다.

```


```

