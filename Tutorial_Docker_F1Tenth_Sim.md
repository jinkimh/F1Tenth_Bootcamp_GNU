# 🐳 F1TENTH 시뮬레이터용 Docker 컨테이너 사용법 (전체 튜토리얼)

이 문서는 F1Tenth 시뮬레이터용 Docker 환경을 **처음부터** 설정하고  
**이미지 생성 → 컨테이너 실행 → 재접속 및 GUI 실행**까지 전 과정을 안내합니다.

---

## 📦 1. Docker 이미지 준비

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

## 🧱 2. 컨테이너 최초 실행

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

## 🚪 3. 컨테이너 종료 방법

```bash
# 방법 1
exit

# 방법 2 (단축키)
Ctrl + D
```

> 컨테이너는 삭제되지 않고 **정지된 상태**로 남아 있게 됩니다.

---

## 🔁 4. 컨테이너 재실행 및 재접속

컨테이너를 다시 실행하고 들어가는 명령어는 다음과 같습니다:

```bash
# (1) 컨테이너 재시작
docker start f110_gym_docker

# (2) bash 셸로 접속
docker exec -it f110_gym_docker bash
```

> `docker run`은 처음 한 번만 사용하며, 이후엔 항상 `start` + `exec` 조합을 사용합니다.

---

## 🖼️ 5. GUI 프로그램(RViz 등)을 위한 접속

```bash
docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  f110_gym_docker bash
```

> GUI 동작이 안 될 경우, 호스트에서 다음 명령어를 먼저 실행하세요:

```bash
xhost +local:docker
```

---

## 🛠️ 6. 유용한 Docker 명령어 모음

```bash
# 컨테이너 목록 보기 (정지된 것도 포함)
docker ps -a

# 컨테이너 강제 종료
docker stop f110_gym_docker

# 컨테이너 삭제
docker rm f110_gym_docker

# 이미지 삭제
docker rmi f1tenth_gym_ros:latest
```

---

## 🚀 7. 실행 alias 등록 (선택 사항)

자주 사용하는 명령어를 단축어로 등록해두면 편리합니다.

### ✅ .bashrc 또는 .zshrc에 다음 내용 추가:

```bash
# 일반 접속
alias f1start="docker start f110_gym_docker && docker exec -it f110_gym_docker bash"

# GUI 접속
alias f1gui='docker start f110_gym_docker && docker exec -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 f110_gym_docker bash'
```

### 적용하기

```bash
source ~/.bashrc  # 또는 ~/.zshrc
```

### 사용 예시

```bash
f1start   # 텍스트 셸 접속
f1gui     # RViz 등 GUI 사용 가능 접속
```

---

## 📂 디렉토리 구조 예시

```text
f1tenth_ws/
├── src/
│   ├── f1tenth_gym_ros/
│   └── f1tenth-software-stack/
├── install/
├── build/
├── docker_run_f1tenth_sim.sh (선택)
```

---

## 📄 저작권 정보

```
© 2025 Jin Kim, Gyeongsang National University

본 문서는 F1TENTH 시뮬레이터 및 자율주행 교육용으로 제작되었으며,
교육 및 연구 목적 외의 무단 복제 및 재배포를 금지합니다.
```

---

```

---

필요하시면 이 문서를 실제 `docker_usage_tutorial.md` 파일로 저장해드릴 수 있고, `.pdf` 형식으로 변환하거나 실행 스크립트도 함께 생성해드릴 수 있습니다. 원하시면 알려주세요!
```
