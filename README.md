# F1Tenth Bootcamp  

## 1장. F1Tenth AV 시뮬레이터: 사전 준비

F1Tenth Simulator를 사용하기 위해 다음과 같은 필수 소프트웨어와 환경을 준비해야 합니다. 아래는 필요한 설치 항목들과 간단한 설명입니다.

---

### **Step 1: Ubuntu 혹은 WSL 내 Ubuntu 설치**  
- **권장 버전:** Ubuntu 22.04 또는 20.04  
  - WSL(Windows Subsystem for Linux) 사용 시, Windows 11에서 Ubuntu를 설치하여 리눅스 환경을 구축할 수 있습니다.

---

### **Step 2: ROS2 설치**  
- **필수 버전:** ROS2 Foxy  
  - **대안:** ROS2 Humble도 지원됩니다.  
  ROS2(Foxy 또는 Humble)는 로봇 운영 시스템으로, F1Tenth 시뮬레이터의 핵심 라이브러리를 제공합니다.  

---

### **Step 3: Docker 설치**  
- Docker는 컨테이너 기반 가상환경을 제공하여 소프트웨어를 격리된 환경에서 실행할 수 있도록 돕습니다.  
  - **Ubuntu 용:** 리눅스 기반의 Docker 설치 방법을 통해 환경을 설정합니다.  
  - **Windows 11 용:** Windows 전용 Docker Desktop을 설치하여 컨테이너 환경을 구축합니다.

---
## **2장. F1Tenth Docker 이미지 만들기**

F1Tenth 시뮬레이터를 실행하기 위한 Docker 이미지를 생성하는 과정은 다음과 같습니다.

---

### **1. 작업 디렉토리 생성 및 이동**  
```bash
mkdir -p ~/f1tenth_ws/src  
cd ~/f1tenth_ws/src  
```
- `~/f1tenth_ws/src`: 작업 공간 디렉토리 생성 및 이동  
  - `src` 폴더는 소스 코드 저장을 위한 디렉토리입니다.

---

### **2. Git 저장소 클론**  
```bash
git clone https://github.com/f1tenth/f1tenth_gym_ros.git  
git clone https://github.com/jinkimh/f1tenth-software-stack.git  
```
- `f1tenth_gym_ros`: F1Tenth 시뮬레이터 ROS2 패키지  
- `f1tenth-software-stack`: 자율주행 및 ROS 관련 주요 소프트웨어 패키지

---

### **3. Docker 이미지 빌드 준비 및 실행**  
```bash
cd f1tenth_gym_ros  
docker build -t f1tenth_gym_ros -f Dockerfile .  
```
- `f1tenth_gym_ros` 디렉토리로 이동 후 Docker 이미지를 빌드합니다.  
  - `-t f1tenth_gym_ros`: 이미지 이름 태그  
  - `-f Dockerfile`: Dockerfile을 사용해 이미지 생성

---

## **3장. Docker 컨테이너 및 시뮬레이터 실행**

F1Tenth 시뮬레이터를 실행하기 위해 Docker 컨테이너를 실행하는 방법은 아래 두 가지 방식 중 하나를 사용합니다.

---

### **옵션 1. 명령어를 직접 실행하는 방법**  
아래 명령어를 터미널에 입력하여 Docker 컨테이너를 실행합니다.

```bash
xhost +local:docker  # Docker가 GUI 디스플레이에 접근할 수 있도록 권한 부여

docker run -it \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd)/f1tenth_ws/src/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros" \
  --volume="$(pwd)/f1tenth_ws/src/f1tenth-software-stack:/sim_ws/src/f1tenth-software-stack" \
  --name f110_gym_docker \
  f1tenth_gym_ros:latest
```

- **옵션 설명**  
  - `--privileged`: 하드웨어 접근 권한을 부여  
  - `--env="DISPLAY"`: 그래픽 출력 설정  
  - `--volume`: 로컬 디렉토리를 Docker 컨테이너 내부와 연결 (마운트)  
  - `--name f110_gym_docker`: 컨테이너 이름을 `f110_gym_docker`로 설정  
  - `f1tenth_gym_ros:latest`: 사용할 Docker 이미지 이름  

---

### **옵션 2. 스크립트 실행 방법**

반복되는 명령어 입력을 방지하기 위해 쉘 스크립트를 사용합니다.

1. **쉘 스크립트 생성 및 다운로드**  
   `docker_run_f1tenth_sim.sh` 파일을 다운로드합니다.
 
2. **실행 권한 부여**  
   쉘 스크립트에 실행 권한을 부여합니다:
   ```bash
   chmod +x docker_run_f1tenth_sim.sh
   ```

3. **스크립트 실행**  
   쉘 스크립트를 통해 컨테이너를 실행합니다:
   ```bash
   ./docker_run_f1tenth_sim.sh
   ```

--- 
Docker 컨테이너가 성공적으로 실행되면 터미널 프롬프트가 아래와 같이 표시됩니다:

```bash
root@12ff300395b4:/sim_ws#
```

- **의미**  
  - `root@12ff300395b4`: Docker 컨테이너의 ID를 나타냅니다.  
  - `/sim_ws#`: Docker 컨테이너 내부의 작업 디렉토리가 `/sim_ws`로 설정되어 있음을 나타냅니다.  
