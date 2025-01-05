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
