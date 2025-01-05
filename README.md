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

F1Tenth 시뮬레이터를 실행하기 위해 Docker 이미지를 생성합니다. 아래는 기본적인 작업 디렉토리 설정 및 Docker 이미지 빌드 과정입니다.

---

### **1. 작업 디렉토리 생성 및 이동**  
```bash
$ mkdir -p ~/f1tenth_ws  
$ cd ~/f1tenth_ws  
```
- `~/f1tenth_ws`: 시뮬레이터 관련 파일을 저장할 작업 디렉토리 생성 및 이동  

---

### **2. Git 저장소 클론**  
```bash
$ git clone https://github.com/jinkimh/f1tenth_gym_ros_gnu  
```
- F1Tenth 시뮬레이터 GitHub 저장소를 클론하여 로컬에 저장  
  - 해당 저장소에는 ROS2 기반 시뮬레이터 소스 코드와 Docker 관련 설정 파일이 포함되어 있습니다.

---

### **3. Docker 빌드 준비 및 실행**  
```bash
$ cd src/f1tenth_gym_ros  
$ docker build -t f1tenth_gym_ros -f Dockerfile .  
```
- `src/f1tenth_gym_ros`: 시뮬레이터 ROS 패키지 디렉토리로 이동  
- Dockerfile을 이용해 Docker 이미지를 빌드합니다.  
  - `-t f1tenth_gym_ros`: 빌드된 이미지에 태그를 지정 (이미지 이름: `f1tenth_gym_ros`)  
  - `-f Dockerfile`: Dockerfile을 사용해 이미지 빌드  

---
