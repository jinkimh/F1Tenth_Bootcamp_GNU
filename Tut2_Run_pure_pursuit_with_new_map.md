## **1장. Pure Pursuit 알고리즘 실행하기**

현재 시뮬레이터 환경에서 Pure Pursuit 알고리즘을 사용하여 차량을 주행시켜봅니다. 아래는 Pure Pursuit 알고리즘 구동을 위한 단계입니다.

---

### **Step 1: ROS2 환경 설정 및 빌드**

1. **ROS2 Foxy 환경 설정**  
   시뮬레이터 작업 디렉토리(`/sim_ws`)에서 다음 명령어를 실행합니다:
   ```bash
   /sim_ws$ source /opt/ros/foxy/setup.bash  # ROS2 Foxy 환경 설정
   /sim_ws$ colcon build                     # 작업 공간 빌드
   /sim_ws$ source install/setup.bash        # 빌드된 ROS2 패키지 설정
   ```
   - `setup.bash`: ROS2 설정 파일을 불러옵니다.
   - `colcon build`: ROS2 워크스페이스를 빌드합니다.

---

### **Step 2: Pure Pursuit 실행**

2. **Pure Pursuit 노드 실행**  
   새로운 터미널을 열어 Docker 컨테이너에 접속한 후, 아래 명령어를 입력합니다:
   ```bash
   /sim_ws$ source /opt/ros/foxy/setup.bash  # ROS2 환경 설정
   /sim_ws$ source install/setup.bash        # ROS2 워크스페이스 설정
   /sim_ws$ ros2 run wall_follow wall_follow_node.py
   ```
   - `wall_follow_node.py`: Pure Pursuit 알고리즘을 실행하는 노드입니다.

---

### **중요 파일 설명**

F1Tenth 시뮬레이터에서 자동차가 주행할 맵과 웨이포인트 설정에 필요한 주요 파일과 디렉토리 구조는 아래와 같습니다.

---

### **1. 지도 파일 (Map Files)**

#### **맵 구성 파일**  
- **맵 파일 확장자:** `.png`, `.pgm`, `.yaml`  
- 맵 파일은 **Slam Toolbox**를 사용하여 주행 중 자동차의 센서 데이터를 기반으로 생성됩니다.

- **주요 파일 설명**:  
  - `map_xxx.png`: 주행 경로 및 장애물 정보가 포함된 스캔된 맵 이미지  
  - `map_xxx.yaml`: 맵의 기본 좌표 및 메타데이터 파일 (해상도, 원점 좌표 등)  

#### **맵 파일 저장 경로**  
```bash
src/f1tenth_gym_ros/maps/
```
- 생성된 지도 파일은 `src/f1tenth_gym_ros/maps` 디렉토리에 저장됩니다.

---

### **2. 웨이포인트 파일 (Waypoint Files)**

#### **웨이포인트 파일 설명**  
- **파일 확장자:** `.csv`  
- 웨이포인트 파일은 차량이 따라가야 할 좌표 정보(경로)를 담고 있으며, Pure Pursuit 알고리즘의 필수 입력 파일입니다.
  
- **웨이포인트 파일 예시:**  
  - `map_xxx.csv`: 맵과 매칭되는 웨이포인트 데이터 파일

#### **웨이포인트 파일 저장 경로**  
```bash
src/f1tenth-software-stack/csv_data/
```
- 웨이포인트 파일은 `src/f1tenth-software-stack/csv_data/` 디렉토리에 저장되어야 합니다.  
  - 웨이포인트 파일과 맵 파일의 이름을 일관성 있게 지정하여 맵과 연동되도록 설정해야 합니다.

### **웨이포인트 수동 생성 매뉴얼**

웨이포인트를 수동으로 생성하는 방법에 대한 자세한 설명은 다음 링크를 참고해 주세요:

[웨이포인트 수동 생성 방법 매뉴얼](https://github.com/jinkimh/f1tenth_bootcamp/blob/main/Tut3-Manual_creation_wp.md)

---

### **3. 새로운 맵을 사용한 Pure Pursuit 실행 방법**

- 맵 파일과 웨이포인트 파일이 준비되었을 때:
  1. 새로운 맵 파일을 `src/f1tenth_gym_ros/maps`에 추가  
  2. 해당 맵에 맞는 웨이포인트 파일을 `csv_data` 폴더에 저장  
  3. `gym_bridge_launch.py`를 사용하여 Pure Pursuit 알고리즘을 실행하면, 해당 맵에서 차량이 지정된 경로를 따라 주행하게 됩니다.

---

### **결론**  
맵과 웨이포인트 파일을 준비하여 자신만의 주행 경로를 설정하면, 다양한 경로와 장애물을 실험할 수 있습니다. 이를 통해 Pure Pursuit 알고리즘의 성능을 평가하고 최적화할 수 있습니다.
