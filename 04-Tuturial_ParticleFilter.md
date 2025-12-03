# F1TENTH Particle Filter 설치 및 실행 

이 문서는 F1TENTH 플랫폼에서 Particle Filter를 설치하고 실행하는 방법을 설명합니다.  
옵션으로 `range_libc`를 사용하여 빠른 레이저 레인지 모델을 사용할 수도 있습니다.

---

## 1. (선택 사항) range_libc 설치

보다 빠른 레이저 센서 모델을 사용하려면 `range_libc`를 설치합니다.

```bash
cd ~
git clone https://github.com/f1tenth/range_libc.git
cd range_libc/pywrapper
sudo WITH_CUDA=ON python setup.py install
````

> ⚠️ CUDA가 없는 경우 `WITH_CUDA=OFF`로 설정하세요.

---

## 2. Particle Filter 패키지 설치

```bash
# 소스 디렉토리로 이동 후 클론
cd ~/f1tenth_ws/src
git clone https://github.com/f1tenth/particle_filter.git

# 의존성 설치
cd ~/f1tenth_ws
rosdep install -r --from-paths src --ignore-src --rosdistro foxy -y

# 워크스페이스 빌드
colcon build
source install/setup.bash
```

---

## 3. Particle Filter 실행

### 3.1. Teleop 실행 (다른 터미널)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3.2. Particle Filter 노드 실행

```bash
ros2 launch particle_filter localize_launch.py
```

---

## 4. RViz2에서 시각화

```bash
rviz2
```

### 추가 설정:

* **Map 표시**:

  * `Add` → `By topic` → `/map`
  * `Durability Policy`: `Transient Local` 로 설정

* **Localization 위치 표시**:

  * `/pf/viz/inferred_pose`

* **입자 시각화 (선택)**:

  * `/pf/viz/particles`

---

## 5. 퍼블리시 주기 확인

```bash
# /pf/viz/inferred_pose 주파수가 최소 30Hz 이상인지 확인
ros2 topic hz /pf/viz/inferred_pose
```

---

## 6. 사용 맵 변경

1. 맵 이미지(`.pgm`, `.yaml`)를 `particle_filter/maps` 폴더에 복사
2. `particle_filter/config/localize.yaml` 파일에서 사용하고자 하는 맵 파일 이름으로 수정

```yaml
# 예시
map_yaml_file: maps/my_custom_map.yaml
```

---

## 7. 초기 위치 설정

RViz 상단 툴바에서 **"2D Pose Estimate"** 버튼을 눌러 초기 위치를 수동으로 설정합니다.
이는 Particle Filter의 위치 초기화를 위한 것입니다.

---

## 요약 명령어

```bash
# 1. teleop 실행
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 2. Particle Filter 실행
ros2 launch particle_filter localize_launch.py

# 3. RViz 시각화 실행
rviz2
```

---

## 참고 사항

* ROS2 Foxy 버전에 맞춰 설치되었습니다.
* RViz 설정이 누락될 경우 Localization 결과가 보이지 않을 수 있으니 Topic 및 Durability 설정을 꼭 확인하세요.

---

## 📄 저작권

```
© 2025 Jin Kim, Gyeongsang National University

본 문서는 F1Tenth 자율주행 플랫폼의 실험 및 교육용으로 제작되었으며,  
무단 복제 및 상업적 사용을 금지합니다.
```

---

