# 🧪 Odometry Tuning

F1Tenth 자율주행 차량의 주행 정확도를 높이기 위해서는 Odometry 관련 파라미터를 적절히 튜닝해야 합니다.  
본 실습에서는 **ERPM 게인**, **조향각 오프셋**, **조향각 게인**을 조정하여 직진 및 회전 정확도를 향상시키는 방법을 학습합니다.

> ✅ Source: [Mushr Odometry Tuning Guide (ROS1)](https://mushr.io/tutorials/tuning/)

---

## ⚙️ 설정 파일 위치

```bash
$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
````

### 주요 파라미터

```yaml
/**:
  ros__parameters:
    speed_to_erpm_gain: 4614.0
    speed_to_erpm_offset: 0.0
    steering_angle_to_servo_gain: -1.2135
    steering_angle_to_servo_offset: 0.5304
```

---

## ⚠️ vesc.yaml 수정 시 필수 절차

`vesc.yaml` 파일을 수정할 때마다 반드시 다음 절차를 **반드시 수행**해야 합니다:

### 🔄 파라미터 수정 후 재반영 절차

```bash
cd ~/f1tenth_ws

# 1. 재빌드
colcon build

# 2. ROS2 환경 설정
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 3. bringup 다시 실행
ros2 launch f1tenth_stack bringup_launch.py
```

> ⚠️ `colcon build` 없이 bringup만 다시 실행하면 **변경 사항이 적용되지 않습니다**.

---

## 🛞 1. 조향각 오프셋 (steering\_angle\_to\_servo\_offset)

### 🎯 목적

차량이 똑바로 주행하도록 기본 조향 위치(중립점)를 보정합니다.

### 🛠 튜닝 절차

1. 원격 조작 실행

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

2. 차량 직선 주행 후 방향 확인

3. 방향 편차에 따라 `steering_angle_to_servo_offset` 수정

   * 왼쪽 치우침 → 오프셋 증가
   * 오른쪽 치우침 → 오프셋 감소

4. 수정 후 반드시 다음 명령 실행

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

5. 반복하여 최적 값 찾기

> 📌 일반적인 값 범위: `0.4 ~ 0.6`

---

## ⚡ 2. ERPM 게인 (speed\_to\_erpm\_gain)

### 🎯 목적

속도 명령과 실제 이동 거리 간 오차를 줄입니다.

### 🛠 튜닝 절차

1. 줄자 또는 측정 도구 준비 (3m 이상)

2. 차량 위치 0점 정렬 후 bringup 실행

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

3. 거리 주행 후 `/odom` 주행 거리 측정

```bash
ros2 topic echo --no-arr /odom
```

4. 오차에 따라 게인 수정

   * 보고된 거리 < 실제 거리 → 게인 증가
   * 보고된 거리 > 실제 거리 → 게인 감소

5. 수정 후 다음 명령 실행

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

> 📌 권장 조정 단위: `500`
> 📌 일반적인 범위: `2000 ~ 5000`

---

### ❗ odom 값이 **음수**로 나올 때

일부 차량에서는 `/odom`의 x 값이 음수로 출력될 수 있습니다. 이 경우, 내부적으로 속도 계산이 반대로 되어 있음을 의미하므로 아래 코드를 수정해야 합니다.

### 🔧 수정 파일 위치

```bash
$HOME/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp
```

### ✏️ 수정 내용 (Line 102 부근)

**변경 전:**

```cpp
double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
```

**변경 후:**

```cpp
double current_speed = -(-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
```

> ✅ 속도를 양수로 보정하기 위해 `-()`로 한 번 더 감쌉니다.

수정 후 아래 명령어로 다시 빌드합니다:

```bash
cd ~/f1tenth_ws
colcon build
```

---

## 🌀 3. 조향각 게인 (steering\_angle\_to\_servo\_gain)

### 🎯 목적

정해진 회전 반경에 맞도록 조향 명령을 튜닝합니다.

### 🛠 튜닝 절차

1. 측정 테이프 위에 차량 정렬

2. 최대 조향각 명령 후 주행

3. 주행 반경 측정 (목표값: `1.722m`)

4. 오차에 따라 게인 조정

   * 도달하지 못하면 → 게인 감소
   * 지나치면 → 게인 증가

5. 수정 후 다음 실행

```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

> 📌 일반 범위: `1.1 ~ 1.3`

---

## 📑 요약

| 항목         | 튜닝 변수                            | 일반 범위        | 수정 후 해야 할 작업                 |
| ---------- | -------------------------------- | ------------ | ---------------------------- |
| 조향 오프셋     | `steering_angle_to_servo_offset` | 0.4 \~ 0.6   | ✅ colcon build + bringup 재실행 |
| ERPM 게인    | `speed_to_erpm_gain`             | 2000 \~ 5000 | ✅ colcon build + bringup 재실행 |
| 조향각 게인     | `steering_angle_to_servo_gain`   | 1.1 \~ 1.3   | ✅ colcon build + bringup 재실행 |
| odom 음수 보정 | `vesc_to_odom.cpp`의 speed 계산식 수정 | -            | ✅ colcon build               |

---

## 📄 저작권

```
MIT License

Copyright (c) 2025 Jin Kim

본 소프트웨어 및 관련 문서는 "있는 그대로(as is)"의 상태로 제공되며,  
다음 조건을 충족하는 한 누구나 무료로 사용, 복사, 수정, 병합, 게시, 배포,  
재라이선스 및 판매할 수 있습니다:

1. 위 저작권 고지와 본 허가 고지는 소프트웨어의 모든 복사본 또는  
   주요 부분에 포함되어야 합니다.

2. 본 소프트웨어는 상품성 또는 특정 목적에의 적합성에 대한  
   보증 없이 제공됩니다. 제작자는 소프트웨어와 관련하여 발생한  
   손해에 대해 책임지지 않습니다.

```

---


