
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

## 📌 조정할 파라미터 3종

| 항목                               | 설명                    |
| -------------------------------- | --------------------- |
| `speed_to_erpm_gain`             | 속도(m/s)를 ERPM으로 변환    |
| `steering_angle_to_servo_gain`   | 조향각(rad)을 서보 입력값으로 변환 |
| `steering_angle_to_servo_offset` | 조향 중립 위치 보정 (직진 보정)   |

---

## 🛞 1. 조향각 오프셋 (steering\_angle\_to\_servo\_offset)

### 🎯 목적

차량이 똑바로 주행하도록 기본 조향 위치(중립점)를 보정합니다.

### 🛠 튜닝 절차

1. **원격 조작 실행**

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

2. 차량을 직선으로 주행 (3\~4m). 약간의 편차는 허용됩니다.

3. 방향이 왼쪽으로 치우치면 → `오프셋 값을 증가`
   방향이 오른쪽으로 치우치면 → `오프셋 값을 감소`

4. `vesc.yaml` 파일에서 `steering_angle_to_servo_offset` 값 수정 후 다시 실행

5. 만족할 때까지 반복

> 📌 보통 값의 범위: `0.4 ~ 0.6`

---

## ⚡ 2. 속도 → ERPM 게인 (speed\_to\_erpm\_gain)

### 🎯 목적

속도 명령과 실제 이동 거리 간 오차를 줄입니다.

### 🛠 튜닝 준비

* 줄자 또는 측정 테이프 (3m 이상)
* 차량을 0점에 맞추고 뒤쪽 바퀴 기준으로 측정

### 🛠 튜닝 절차

1. **원격 조작 실행**

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

2. 다른 터미널에서 odometry 값 확인:

```bash
ros2 topic echo --no-arr /odom
```

3. 차량을 약 2.5m 주행시키고 `/odom`에 표시된 거리와 비교

4. **오차 계산**:

   * 보고된 값이 실제보다 크면 → 게인 감소
   * 보고된 값이 실제보다 작으면 → 게인 증가

5. `vesc.yaml`의 `speed_to_erpm_gain` 값 수정 후 반복

> 📌 초기에는 `500` 단위로 조정하며, 일반적으로 `2000 ~ 5000` 사이가 적절

### ❗ odom 값이 음수일 경우

```cpp
// 파일 위치:
$HOME/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp

// 102번째 줄 수정:
double current_speed = -(-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
```

---

## 🌀 3. 조향각 게인 (steering\_angle\_to\_servo\_gain)

### 🎯 목적

원하는 회전 반경에 맞는 조향각을 정확히 설정

### 🛠 튜닝 준비

* 줄자 약 2.5m 이상
* 시작점에 차량을 정렬

### 🛠 튜닝 절차

1. **원격 조작 실행**

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

2. 한쪽 방향으로 최대 조향각 명령 (예: 왼쪽)

3. 차량을 곡선 주행시켜 다시 줄자 위에 도달하게 함

4. 주행 반경 거리 측정

5. 목표 거리: `1.722m` (67.79인치)

6. 측정값이 목표보다 크면 → 게인 증가
   측정값이 목표보다 작으면 → 게인 감소

7. `vesc.yaml`에서 `steering_angle_to_servo_gain` 조정 후 반복

> 📌 일반적인 범위: `1.1 ~ 1.3`

---

## 📑 요약

| 항목      | 튜닝 변수                            | 기준 값 범위      |
| ------- | -------------------------------- | ------------ |
| 조향 오프셋  | `steering_angle_to_servo_offset` | 0.4 \~ 0.6   |
| ERPM 게인 | `speed_to_erpm_gain`             | 2000 \~ 5000 |
| 조향 게인   | `steering_angle_to_servo_gain`   | 1.1 \~ 1.3   |

---

## 📄 저작권

```
© 2025 Jin Kim, Gyeongsang National University

본 문서는 F1Tenth 자율주행 플랫폼의 실험 및 교육용으로 제작되었으며,  
무단 복제 및 상업적 사용을 금지합니다.
```

```

---
