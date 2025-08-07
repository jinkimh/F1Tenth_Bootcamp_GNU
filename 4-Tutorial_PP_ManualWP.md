# 수동 생성 웨이포인트(WP)를 시뮬레이터에서 Pure Pursuit와 함께 실행

본 문서는 사용자가 수동으로 생성한 `(x, y)` 웨이포인트를 기반으로,  
F1Tenth 시뮬레이터 내에서 Pure Pursuit 알고리즘을 실행하는 절차 및 코드 수정 사항을 안내합니다.

---

## 1. Pure Pursuit 노드 코드 수정

**파일 위치**:  
`/sim_ws/src/f1tenth-software-stack/pure_pursuit/scripts/pure_pursuit_node.py`

---

### 🔄 기존 Tutorial-2 방식과의 차이점

| 항목           | 수동 웨이포인트 방식         | Tutorial-2 방식             |
|----------------|-----------------------------|-----------------------------|
| 포맷 구성       | `(x, y)` 좌표만 포함         | `(x, y, speed, steering...)` 등 여러 열 |
| 사용 목적       | 단순 경로 주행               | 정밀한 속도 및 제어 경로 주행  |
| 데이터 형태     | 단순 CSV (2열)              | 복합 CSV (다중 열)          |

---

### 🗂 수동 웨이포인트 CSV 예시

```csv
x, y
1.0, 2.0
2.0, 3.5
3.2, 4.8
````

* 열 구성: `x`, `y` (헤더 포함)
* 쉼표 구분자 사용

---

## 2. 코드 수정 절차

### ✅ 1) 맵 이름 설정 (27번째 줄)

```python
self.map_name = 'levine_2nd'  # 사용할 맵 이름으로 수정
```

---

### ✅ 2) CSV 파일 로딩 방식 수정 (44–45번째 줄)

**변경 전:**

```python
csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=';', skiprows=0)
self.waypoints = csv_data[:, 1:3]
```

**변경 후:**

```python
csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=',', skiprows=1)
self.waypoints = csv_data[:, :]  # (x, y) 열 전체 사용
```

**변경 내용 요약:**

* `delimiter=';'` → `','` (쉼표 구분자)
* `skiprows=0` → `1` (헤더 무시)
* `[:, 1:3]` → `[:, :]` (모든 열 참조)

---

### ✅ 3) 속도 설정 방식 수정 (96번째 줄)

**변경 전:**

```python
self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * self.ref_speed[self.closest_index]
```

**변경 후:**

```python
self.drive_msg.drive.speed = 2.0  # 고정 속도 설정 (예: 2.0 m/s)
```

---

### ✅ 4) 속도 입력 관련 코드 제거 (49번째 줄)

**기존 코드:**

```python
# self.ref_speed = csv_data[:, 5] * 0.6
self.ref_speed = csv_data[:, 5]
```

**변경 후:**

```python
# self.ref_speed = csv_data[:, 5] * 0.6
# self.ref_speed = csv_data[:, 5]
```

* `ref_speed` 관련 내용을 주석 처리하여 사용하지 않도록 함

---

## 3. Pure Pursuit 실행

시뮬레이터 내에서 다음 순서로 Pure Pursuit 알고리즘을 실행합니다.

---

### 🖥️ (1) 시뮬레이터 브릿지 실행

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

---

### 🚗 (2) Pure Pursuit 노드 실행

```bash
/sim_ws$ source /opt/ros/foxy/setup.bash
/sim_ws$ source install/setup.bash
/sim_ws$ ros2 run pure_pursuit pure_pursuit_node.py
```

* 해당 노드는 `/csv_data/<map_name>.csv` 파일의 웨이포인트를 따라 차량을 주행시킴

---

## 4. 최종 점검 체크리스트 ✅

| 항목         | 확인사항                                                                           |
| ---------- | ------------------------------------------------------------------------------ |
| **CSV 경로** | `~/sim_ws/src/f1tenth-software-stack/csv_data/<map_name>.csv`에 위치              |
| **CSV 포맷** | `(x, y)` 두 열만 포함, 쉼표 구분자, 첫 줄은 헤더                                              |
| **코드 위치**  | `/sim_ws/src/f1tenth-software-stack/pure_pursuit/scripts/pure_pursuit_node.py` |
| **속도 설정**  | 고정 속도 `2.0` m/s 또는 상황에 맞게 조절                                                   |
| **노드 실행**  | `ros2 run pure_pursuit pure_pursuit_node.py` 명령으로 실행                           |

---

## 📄 저작권

```
© 2025 Jin Kim, Gyeongsang National University

본 문서는 F1Tenth 자율주행 플랫폼의 실험 및 교육용으로 제작되었으며,  
무단 복제 및 상업적 사용을 금지합니다.
```

---




