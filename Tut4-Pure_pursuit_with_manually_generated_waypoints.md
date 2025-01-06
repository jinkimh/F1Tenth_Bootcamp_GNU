
## **매뉴얼: 수동 Waypoint 실행 방법**

---

### **1. 개요**  
본 장에서는 **수동으로 생성된 웨이포인트**를 사용하여 시뮬레이션을 실행하는 방법과 필요한 코드 수정 사항을 설명합니다.  
- **파일 위치**: `/sim_ws/src/f1tenth-software-stack/pure_pursuit/scripts/pure_pursuit_node.py`  

---

### **2. 주요 차이점**
| **항목**           | **수동 웨이포인트** | **Tut2 웨이포인트** |
|-------------------|-------------------|-------------------|
| **포맷 구성**       | `(x, y)` 좌표만 포함 | `(x, y)` 외 속도, 제어값 포함 |
| **사용 목적**       | 단순 경로 실행       | 정밀한 경로 및 속도 제어 |
| **데이터 형태**     | 단순 CSV (2열)       | 복합 CSV (여러 열) |

---

### **3. 실행 준비 단계**  
- **수동 웨이포인트 CSV 파일**의 열 구성:
  - 열 구조: `(x, y)` 두 개의 열만 포함된 CSV 파일
  - 예시:
    ```
    x, y
    1.0, 2.0
    2.0, 3.5
    3.2, 4.8
    ```

---

### **4. 코드 수정 절차**

#### **1) 맵 파일 이름 변경  ** 27번쨰 줄**
``` python
self.map_name = 'levine_2nd' <- 새로운 맵 파일 이름 
```


### **2) CSV 파일 로드 코드 변경**  
**위치**: 약 **44~45번째 줄**  
**변경 전**:
```python
csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=';', skiprows=0)  # csv data
self.waypoints = csv_data[:, 1:3]  # first row is indices
```
**변경 후**:
```python
csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=',', skiprows=1)  # csv data
self.waypoints = csv_data[:, :]  # (x, y)만 포함된 웨이포인트 데이터
```
**수정 내용**:
- `delimiter=';'` → `delimiter=','`: 구분자를 세미콜론에서 쉼표로 변경  
- `skiprows=0` → `skiprows=1`: 첫 번째 행을 건너뛰어 헤더를 무시  
- `[:, 1:3]` → `[:, :]`: 모든 열을 참조하도록 수정  

---

### **3) 속도 설정 코드 변경**  
**위치**: **96번째 줄**  
**변경 전**:
```python
self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * self.ref_speed[self.closest_index]
```
**변경 후**:
```python
self.drive_msg.drive.speed = 2.0  # 고정 속도 2.0 m/s
```
**수정 내용**:
- `self.ref_speed` 배열에서 속도 값을 참조하던 방식을 제거  
- 고정된 속도 `2.0 m/s`로 설정  

---

### **4) 속도 입력값 무시**  
**위치**: **49번째 줄**  
**변경 전**:
```python
# self.ref_speed = csv_data[:, 5] * 0.6  # max speed for levine 2nd - real is 2m/s
self.ref_speed = csv_data[:, 5]  # max speed - sim is 10m/s
```
**변경 후**:
```python
# self.ref_speed = csv_data[:, 5] * 0.6  # max speed for levine 2nd - real is 2m/s
# self.ref_speed = csv_data[:, 5]  # max speed - sim is 10m/s
```
**수정 내용**:
- self.ref_speed = csv_data[:, 5]  # max speed - sim is 10m/s -> 주석처리 

---

### **5) 최종 점검 항목**  
**파일 위치**: `/sim_ws/src/f1tenth-software-stack/pure_pursuit/scripts/pure_pursuit_node.py`  

