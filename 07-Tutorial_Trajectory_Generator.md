# Optimal Trajectory Generator를 이용, WP 생성

이 튜토리얼은 **(Optimal) Trajectory Generator**를 사용하여 레이싱 트랙의 최적 경로를 생성하는 방법을 안내합니다.

---

## 1. Recommended Environment
Python **3.8** 버전의 가상환경 사용을 권장합니다.  
아래 두 가지 방법 중 하나를 선택하세요.

### (1) Anaconda 사용

```bash
# 아나콘다 설치
bash ~/Downloads/Anaconda3-{version}-Linux-x86_64.sh
echo "export PATH=~/anaconda3/bin:~/anaconda3/condabin:$PATH" >> ~/.bashrc
source ~/.bashrc
conda config --set auto_activate_base False

# 가상환경 생성 및 활성화
conda create -n traj_generator python=3.8
conda activate traj_generator
```

### (2) venv 사용
```bash
python3 -m venv ./traj_generator
source ./traj_generator/bin/activate  # 가상환경 활성화
````



---

## 2. Installation

```bash
# 새 Git 저장소 초기화
git init

# Sparse Checkout 활성화
git config core.sparseCheckout true

# 원격 저장소 추가 (zzjun725 저장소)
git remote add -f origin https://github.com/zzjun725/f1tenth-racing-stack-ICRA22.git

# 필요한 디렉토리만 명시
echo trajectory_generator/* > .git/info/sparse-checkout

# main 브랜치에서 가져오기
git pull origin main

# 불필요한 Git 메타데이터 제거 (선택)
rm -rf .git

# 의존성 설치
cd trajectory_generator
pip install -r requirements.txt
 
# config 디렉터리 생성 및 params.yaml 다운로드
mkdir config && cd config
curl -L -O https://raw.githubusercontent.com/jinkimh/f1tenth-racing-stack-ICRA22/main/config/params.yaml

# `maps` 디렉토리 다운로드
cd ../ # config 폴더 밖으로
curl -L -o maps.zip https://github.com/jinkimh/f1tenth-racing-stack-ICRA22/archive/refs/heads/main.zip
unzip maps.zip 'f1tenth-racing-stack-ICRA22-main/maps/*'
mv f1tenth-racing-stack-ICRA22-main/maps ./maps
rm -rf f1tenth-racing-stack-ICRA22-main maps.zip

```

---

## 3. Configuration

### (1) 맵 데이터 추가

* `trajectory_generator/maps` 디렉터리를 생성한 후, **맵 이미지 파일**(`.png`, `.jpg`)과 **맵 메타데이터 파일**(`.yaml`)을 넣어주세요.

```plaintext
trajectory_generator/
 ├── config/
 │    └── params.yaml
 ├── maps/
 │    ├── mytrack.png
 │    └── mytrack.yaml
```

`mytrack.yaml` 예시:

```yaml
image: mytrack.png
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

### (2) params.yaml 수정

* `trajectory_generator/config/params.yaml` 파일에서 **맵 이름**과 **확장자**를 변경하세요.

```yaml
# params.yaml 예시
map_name: mytrack
map_img_ext: .png

# 경로 생성 파라미터 (필요 시 조정)
track_width: 0.8
vehicle_width: 0.3
smoothing_weight: 0.5
```

---

## 4. Execute

### (1) Centerline 생성

```bash
python3 ~/trajectory_generator/lane_generator.py
```

### (2) 최적 경로 생성

```bash
python3 ~/trajectory_generator/main_globaltraj.py
```

---

## 5. Troubleshooting

### (1) quadprog 관련 오류

```text
ImportError: /home/mina/anaconda3/envs/traj_generator/lib/python3.8/site-packages/quadprog.cpython-38-x86_64-linux-gnu.so: undefined symbol: _Z7qpgen2_PdS_PiS0_S_S_S_S_S_S0_S0_S0_S0_S0_S0_S_S0_
```

➡ 해결 방법:

```bash
conda install -c conda-forge quadprog=0.1.7
```

---

## 6. Parameter Tuning

* 실행 시 에러창이 뜨거나 커널에 에러 메시지가 발생하면
  `params/racecar.ini` 파일의 파라미터를 수정하세요.

---

## 7. 디렉터리 구조 예시

```plaintext
trajectory_generator/
 ├── config/
 │    └── params.yaml
 ├── maps/
 │    ├── mytrack.png
 │    └── mytrack.yaml
 ├── requirements.txt
 ├── lane_generator.py
 ├── main_globaltraj.py
```

---

## 📌 참고

* 본 튜토리얼은 **F1Tenth Bootcamp** 실습 환경에서 사용됩니다.
* Python 3.8 환경에서 테스트되었습니다.
* 맵 이미지는 OpenCV 또는 ROS map\_server로 생성된 맵을 사용할 수 있습니다.


 
# Optimal Trajectory Generator — Parameter Manual

이 문서는 `main_globaltraj.py` 실행 시 조정할 수 있는 주요 파라미터와 사용법을 설명합니다.

```
1. **입력 경로 및 차량 파라미터 설정** (`file_paths`, `veh_params_file`)
2. **디버그 및 플로팅 옵션** (`debug`, `plot_opts`)
3. **트랙 가져오기 및 변환 옵션** (`imp_opts`)
4. **최적화 알고리즘 선택 및 세부 옵션** (`opt_type`, `mintime_opts`)
5. **랩타임 계산 및 결과 저장 옵션** (`lap_time_mat_opts`)
```
---

## 1. Vehicle Parameter File
```python
file_paths = {"veh_params_file": "racecar.ini"}
````

* 차량의 물리 파라미터를 정의한 `.ini` 파일 경로
* `params/` 폴더 내 파일을 지정
* 예: `racecar.ini`, `f1tenth.ini` 등

---

## 2. Debug & Plot Options

```python
debug = True
plot_opts = {
    "mincurv_curv_lin": False,  # 최소 곡률 선형화 결과 플롯
    "raceline": True,           # 최적화 경로 표시
    "imported_bounds": False,   # 원본 트랙 경계 표시
    "raceline_curv": True,      # 최적화 경로의 곡률 프로필 표시
    "racetraj_vel": True,       # 속도 프로필 표시
    "racetraj_vel_3d": False,   # 3D 속도 프로필 표시
    "racetraj_vel_3d_stepsize": 1.0,
    "spline_normals": False,    # 스플라인 법선 벡터 표시
    "mintime_plots": False      # 시간 최소화 최적화 시 내부 변수 플롯
}
```

* **`debug`** : `True` 시 상세 로그 출력
* **`plot_opts`** : 실행 후 표시할 그래프 종류 선택

---

## 3. Track Import Options (`imp_opts`)

```python
imp_opts = {
    "flip_imp_track": False,           # 트랙 주행 방향 반전
    "set_new_start": False,            # 새로운 시작점 설정
    "new_start": np.array([0.0, -47.0]), # 시작점 좌표
    "min_track_width": None,           # 최소 트랙 폭 강제 적용 (m)
    "num_laps": 1                      # 랩 수
}
```

* **`flip_imp_track`** : 주행 방향 반대로 변경
* **`set_new_start` + `new_start`** : 원하는 좌표에서 시작
* **`min_track_width`** : 좁은 구간을 최소 폭으로 보정
* **`num_laps`** : 주행 랩 수 (전력 제한 시 유의)

---

## 4. Optimization Type (`opt_type`)

```python
# 선택 가능 모드
opt_type = 'mincurv_iqp'
```

* `'shortest_path'` : 최단 거리 경로
* `'mincurv'` : 최소 곡률 경로 (단일 실행)
* `'mincurv_iqp'` : 최소 곡률 경로 (반복 개선)
* `'mintime'` : 시간 최소 경로 (속도 프로필 포함)

---

## 5. Mintime Options (`mintime_opts`)

```python
mintime_opts = {
    "tpadata": None,                  # 마찰 맵 데이터 파일
    "warm_start": False,              # IPOPT 워밍 스타트
    "var_friction": None,             # None / "linear" / "gauss"
    "reopt_mintime_solution": False,  # mintime → mincurv 재최적화
    "recalc_vel_profile_by_tph": False # ggv 기반 속도 재계산
}
```

* **`tpadata`** : 마찰 맵 JSON 파일 지정
* **`warm_start`** : 이전 결과로 초기값 설정
* **`var_friction`** : 가변 마찰계수 적용
* **`reopt_mintime_solution`** : mintime 결과를 곡률 최적화로 재정제
* **`recalc_vel_profile_by_tph`** : ggv 다이어그램 기반 속도 재계산

---

## 6. Lap Time Matrix Options (`lap_time_mat_opts`)

```python
lap_time_mat_opts = {
    "use_lap_time_mat": False,        # 랩타임 매트릭스 계산 여부
    "gg_scale_range": [0.3, 1.0],     # ggv 스케일 범위
    "gg_scale_stepsize": 0.05,        # ggv 스케일 간격
    "top_speed_range": [100.0, 150.0],# 최고 속도 범위 [km/h]
    "top_speed_stepsize": 5.0,        # 최고 속도 간격
    "file": "lap_time_matrix.csv"     # 저장 파일명
}
```

* 다양한 ggv 스케일과 최고 속도 조합별 예상 랩타임 계산
* `use_lap_time_mat=True`로 활성화

---

## 7. Vehicle Dynamics Parameters

* `.ini` 파일에서 불러오는 값:

  * **`veh_params`** : 차량 폭, 길이, 질량, 최대 속도 등
  * **`ggv_file`** : ggv 다이어그램 데이터
  * **`stepsize_opts`** : 샘플링 간격
  * **`reg_smooth_opts`** : 스플라인 보정 옵션
  * **`optim_opts`** : 각 최적화 모드별 내부 파라미터

---

## 8. Parameter Tuning Tips

* **좁은 코너에서 경로 이탈 발생** → `veh_params["curvlim"]` 감소
* **경로가 너무 중앙에 치우침** → `optim_opts["width_opt"]` 조정
* **속도 프로필이 비현실적으로 높음** → `v_max` 낮추기, `dragcoeff` 증가
* **랩타임 계산 속도 향상** → `stepsize_opts`의 간격 확대

---

## 📌 참고

* 파라미터 변경 후 실행 전, `outputs/` 디렉터리를 삭제하면 깨끗한 상태에서 재계산됩니다.
* `mintime` 모드는 CPU 성능과 IPOPT 설정에 따라 시간이 많이 소요될 수 있습니다.
---

## 📄 저작권

```
© 2025 Jin Kim, Gyeongsang National University

본 문서는 F1Tenth 자율주행 플랫폼의 실험 및 교육용으로 제작되었으며,  
무단 복제 및 상업적 사용을 금지합니다.
```

---

