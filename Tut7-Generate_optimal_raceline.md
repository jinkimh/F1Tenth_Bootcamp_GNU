# F1Tenth Bootcamp — Optimal Trajectory Generator Tutorial

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
# Sparse Checkout으로 필요한 디렉터리만 가져오기
git init
git config core.sparseCheckout true
git remote add -f origin https://github.com/zzjun725/f1tenth-racing-stack-ICRA22.git
echo trajectory_generator/* > .git/info/sparse-checkout
git pull origin main
rm -rf .gita

# 의존성 설치
cd trajectory_generator
pip install -r requirements.txt

# config 디렉터리 생성 및 params.yaml 다운로드
mkdir config && cd config
curl -L -O https://github.com/zzjun725/f1tenth-racing-stack-ICRA22/raw/refs/heads/main/config/params.yaml
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

```
