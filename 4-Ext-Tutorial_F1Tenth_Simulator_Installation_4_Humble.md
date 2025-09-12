# F1TENTH 시뮬레이터 설치 매뉴얼

(ROS 2 **Humble**, Python **3.10**, **gym 0.19.0** 고정)

> 핵심 원칙
>
> 1. **venv(3.10)** 내부에서만 pip/colcon 사용
> 2. **pip<24.1 / setuptools<60 / wheel<0.40** 로 고정
> 3. **gym==0.19.0** + `numpy<2`, `cloudpickle<2`, `pyglet<=1.5.27`
> 4. **venv의 colcon**으로 빌드하고 **파이썬 경로 3종** 강제 지정
> 5. pip 패키지는 **venv에 직접 설치**, rosdep에는 **pip 설치 금지** 옵션

---

## 0) 환경 초기화

```bash
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO   # humble
```

## 1) Python 3.10 venv

```bash
mkdir -p ~/.venvs
python3.10 -m venv ~/.venvs/f1tenth_humble310
source ~/.venvs/f1tenth_humble310/bin/activate
python -V    # 3.10.x
```

## 2) pip/setuptools/wheel “잠금”

```bash
python -m pip install --force-reinstall "pip<24.1" "setuptools<60" "wheel<0.40"
# 예: pip 24.0 / setuptools 59.8.0 / wheel 0.38.4
```

## 3) 필수 파이썬 패키지 (venv에 직접)

```bash
pip install -U pyyaml transforms3d "numpy<2" "cloudpickle<2" "pyglet<=1.5.27"
```

## 4) **gym 0.19.0** 설치 (메타데이터 이슈 회피 옵션)

```bash
pip install --no-build-isolation --no-use-pep517 "gym==0.19.0"
```

### 설치 확인

```bash
python - <<'PY'
import sys, gym, yaml, transforms3d
print("py:", sys.executable)
print("gym:", gym.__version__, "->", gym.__file__)
print("yaml:", yaml.__version__)
print("t3d:", transforms3d.__version__)
PY
# 모두 ~/.venvs/f1tenth_humble310/ 아래 경로여야 정상
```

## 5) f1tenth\_gym 설치

```bash
git clone https://github.com/f1tenth/f1tenth_gym ~/f1tenth_gym
cd ~/f1tenth_gym
pip install -e .
```

## 6) 워크스페이스 & f1tenth\_gym\_ros

```bash
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros
```

## 7) 맵 경로 설정(sim.yaml)

`~/sim_ws/src/f1tenth_gym_ros/config/sim.yaml` 수정:

```yaml
map_path: "/home/<USER>/sim_ws/src/f1tenth_gym_ros/maps/levine"
```

## 8) rosdep (pip 시스템 설치 금지)

```bash
cd ~/sim_ws
rosdep update || true
rosdep install -i --from-path src --rosdistro humble -y \
  --as-root pip:false \
  --skip-keys=transforms3d
```

## 9) **venv의 colcon**으로 클린 빌드 (3.10 파이썬 강제)

```bash
# venv에 colcon 설치(툴체인 혼선 방지)
pip install -U colcon-common-extensions empy pybind11 catkin_pkg
which colcon   # ~/.venvs/f1tenth_humble310/bin/colcon 여야 안전

cd ~/sim_ws
rm -rf build/ install/ log/
find . -name CMakeCache.txt -delete

export PY310=$(which python)
~/.venvs/f1tenth_humble310/bin/colcon build --symlink-install --cmake-force-configure \
  --event-handlers console_direct+ \
  --cmake-args \
    -DPython3_EXECUTABLE=$PY310 \
    -DPYTHON_EXECUTABLE=$PY310 \
    -DAMENT_PYTHON_EXECUTABLE=$PY310 \
    -DBUILD_TESTING=OFF

source install/setup.bash
```

## 10) 실행

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py \
  map:=$(ros2 pkg prefix f1tenth_gym_ros)/share/f1tenth_gym_ros/maps/levine.yaml
```

---

## ✅ 자가진단 3종

```bash
# 1) gym이 venv에서 보이는가?
python -c "import sys,gym; print(sys.executable, gym.__version__, gym.__file__)"

# 2) 설치 산출물에 python3.10 디렉터리 존재(Humble이면 정상)
find install -type d -name "python3.10"

# 3) 실행 래퍼 shebang이 venv 파이썬인가?
head -n1 install/f1tenth_gym_ros/lib/f1tenth_gym_ros/gym_bridge
# => #!/home/<USER>/.venvs/f1tenth_humble310/bin/python
```

---

## 🔧 문제 해결 가이드

* **pip 경고(“launch-ros/ament-… requires pyyaml”)**
  → `pip install -U pyyaml` (이미 3단계에서 해결됨)

* **gym 설치 실패(메타데이터/`extras_require`)**
  → 2단계 버전 고정 유지 + 4단계 옵션

  ```bash
  python -m pip install --force-reinstall "pip<24.1" "setuptools<60" "wheel<0.40"
  pip install --no-build-isolation --no-use-pep517 "gym==0.19.0"
  ```

* **rosdep가 sudo pip3로 시스템에 transforms3d 설치 시도**
  → 8단계처럼
  `--as-root pip:false --skip-keys=transforms3d`
  (필요한 pip 패키지는 venv에 직접 설치)

* **`ModuleNotFoundError: gym`**
  → 9단계에서 **venv colcon** 사용 + **3개 -D 옵션** + **--cmake-force-configure**로 다시 빌드.
  → 당장 띄워야 할 응급처치:

  ```bash
  VENV_PY=$(which python)
  sed -i "1 s|^#!.*|#! $VENV_PY|" ~/sim_ws/install/f1tenth_gym_ros/lib/f1tenth_gym_ros/gym_bridge
  VENV_SITE=$(python -c 'import site; print([p for p in site.getsitepackages() if "/python3.10/" in p][0])')
  export PYTHONPATH="$VENV_SITE:$PYTHONPATH"
  ```

* **RViz GLSL 경고 (`active samplers ...`)**
  → 기능 영향 적음. 필요 시 임시 회피:

  ```bash
  LIBGL_ALWAYS_SOFTWARE=1 ros2 launch f1tenth_gym_ros gym_bridge_launch.py ...
  ```

---

## (부록) 만약 `gym==0.21`로 가고 싶다면?

* `env.reset(...)` 인자/반환 규약이 달라 **소스 수정**이 필요합니다(예: `env.unwrapped.reset(...)` 사용 등).
* 이번 매뉴얼은 **수정 없이 바로 동작하는 `gym==0.19.0`** 조합을 공식 해법으로 채택했습니다.
