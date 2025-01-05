# **수동 Waypoint 생성기 사용**  

수동 입력 방식을 사용하여 웨이포인트를 처리하는 도구입니다. 이 문서는 환경 설정 및 스크립트를 실행하는 방법을 설명합니다.

---

## **사전 준비 사항**

스크립트를 실행하기 전에 아래 항목을 준비해야 합니다:  
- **Python 3.7**  
- **conda** (또는 다른 환경 관리 도구)

---

## **설치 방법**

다음 단계에 따라 환경을 설정하고 필수 라이브러리를 설치합니다.

### **1. 저장소 클론**
```bash
git clone https://github.com/jinkimh/wpgen_manual.git
cd wpgen_manual/
```
- GitHub 저장소를 복제하고 해당 디렉토리로 이동합니다.

---

### **2. Conda 환경 생성 및 활성화**
```bash
conda create -n wpgen python=3.7 -y
conda activate wpgen
```
- `wpgen`이라는 이름의 Python 3.7 기반 Conda 환경을 생성하고 활성화합니다.

---

### **3. 의존성 패키지 설치**
`requirements.txt` 파일을 사용하여 필요한 라이브러리를 설치합니다:
```bash
pip install -r requirements.txt
```
- `requirements.txt` 파일에 정의된 Python 패키지를 모두 설치합니다.

---

## **사용 방법**

### **맵 파일 준비**
스크립트를 실행하기 전에 아래 작업을 완료해야 합니다:
1. **SLAM Toolbox 출력 파일**: SLAM을 통해 생성된 맵 이미지와 `.yaml` 파일을 `map` 폴더에 저장합니다.
   - 예시:
     ```
     ./map/ict_3rd_floor.png
     ./map/ict_3rd_floor.yaml
     ```

---

### **스크립트 실행**
웨이포인트 생성 과정을 시작하려면 다음 명령어를 실행합니다:
```bash
python3 wpgen_manual.py ./map/ict_3rd_floor.png
```
- 맵 파일 경로를 인자로 입력합니다.

---

## **웨이포인트 생성 방법**

1. **맵 이미지 불러오기**:  
   스크립트를 실행하면 맵 이미지가 열립니다.

2. **웨이포인트 지정하기**:  
   맵 위를 클릭하여 원하는 경로를 따라 웨이포인트를 지정합니다.  
   - **팁**: 웨이포인트를 촘촘하게 배치하면 부드러운 주행 경로가 생성됩니다.

3. **웨이포인트 생성**:  
   웨이포인트 지정이 끝나면 **"r"** 키를 눌러 최종 웨이포인트를 생성합니다.

---

## **문제 해결**

### **의존성 누락**  
`ModuleNotFoundError` 오류가 발생할 경우 모든 라이브러리가 설치되었는지 확인합니다:
```bash
pip install -r requirements.txt
```

### **Conda 환경 문제**  
스크립트를 실행하기 전에 Conda 환경이 활성화되어 있는지 확인합니다:
```bash
conda activate wpgen
```

---

## **참고 사항**  

- Python 3.7 환경을 사용하여 호환성을 유지합니다.  
- 추가적인 커스터마이징이 필요하면 `wpgen_manual.py` 스크립트를 수정하여 기능을 변경할 수 있습니다.

---

## **라이선스**

이 프로젝트는 [MIT License](LICENSE) 하에 공개된 오픈 소스 프로젝트입니다.

---

즐거운 웨이포인트 생성 되세요! 🚗
