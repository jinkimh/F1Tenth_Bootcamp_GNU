아래는 **전체 내용을 매끄럽게 정리하여 다시 작성한 최종 Lab-12 과제 안내문(한국어 버전)**입니다.
중복 요소를 제거하고, 흐름에 맞게 재배치하고, 표현을 다듬어 **공식 교과용 과제 공지** 형태로 완성했습니다.

---

# 🏁 **Lab-12 Final Race: 최적 경로 생성 → Pure Pursuit → Time-Trial 자율주행 레이스**

---

## 🎯 **1. Lab-12 과제 목표**

Lab-12의 목표는 **Lab-11에서 구축한 기본 자율주행 파이프라인을 고성능 레이스 수준으로 확장하는 것**입니다.

Lab-11에서 구현한
**SLAM → Particle Filter → Manual Waypoint → Pure Pursuit**
기반의 기본 자율주행을 바탕으로,

Lab-12에서는 다음을 수행합니다:

1. **최적 경로(Optimal Trajectory) 생성**
2. **최적 경로 기반 Pure Pursuit 고속 자율주행**
3. **Time-Trial 방식의 실제 레이스 참여**

최종 평가는 **발표자료 + 최종 주행 성능(Time Trial)** 을 기반으로 이루어집니다.

---

# 🧭 **2. Final Race 수행 절차**

Lab-12는 아래의 **두 개의 기술 과제(Task 1~2) + 최종 레이스(Task 3)**로 구성됩니다.
(※ Lab-11 완료를 전제로 함)

---

## 1️⃣ **Task 1 — 최적 경로 생성 (Optimal Trajectory Generator)**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/07-Tutorial_Trajectory_Generator(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/07-Tutorial_Trajectory_Generator%28English%29.md)

### 📌 **Task 설명**

**Optimal Trajectory Generator**를 활용하여 다음을 수행합니다:

* Lab-11에서 생성한 트랙 경계(boundary) 또는 수동 waypoint 파일 불러오기
* `wp_gen.py` 실행하여 **최적 레이싱 라인(racing line)** 생성
* 생성된 최적 경로의 특징:
  ✔ 코너를 안쪽으로 부드럽게 공략
  ✔ 트랙 폭 전체 활용
  ✔ 조향 진동(oscillation) 최소화
  ✔ 더 높은 주행 속도 가능
* 출력 파일은 **CSV waypoint 파일** 형태이며 Task 2에서 사용됩니다.

---

## 2️⃣ **Task 2 — 최적 경로 기반 Pure Pursuit 자율주행**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/08-Tutorial_PP_OptWP(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/08-Tutorial_PP_OptWP%28English%29.md)

### 📌 **Task 설명**

* Task 1에서 생성한 **최적 경로 waypoint**를 Pure Pursuit 노드 입력으로 사용
* 고성능 주행을 위해 다음 파라미터 직접 튜닝:

  * Lookahead distance
  * Max speed
  * Steering angle limit
  * (필요 시) 추가 튜닝: dynamic lookahead, gain 조정 등
* 트랙에서 주행 안정성을 검증하여 고속에서도 흔들림 없는 성능 확보
* 선택 사항: **수동 WP vs. 최적 WP 비교 실험**

→ 이 Task의 핵심 목표는 **속도 + 안정성의 균형을 갖춘 고성능 자율주행 구현**입니다.

---

## 3️⃣ **Task 3 — Final Time-Trial Autonomous Race**

### 🏎️ **레이스 형식**

* **100% 완전 자율주행**
* 레이스 중 **어떠한 수동 조작 금지**
* 성적은 **랩타임 + 안정성 + 기술적 완성도**로 평가

### 🏟️ **레이스 규칙**

* 팀당 **2회 공식 주행 기회 제공**
* 각 시도 구성:

  1. 자동 워밍업 랩(타임 미측정)
  2. **연속 3랩 타임 측정**
* 두 시도 중 **가장 빠른 랩타임**을 기록으로 인정

### ⚠️ **충돌 규정**

* 충돌 1회: +5초 패널티
* 충돌 3회: 해당 시도 무효

### 🥇 **우승 기준**

* **가장 빠른 유효 랩타임 기록 팀 우승**
* 추가 시상(선택):

  * 최적 경로상
  * 가장 부드러운 조향상
  * 기술 구현상
  * 문제 해결상

---

# 👥 **3. 팀 구성**

* 팀당 **2–4명 구성 유지 (Lab-11과 동일)**

### 보고서/발표 자료에 포함할 내용

* 팀원 이름 및 학번
* 역할 분담(예시):

  * 최적 경로 생성 담당
  * Pure Pursuit 고속 튜닝 담당
  * 시스템 통합 및 안전 점검
  * 영상 촬영 및 편집
  * 레이스 운영
* 개인별 기여도(%)

---

# 📝 **4. 제출물 구성 (총 30점) — 간단 버전**

Lab-12에서는 제출 부담을 줄이기 위해 **두 항목만 제출**하면 됩니다.

---

## ① **Final Race 발표자료 (20점)**

발표 시간: **팀당 5분 이내**

### 발표자료 구성 (필수 4가지)

1. **프로젝트 요약**

   * Lab-11 기반 파이프라인 요약
   * Lab-12에서 추가된 기능(최적 경로, 고속 PP)

2. **최적 경로 & Pure Pursuit 핵심 설정 요약**

   * Trajectory Generator로 생성한 최적 경로 특징
   * 핵심 파라미터(lookahead, 최고속도, steering limit 등)

3. **최종 레이스 전략 요약**

   * 최고속도 확보 전략
   * 안정성 확보 전략
   * 팀이 적용한 가장 효과적 개선 1–2가지

4. **Lessons Learned**

   * Lab-11 → Lab-12에서 달라진 점
   * 향후 개선하고 싶은 기술

📌 스크린샷/그래프 등은 *선택 사항*
📌 **4–6장 슬라이드**로 충분

---

## ② **Final Race 주행 영상 링크 제출 (10점)**

### 포함 내용

* 공식 **타임랩 1회 전체 영상** (필수)
* 팀 소개 약 10초(선택)
* YouTube 또는 Google Drive 링크 제출

---

# 🚗 **5. Final Race 평가 (총 70점)**

### **① 랩타임 성능 (45점)**

* Time-Trial 랩타임 순위별 점수
* 1위 45점 → 2위 40점 → 3위 37점 …

### **② 주행 안정성 & 충돌 여부 (15점)**

* 충돌 없음 & 안정적 주행 → 15점
* 경미한 흔들림 → 10~12점
* 충돌 1회: -5초 패널티
* 충돌 2회: 시도 무효

### **③ 기술 Q&A (10점)**

* 발표 후 간단 질의

  * 핵심 파라미터 선택 이유
  * 최적 경로가 성능에 미친 영향
  * 직선/코너 주행 전략
  * 문제 해결 과정 설명

---

# 📊 **최종 점수 요약**

| 평가 항목                 | 점수       |
| --------------------- | -------- |
| 제출물 (발표자료 + 영상)       | **30점**  |
| Final Time-Trial Race | **70점**  |
| **최종 점수**             | **100점** |

---

# 📬 문의

**경상국립대학교 김진현 교수**
📧 **[jin.kim@gnu.ac.kr](mailto:jin.kim@gnu.ac.kr)**

