
# 🏁 **Lab-11 과제: **SLAM → Particle Filter → Waypoint → Pure Pursuit 자율주행 구현**

---

## 🎯 **1. 과제 목표**

이 Lab의 목표는 **F1TENTH 자율주행 차량의 핵심 파이프라인을 처음부터 끝까지 구축하여 실제 자율주행을 완성하는 것**입니다.

학생 팀은 아래 네 단계(SLAM 맵 생성 → Particle Filter Localization → Waypoint 생성 → Pure Pursuit 자율주행)를 **정해진 순서로** 수행해야 하며,
**12월 11일 실습 시간에 실제 차량으로 2랩 이상 완전 자율주행을 성공**하는 것이 최종 평가의 핵심입니다.

---

# 🧭 **2. 수행해야 할 전체 파이프라인**

아래 네 단계는 자율주행을 구현하기 위해 반드시 **연결된 흐름**이므로, 순서를 바꿀 수 없습니다.

---

# 1️⃣ **Task 1 — SLAM Toolbox로 맵 생성 (환경 구축)**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/03-Tuturial_SLAM.md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/03-Tuturial_SLAM.md)

### 📌 **Task 설명 (추가 설명 포함)**

* SLAM Toolbox를 ROS2 환경에 설치하고 SLAM 노드를 실행합니다.
* 키보드 텔레옵 또는 RC 조작을 사용하여 직접 주행하면서 **환경 지도를 생성**합니다.
* 생성된 맵은 `.yaml`과 `.pgm` 형태로 저장되며, 이후 **Particle Filter Localization의 입력**으로 사용됩니다.
* 이 단계의 품질(맵 정밀도)이 뒤 단계의 Localization과 Waypoint 안정성에 직접 영향을 줍니다.
  → 따라서 **과격한 조향 없이 일정 속도로 부드럽게 주행**하는 것이 중요합니다.

---

# 2️⃣ **Task 2 — Particle Filter Localization 실행**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/04-Tuturial_ParticleFilter.md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/04-Tuturial_ParticleFilter.md)

### 📌 **Task 설명 (추가 설명 포함)**

* Particle Filter(PF) 패키지를 설치하고 실행하여 **실시간 위치 추정(Localization)** 을 수행합니다.
* Task 1에서 만든 맵을 불러와 차량이 현재 어디에 있는지 지속적으로 추정합니다.
* RViz에서 파티클 분포가 점점 좁아지고 실제 차량 위치로 수렴하는지 확인해야 합니다.
* PF가 불안정할 경우 다음을 점검해야 합니다:

  * 맵 오류
  * LiDAR 스캔 품질
  * 파티클 수 및 노이즈 파라미터
* 이 단계는 **Waypoint 생성 및 자율주행의 정확도에 핵심적**입니다.

---

# 3️⃣ **Task 3 — Manual Waypoint Generator로 Waypoint 생성**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/06-1-Tutorial_Manual_WP_Gen.md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/06-1-Tutorial_Manual_WP_Gen.md)

### 📌 **Task 설명 (추가 설명 포함)**

* Particle Filter가 제공하는 실시간 차량 포즈를 기반으로 **경로 정보(waypoints)** 를 기록합니다.
* 차량을 트랙 전체에 걸쳐 부드럽게 주행하며 한 번에 경로를 생성하는 것이 중요합니다.
* Waypoint는 CSV 형태 좌표 리스트이며 Pure Pursuit의 목표 경로로 사용됩니다.
* Waypoint가 끊기거나 급격하게 요동하면 자율주행에서 불안정한 조향을 만들게 됩니다.
  → 가능한 **중심선에 가깝게 일정한 속도로 주행**하며 waypoint를 만드세요.

---

# 4️⃣ **Task 4 — Pure Pursuit 기반 완전 자율주행 구현**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/06-2-Tutorial_PP_ManualWP.md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/06-2-Tutorial_F110_PP_ManWP.md)

### 📌 **Task 설명 (추가 설명 포함)**

* Task 3에서 생성한 waypoint 파일을 Pure Pursuit 노드의 입력으로 사용합니다.
* Lookahead distance, 최대 속도, 조향 각도 제한 등 핵심 파라미터를 팀이 직접 조정해야 합니다.
  → 직선/코너/좁은 구간 등을 고려하면 성능이 크게 달라집니다.
* 목표는 **사람의 조작 없이 차량이 스스로 2랩 이상 완전 자율주행**하는 것입니다.
* Pure Pursuit 주행은 다음에 영향을 받습니다:

  * waypoint 품질
  * lookahead 설정
  * Steering limit
  * 차량 속도
* 실험을 반복하며 최소 2랩 안정적으로 완주하는 튜닝이 필수입니다.

---

# 👥 **3. 팀 구성 안내**

* 팀 인원: **2–3명**
* 최종 문서에 반드시 포함해야 하는 내용:

  * 팀원 이름 및 학번
  * 역할 분담 (예시):

    * SLAM 담당
    * PF Localization 담당
    * Waypoint 생성 담당
    * Pure Pursuit 및 파라미터 튜닝 담당
    * 주행 영상 기록 및 정리 담당
  * 실 기여도(%) 표기

---

# 📝 **4. 제출물 구성 (총 40점)**

📌 단계별 제출물(맵/캡처/파라미터 파일 등)은 일절 제출할 필요 없습니다.
📌 **“최종 프로젝트 문서 1개 + 주행 영상 링크 1개”만 제출합니다.**

## **① 프로젝트 개요 (10점)**

* 실험 환경(차량 번호, LiDAR, Jetson, ROS 버전 등)
* 트랙 환경의 특징
* 파이프라인 전체 목표 설명

## **② 팀 구성 및 책임 (10점)**

* 역할 분담 및 기여도
* 각 역할에서 수행한 주요 활동(서술형)

## **③ 프로젝트 수행 과정 (20점)**

아래 내용을 **텍스트 중심**으로 정리(스크린샷 불필요):

* SLAM 수행 방식 및 특징, 어려움/개선 시도
* PF Localization 안정화 과정
* Waypoint 생성 시 주행 전략 및 파라미터
* Pure Pursuit 실행 및 튜닝 과정
* 주요 문제 해결 사례
* 최종 자율주행 성공을 위한 개선 반복
* Lessons Learned (성능과 안정성 중심)

---

# 🚗 **5. 12/11 실차 자율주행 테스트 평가 (총 60점)**

## **① 자율주행 성공 여부 (25점)**

* 0점: 자율주행 미수행
* 10점: 일부 구간만 자율주행
* 20점: 1랩 완주
* **25점: 2랩 이상 완전 자율주행 성공**

## **② 주행 안정성 & 안전성 (15점)**

* 급조향/진동/브레이크 잦음 여부
* 벽 충돌 유무
* 전체적인 안정성

## **③ 주행 성능 (10점)**

* 평균 주행 속도
* 경로 추종 정확도

## **④ 질의응답 (10점)**

* 파라미터 선택 근거
* SLAM → PF → WP → PP 전체 흐름 이해도
* 문제 해결 과정 설명 능력

---

# 📊 **6. 총점 요약**

| 평가 항목       | 배점       |
| ----------- | -------- |
| 프로젝트 문서     | 40점      |
| 실차 자율주행 테스트 | 60점      |
| **총점**      | **100점** |

---
