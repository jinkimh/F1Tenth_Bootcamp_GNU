# 🏁 **Lab-11 Assignment: Implementing the Full Autonomy Pipeline — SLAM → Particle Filter → Waypoint → Pure Pursuit**

---

## 🎯 **1. Assignment Objective**

The objective of this Lab is to **build the core autonomy pipeline of the F1TENTH autonomous vehicle from start to finish and achieve full autonomous driving in the real world.**

Student teams must complete the following four stages **in the specified order**:

1. SLAM map creation
2. Particle Filter localization
3. Waypoint generation
4. Pure Pursuit autonomous driving

The **final evaluation** is based on **successfully achieving 2 or more fully autonomous laps** during the in-person driving test on **December 11**.

---

# 🧭 **2. Required Full Pipeline Overview**

The following four stages form a **single, connected pipeline**, meaning the order **cannot be changed**.

---

# 1️⃣ **Task 1 — Map Creation Using SLAM Toolbox (Environment Construction)**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/03-Tuturial_SLAM(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/03-Tuturial_SLAM%28English%29.md)

### 📌 **Task Description (with additional explanation)**

* Install SLAM Toolbox in the ROS2 environment and run the SLAM node.
* Drive manually using keyboard teleop or RC control to **create a 2D map of the environment**.
* The generated map is saved as `.yaml` and `.pgm` files, which will later be used as the **input for the Particle Filter localization**.
* The quality of this stage (map accuracy) has a **direct impact** on the stability of localization and waypoint generation.
  → Therefore, maintain a **smooth and steady driving speed** without aggressive steering.

---

# 2️⃣ **Task 2 — Running Particle Filter Localization**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/04-Tuturial_ParticleFilter(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/04-Tuturial_ParticleFilter%28English%29.md)

### 📌 **Task Description (with additional explanation)**

* Install and run the Particle Filter (PF) package for **real-time pose estimation**.
* Load the map created in Task 1 and continuously estimate the vehicle’s current position.
* In RViz, verify whether the particle distribution gradually converges around the vehicle’s actual pose.
* If PF behavior is unstable, check:

  * Mapping errors
  * LiDAR scan quality
  * Number of particles & noise parameters
* This stage is **critical for the accuracy of waypoint generation and final autonomous driving**.

---

# 3️⃣ **Task 3 — Generating Waypoints Using the Manual Waypoint Generator**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/06-1-Tutorial_Manual_WP_Gen(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/06-1-Tutorial_Manual_WP_Gen%28English%29.md)

### 📌 **Task Description (with additional explanation)**

* Using the real-time pose estimated by the Particle Filter, record **waypoints along the driving path**.
* Drive smoothly along the entire track to generate a continuous path in one session.
* The generated waypoint file is a CSV list of coordinates, which will be used as the target path for Pure Pursuit.
* If the waypoint path has gaps or sudden jumps, the autonomous steering will become unstable.
  → Drive **near the center line** with **consistent speed** to create clean waypoints.

---

# 4️⃣ **Task 4 — Implementing Pure Pursuit Autonomous Driving**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/06-2-Tutorial_F110_PP_ManWP.md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/06-2-Tutorial_F110_PP_ManWP.md)

### 📌 **Task Description (with additional explanation)**

* Load the waypoint file created in Task 3 into the Pure Pursuit node.
* Adjust key parameters such as lookahead distance, maximum velocity, and steering angle limits.
  → Performance varies significantly depending on straight sections, curves, and narrow spaces.
* The goal is to achieve **2 or more fully autonomous laps without any manual intervention**.
* Pure Pursuit performance depends on:

  * waypoint quality
  * lookahead distance
  * steering limits
  * vehicle speed
* Repeated tuning is required to achieve stable 2-lap performance.

---

# 👥 **3. Team Composition Guidelines**

* Team size: **2–3 members**
* The final report must include:

  * Names and student IDs
  * Clear role assignments (example):

    * SLAM operation
    * PF localization
    * Waypoint generation
    * Pure Pursuit & parameter tuning
    * Driving test & video recording
  * Contribution percentage (%) for each member

---

# 📝 **4. Submission Requirements (Total 40 points)**

📌 No stage-specific submissions are required (no maps, screenshots, parameter files, etc.)
📌 Submit only **one final project report + one driving video link**.

---

## **① Project Overview (10 pts)**

* Hardware environment (vehicle ID, LiDAR, Jetson, ROS version, etc.)
* Track characteristics
* Overall pipeline goal

---

## **② Team Composition & Responsibilities (10 pts)**

* Role assignments and contribution percentages
* Summary of major work by each member

---

## **③ Project Execution Description (20 pts)**

Write the content **in descriptive text only** (screenshots not required):

* SLAM method, characteristics, difficulties, improvements
* PF localization stabilization process
* Waypoint generation strategy & parameters
* Pure Pursuit tuning rationale and testing process
* Major troubleshooting examples
* Improvement iterations leading to final autonomous driving success
* Lessons Learned (focused on performance & stability)

---

# 🚗 **5. Autonomous Driving Test on 12/11 (Total 60 points)**

## **① Autonomous Driving Success (25 pts)**

* 0 pts: No autonomous driving
* 10 pts: Partial autonomous driving
* 20 pts: Completes 1 lap
* **25 pts: Successfully completes 2 or more fully autonomous laps**

---

## **② Driving Stability & Safety (15 pts)**

* Excessive steering / vibration / frequent braking
* Collisions with walls
* Overall stability

---

## **③ Driving Performance (10 pts)**

* Average speed
* Path-tracking accuracy

---

## **④ Q&A Evaluation (10 pts)**

* Parameter selection rationale
* Understanding of the full pipeline (SLAM → PF → WP → PP)
* Ability to explain troubleshooting steps

---

# 📊 **6. Total Score Summary**

| Evaluation Category     | Points      |
| ----------------------- | ----------- |
| Project Report          | 40 pts      |
| Autonomous Driving Test | 60 pts      |
| **Total**               | **100 pts** |

---

---

## 🔒 License & Copyright

The tutorials and related scripts are protected under the following copyright:

```
© 2025 Jin Kim, Gyeongsang National University

This material may be used freely for educational and research purposes.
Commercial use or redistribution without prior consent is prohibited.
```

---

## 📬 Contact

For inquiries or feedback, please contact:

**Prof. Jin Hyun Kim, Gyeongsang National University
Email: [jin.kim@gnu.ac.kr](mailto:jin.kim@gnu.ac.kr)**

---

## 📄 Copyright Notice

```
© 2025 Jin Kim, Gyeongsang National University

This document was created for educational and experimental use with the F1Tenth autonomous driving platform.  
Unauthorized reproduction or commercial use is strictly prohibited.
```
