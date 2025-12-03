# 🏁 **Lab-12 Final Race: Optimal Trajectory → Pure Pursuit → Time-Trial Autonomous Racing**

---

## 🎯 **1. Lab-12 Objective**

The goal of Lab-12 is to **extend the autonomous driving pipeline developed in Lab-11 to a high-performance, race-ready system**.

Building on the Lab-11 pipeline:
**SLAM → Particle Filter → Manual Waypoint → Pure Pursuit**,
Lab-12 will introduce higher-speed racing concepts and optimization.

In Lab-12, each team will complete the following:

1. **Generate an Optimal Trajectory (Racing Line)**
2. **Run High-Speed Pure Pursuit using the optimized trajectory**
3. **Participate in a real Time-Trial autonomous race**

Final grading is based on **team presentation + race performance (Time Trial)**.

---

# 🧭 **2. Final Race Workflow**

Lab-12 consists of **two technical tasks (Task 1–2) and the Final Race (Task 3)**.
(Completion of Lab-11 is assumed.)

---

## 1️⃣ **Task 1 — Optimal Trajectory Generation (Optimal Trajectory Generator)**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/07-Tutorial_Trajectory_Generator(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/07-Tutorial_Trajectory_Generator%28English%29.md)

### 📌 **Task Description**

Using the **Optimal Trajectory Generator**, teams should:

* Load the track boundary or manual waypoint file generated in Lab-11
* Run `wp_gen.py` to produce an **optimal racing line**
* Characteristics of the optimized path:
  ✔ Smooth inner-line cornering
  ✔ Full utilization of track width
  ✔ Reduced steering oscillation
  ✔ Higher achievable vehicle speed
* The output is a **CSV waypoint file**, which will be used in Task 2.

---

## 2️⃣ **Task 2 — Pure Pursuit Autonomous Driving Using Optimal Trajectory**

🔗 GitHub Tutorial
[https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/08-Tutorial_PP_OptWP(English).md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/08-Tutorial_PP_OptWP%28English%29.md)

### 📌 **Task Description**

* Load the **optimal trajectory waypoint file** from Task 1 into the Pure Pursuit node
* Tune key parameters for high-speed performance:

  * Lookahead distance
  * Maximum speed
  * Steering angle limit
  * (Optional) Dynamic lookahead, gain tuning, etc.
* Validate stability on the track to ensure smooth high-speed driving
* Optional: **Compare manual vs. optimal waypoints**

→ The main objective is to achieve **a stable yet fast high-performance autonomous run**.

---

## 3️⃣ **Task 3 — Final Time-Trial Autonomous Race**

### 🏎️ **Race Format**

* **100% fully autonomous** driving
* **No manual control allowed** during the race
* Scoring is based on **lap time + stability + technical completeness**

### 🏟️ **Race Rules**

* Each team receives **two official race attempts**

* Each attempt consists of:

  1. Autonomous warm-up lap (not timed)
  2. **Three consecutive timed laps**

* The **fastest valid lap time** across both attempts is recorded

### ⚠️ **Collision Rules**

* 1 collision: +5 seconds penalty
* 3 collisions: run invalidated

### 🥇 **Winner Criteria**

* The team with the **fastest valid lap time** wins

Optional special awards:

* Best Trajectory Award
* Smoothest Steering Award
* Best Engineering Award
* Best Problem-Solving Award

---

# 👥 **3. Team Structure**

* Each team consists of **2–4 students (same as Lab-11)**

### Required in presentation materials:

* Names & student IDs
* Clear role assignments, such as:

  * Optimal trajectory generation
  * High-speed Pure Pursuit tuning
  * System integration & safety checks
  * Video recording & editing
  * Race operation
* Contribution percentage (%) per member

---

# 📝 **4. Submission Requirements (Total 30 Points) — Simplified Version**

To reduce the workload, **only two items must be submitted**.

---

## ① **Final Race Presentation Slides (20 pts)**

Presentation Time: **up to 5 minutes per team**

### Required Contents (4 sections)

1. **Project Overview**

   * Summary of the Lab-11 baseline pipeline
   * New features added in Lab-12 (optimal trajectory, high-speed PP)

2. **Optimal Trajectory & Pure Pursuit Configuration**

   * Characteristics of the generated optimal path
   * Key parameters used (lookahead, maximum speed, steering limits)

3. **Final Race Strategy**

   * Methods for maximizing top speed
   * Stability-enhancing strategies
   * 1–2 most effective improvements the team applied

4. **Lessons Learned**

   * What changed/improved from Lab-11 → Lab-12
   * Future improvements if more time were available

📌 Screenshots/graphs are optional
📌 **4–6 slides** are sufficient

---

## ② **Final Race Driving Video Link (10 pts)**

### Required Contents

* **One full official timed lap** (mandatory)
* ~10 seconds of team introduction (optional but recommended)
* Upload via YouTube or Google Drive and submit link

---

# 🚗 **5. Final Race Evaluation (Total 70 Points)**

### **① Lap-Time Performance (45 pts)**

Scored by Time-Trial ranking:

* 1st: 45 pts
* 2nd: 40 pts
* 3rd: 37 pts
* … and so on

### **② Stability & Collisions (15 pts)**

* No collisions + stable driving → 15 pts
* Minor instability → 10–12 pts
* 1 collision: –5 seconds penalty
* 2 collisions: run invalided

### **③ Technical Q&A (10 pts)**

Post-presentation questions:

* Reason for chosen parameters
* How the optimal path influenced performance
* Strategies for straight vs. corner sections
* Explanation of troubleshooting steps

---

# 📊 **Final Score Summary**

| Category                    | Points  |
| --------------------------- | ------- |
| Submission (Slides + Video) | **30**  |
| Final Time-Trial Race       | **70**  |
| **Total**                   | **100** |

---

# 📬 Contact

**Prof. Jin Hyun Kim,
Gyeongsang National University**
📧 **[jin.kim@gnu.ac.kr](mailto:jin.kim@gnu.ac.kr)**

=
Just let me know!
