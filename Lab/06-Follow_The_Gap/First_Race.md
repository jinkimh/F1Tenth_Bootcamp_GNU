Here’s the updated assignment specification with an explicit requirement that **each team must base its development on the Lab-06 materials** from your repository link:

---

# **Lab-06 Team Project — FTG Racer Challenge (ROS 2 Foxy)**

**Race Day:** **November 17, 2025 (Mon)**
**Track:** Indoor F1TENTH loop with static obstacles (cones/boxes)
**Vehicle:** Real F1TENTH AV running ROS 2 Foxy

---

## **1. Project Objective**

Teams must **develop and tune a Follow-the-Gap (FTG) reactive driving system** on the real F1TENTH car.
The starting point **must be the official Lab-06 tutorial and materials** provided here:
🔗 [https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/Lab/06-Follow_The_Gap/Lab-06-Follow_The_Gap_Methods_E.md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/Lab/06-Follow_The_Gap/Lab-06-Follow_The_Gap_Methods_E.md)

All teams are expected to **follow, extend, and modify** the Lab-06 FTG implementation to achieve:

1. **Collision-free autonomous laps** using LiDAR-based FTG.
2. **Three timed laps** per team; best average lap time after penalties determines ranking.

> You may reuse or adapt any **open-source FTG implementation** (with citation), but your integration, tuning, and testing must build upon the above Lab-06 framework.

---

## **2. Team Structure and Duties**

Each team defines clear roles in its repository README:

| Role                       | Responsibilities                                    | Deliverables                     |
| -------------------------- | --------------------------------------------------- | -------------------------------- |
| **Team Lead / Integrator** | Timeline, merges, risk mgmt, final integration      | Build logs, final report         |
| **Perception Engineer**    | LiDAR pre-processing, bubble masking, gap detection | Parameter table, plots           |
| **Control Engineer**       | Steering and speed profiles, safety limits          | Steering-speed maps, tuning logs |
| **Systems Engineer**       | ROS2 package setup, topic remaps, bring-up tests    | Launch files, bag logs           |
| **Documentation Manager**  | Report, slides, submission package                  | PDF report, slides, video        |

---

## **3. Development Requirements**

* **ROS 2 Foxy**, Python 3.8, workspace `~/f1tenth_ws`
* Subscribed topic: `/scan` (`sensor_msgs/LaserScan`)
* Published topic: `/drive` (`ackermann_msgs/AckermannDriveStamped`)
* Pipeline:

  1. Range clipping & smoothing
  2. Bubble masking around closest obstacle
  3. Max-gap detection
  4. Steer-to-gap-midpoint
  5. Speed profiling based on |δ| and gap width
* Safety rules: steering/speed clamps, E-Stop readiness, staged indoor testing
* All code and logs tracked in Git (commit history required)

---

## **4. Deliverables**

1. **Git Repository**

   * Based on Lab-06 FTG package (`gap_follow`).
   * Contains code, `package.xml`, `setup.py`, and launch/config files.
   * Includes:

     * Parameter tuning logs
     * Team duties table
     * External FTG citations
   * Public or course-internal access with tag `v1.0-race`.

2. **Experiment Report (PDF, ≤ 6 pages)**

   * Implementation details referencing the Lab-06 baseline.
   * Parameter sweeps and tuning results.
   * Analysis of success/failure cases.
   * Lap-time summary table and plots.

3. **Presentation Slides (≤ 6 slides)**

   * Overview, key findings, FTG variant used, next steps.

4. **Demo Video (30–60 s)**

   * Clean autonomous lap with obstacle avoidance.

---

## **5. Race Rules**

* 3 timed laps; average time + penalties used.
* **Penalties:**

  * Collision +15 s
  * Lane departure +10 s
  * Human intervention → lap = 120 s
* Ranking = lowest penalized average; tiebreakers = fewest collisions → fastest clean lap.

---

## **6. Evaluation (100 pts)**

| Area                         | Pts | Criteria                                                  |
| ---------------------------- | --: | --------------------------------------------------------- |
| System correctness           |  15 | Node reliability, topics, parameters                      |
| FTG implementation           |  20 | Proper pipeline (per Lab-06)                              |
| Safety & robustness          |  15 | Stable, collision-free behavior                           |
| Experimentation              |  15 | Parameter tuning, analysis, plots                         |
| Racing performance           |  25 | Lap-time vs median, penalties                             |
| Documentation & presentation |  10 | Complete repo, reproducible report                        |
| **Bonus (+5)**               |   – | Advanced FTG (TTC speed cap, hysteresis, adaptive bubble) |

---

## **7. Key Notes**

* Your FTG must **originate from and align with**
  [Lab-06-Follow The Gap Methods E.md](https://github.com/jinkimh/F1Tenth_Bootcamp_GNU/blob/main/Lab/06-Follow_The_Gap/Lab-06-Follow_The_Gap_Methods_E.md).
  All deviations or improvements must be **clearly justified** in your report.
* Using additional **open-source FTG software** is permitted with proper **citations**.
* Simulation tests (Rviz/Gazebo) are encouraged before running on hardware.

---

**Good luck on Race Day!**
Build upon the Lab-06 foundation, iterate safely, and deliver a fast, robust FTG racer.
