# Manual Waypoint Generator User Guide (`wpgen_manual.py`)

This tool allows you to generate waypoints manually by clicking on a map image.
This document explains the environment setup and how to run the script.

---

## **Prerequisites**

Before running the script, prepare the following:

* **Python 3.7**
* **conda** (or any other environment management tool)

---

## **Installation Guide**

Follow the steps below to set up the environment and install required libraries.

---

### **1. Clone the Repository**

```bash
git clone https://github.com/jinkimh/wpgen_manual.git
cd wpgen_manual/
```

* Clones the GitHub repository and moves into the project directory.

---

### **2. Create and Activate a Conda Environment**

```bash
conda create -n wpgen python=3.7 -y
conda activate wpgen
```

* Creates a Conda environment named `wpgen` with Python 3.7 and activates it.

---

### **3. Install Dependency Packages**

Install all required Python libraries listed in `requirements.txt`:

```bash
pip install -r requirements.txt
```

---

## **How to Use**

### **Prepare the Map Files**

Before running the script, make sure to prepare:

1. **SLAM Toolbox output files** — the map image and corresponding `.yaml` file
   Save them into the `map` folder.

   Example:

   ```
   ./map/ict_3rd_floor.png
   ./map/ict_3rd_floor.yaml
   ```

---

### **Run the Script**

To start the waypoint generation process, execute:

```bash
python3 wpgen_manual.py ./map/ict_3rd_floor.png
```

* Provide the map file path as the script argument.

---

## **How to Generate Waypoints**

1. **Load the Map Image**
   When the script starts, the map image will be displayed.

2. **Select Waypoints**
   Click on the map to add waypoints along your desired path.

   * **Tip:** More densely placed waypoints result in smoother driving trajectories.

3. **Generate Final Waypoints**
   Once all waypoints are selected, press **“r”** to generate the final waypoint file.

---

## **Troubleshooting**

### **Missing Dependencies**

If you encounter `ModuleNotFoundError`, reinstall all required packages:

```bash
pip install -r requirements.txt
```

### **Conda Environment Not Activated**

Ensure that the Conda environment is active before running the script:

```bash
conda activate wpgen
```

---

## **Notes**

* Python 3.7 is recommended for compatibility.
* You can customize behavior or extend features by editing the `wpgen_manual.py` script.

---

## **License**

This project is open-source under the [MIT License](LICENSE).

---

## 📄 Copyright

```
© 2025 Jin Kim, Gyeongsang National University

This document was created for experimental and educational use  
with the F1TENTH autonomous driving platform.  
Unauthorized reproduction or commercial use is prohibited.
```

