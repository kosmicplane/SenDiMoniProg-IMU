# 🧭 Magnetometer Calibration and Bluetooth Data Visualization

This project provides a complete workflow for calibrating a standalone magnetometer, visualizing its data, and transmitting the corrected readings over Bluetooth.
It is based on and extends [nliaudat’s Magnetometer Calibration Repository](https://github.com/nliaudat/magnetometer_calibration) with additional features for real-time visualization and wireless data transmission using an ESP32.

---

## 🚀 **Overview**

The workflow consists of five main stages:

1. **Data Collection** — Capture raw magnetometer readings.
2. **Calibration** — Correct distortions using hard-iron and soft-iron compensation.
3. **Correction & Transmission** — Apply calibration and send data over Bluetooth.
4. **Bluetooth Data Reading** — Receive and store transmitted values.
5. **Visualization** — Plot 2D/3D magnetic field data for validation.

---

## 🧩 **Workflow Details**

### 1. Data Collection

* **File:** `main.cpp`
* Reads raw magnetometer data (X, Y, Z axes) via I²C/SPI and outputs it to the serial monitor.
* The raw data stream is saved to the **`mag_out.txt`** file.
* This dataset serves as input for the calibration step.

---

### 2. Calibration

* **File:** `calibration.py`
* Processes raw magnetometer data to calculate **hard-iron offset** and **soft-iron distortion matrix**.
* Utilizes nonlinear least-squares fitting (ellipsoid fitting) to find correction parameters.
* Outputs key calibration values such as:

  * **Bias (hard-iron offset)**
  * **Transformation matrix (soft-iron compensation)**
  * **Calibration quality metrics**

---

### 3. Applying Calibration & Bluetooth Transmission

* **File:** `correctedmain.cpp`
* Applies the computed **bias and transformation matrix** to the live magnetometer readings.
* Publishes both raw and corrected magnetic field data to the serial monitor.
* Enables Bluetooth under the name **“ESP32-Mag”**, broadcasting real-time calibrated data via Bluetooth Serial.

---

### 4. Bluetooth Data Reading

* **File:** `logger.py`
* Connects to the ESP32 Bluetooth interface and reads the transmitted magnetic field data.
* Stores the incoming values as a text file for analysis and visualization.
* Can be configured for live plotting or offline storage.

---

### 5. Data Visualization

* **Files:** `visualizer.py`
* Used for plotting the calibration results and real-time magnetic field visualization.
* Visuals include:

  * 2D XY/YZ/XZ scatter plots showing field uniformity before and after calibration.
  * 3D sphere representation of the corrected field vectors.
  * Live updating graphs for continuous monitoring.

---

## 📁 **File Structure**

```
├── main.cpp               # Raw magnetometer data collection
├── mag_out/              # Directory for saved raw data
├── calibration.py         # Calibration algorithm (hard & soft iron correction)
├── correctedmain.cpp      # Applies calibration and handles Bluetooth transmission
├── logger.py          # Reads Bluetooth data and saves readings
├── visualizer.py          # Static visualization of calibration results
```
---

## 🔗 **Reference**

* Original algorithm: [nliaudat/magnetometer_calibration](https://github.com/nliaudat/magnetometer_calibration)
