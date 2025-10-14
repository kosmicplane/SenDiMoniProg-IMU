# IMU Calibration and Combined Sensor System

This repository provides complete workflows for calibrating individual IMU sensors — **magnetometer**, **accelerometer**, and **gyroscope** — along with a **final integrated program** that combines all three calibrated sensors into a unified data output.

Each calibration folder includes scripts and programs to collect, process, and visualize data for precise sensor correction. The **combined IMU** folder contains the finalized program that merges all calibrated outputs into a single, deployable system.

---

## 📁 Folder Overview

### 🧭 **mag calibration/**

Contains the workflow for **magnetometer calibration**:

* Collects raw magnetic field data from the sensor.
* Calculates **hard iron** and **soft iron** correction matrices.
* Applies bias and scaling corrections.
* Includes visualization tools for checking calibration accuracy.

### ⚙️ **gyro calibration/**

Contains the workflow for **gyroscope calibration**:

* Records raw angular velocity data.
* Computes **bias and drift compensation**.
* Visualizes gyro stability and post-calibration performance.

### 🌍 **accel calibration/**

Contains the workflow for **accelerometer calibration**:

* Collects raw accelerometer data and corrects for bias and scale error.
* Uses a **1D Kalman filter** on each axis for smoothing and noise reduction.
* Supports Bluetooth data transmission and real-time visualization.
* Outputs data in gravitational units (**g**).

### 🔗 **combined imu/**

This is the **final integrated IMU system** that merges all three calibrated sensors (accelerometer, gyroscope, and magnetometer) into one synchronized output stream.

* Outputs data in the format:

  ```
  ax, ay, az, gx, gy, gz, mx, my, mz, pressure, altitude, temperature
  ```
* Implements the same **1D Kalman filter** logic used in accelerometer calibration for stable readings.
* Directly transmits data over **Serial** and **Bluetooth (BLE)**.
* Can be used with `ble reader.py` to receive and log data via Bluetooth.
* Contains only **one main file (`main.cpp`)**, which performs all operations — calibration correction, filtering, and data output.

---

## 🚀 Recommended Usage

For direct deployment and real-world applications, use the program in the **`combined imu`** folder.
It is fully self-contained and ready to be **uploaded to your microcontroller** (e.g., ESP32).
This program provides complete IMU data output and does not require separate calibration or data-saving scripts from the other folders.