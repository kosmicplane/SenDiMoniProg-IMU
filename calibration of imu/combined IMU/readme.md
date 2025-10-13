## Combined IMU: Accelerometer, Gyroscope & Magnetometer Integration

This project provides an integrated firmware for a **combined IMU (Inertial Measurement Unit)** that fuses data from the **accelerometer, gyroscope, and magnetometer** sensors.
Unlike the individual calibration modules, this version runs as a **single unified program** handling data acquisition, calibration correction, and Bluetooth transmission within one file — `main.cpp`.

---

## 🚀 **Overview**

The **Combined IMU** program simultaneously reads data from all three motion sensors — acceleration, angular velocity, and magnetic field — applies calibration correction using a **1D Kalman filter** for each accelerometer axis, and transmits the processed results to both the **serial monitor** and via **Bluetooth**.

This implementation combines all functionality into a single workflow — **no separate calibration or data files** are needed.

---

## ⚙️ **Workflow Summary**

1. **Sensor Data Acquisition**

   * The `main.cpp` file collects readings from all sensors:

     * **Accelerometer:** `ax`, `ay`, `az` (measured in **g**)
     * **Gyroscope:** `gx`, `gy`, `gz` (measured in °/s)
     * **Magnetometer:** `mx`, `my`, `mz` (measured in µT)
     * **Environmental sensors:** `pressure`, `altitude`, `temperature`

2. **Calibration Correction (Accelerometer)**

   * Each accelerometer axis (`ax`, `ay`, `az`) is processed through a **1D Kalman Filter** to reduce noise and smooth the readings.
   * Gyroscope and magnetometer data use pre-applied bias/scale corrections directly embedded within the code.

3. **Data Transmission**

   * Outputs formatted IMU readings to:

     * **Serial Monitor** for direct observation
     * **Bluetooth Serial**, with device name **“ESP32-IMU”**
   * Data format:

     ```
     ax, ay, az, gx, gy, gz, mx, my, mz, pressure, altitude, temperature
     ```

4. **Bluetooth Data Reading**

   * **File:** `ble reader.py`
   * Connects to the ESP32 via Bluetooth and receives the transmitted IMU data stream.
   * Can save or process the readings for further visualization or analysis.

---

## 🧠 **Data Output Format**

Example transmitted data line:

```
0.01, 0.00, 0.99, 0.12, -0.03, 0.04, 31.8, -12.7, 43.2, 1009.3, 13.6, 28.9
ax, ay, az, gx, gy, gz, mx, my, mz, pressure, altitude, temperature
```

ax, ay, az  - Linear acceleration     - g (gravitational units) 
gx, gy, gz  - Angular velocity        - deg/s                        
mx, my, mz  - Magnetic field strength - µT                          
pressure    - Atmospheric pressure    - hPa                         
altitude    - Estimated altitude      - m                           
temperature - Sensor temperature      - °C                          

---

## 📁 **File Structure**

```
home/
├── mag calibration/
├── accel calibration/
├── gyro calibration/
└── combined imu/
    ├── main.cpp           # Unified IMU data acquisition, Kalman filtering, and Bluetooth transmission
    └── ble reader.py      # Reads Bluetooth IMU data stream

```

## 🔗 **Repository Layout**

The combined IMU integrates the functionality of the three calibration modules:

* 🧲 **mag calibration** → Magnetic field offset and distortion correction
* 🌀 **gyro calibration** → Gyroscope bias and scale correction
* ⚙️ **accel calibration** → Accelerometer filtering and gravity-based output

All merged into **one unified firmware** under `combined imu`, optimized for real-time streaming and minimal resource usage.
