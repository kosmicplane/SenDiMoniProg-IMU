# 🧭 Gyroscope Calibration and Bluetooth Data Visualization

This project provides a complete workflow for calibrating a standalone gyroscope, visualizing its data, and **transmitting corrected data over Bluetooth.
It is adapted from [nliaudat’s Magnetometer Calibration Project](https://github.com/nliaudat/magnetometer_calibration) and restructured for gyroscope-specific bias and scale factor correction.

---

## 🚀 **Overview**

The system follows a structured five-step process:

1. **Data Collection** — Acquire raw gyroscope readings.
2. **Calibration** — Compute and apply bias and scale factor corrections.
3. **Correction & Transmission** — Transmit calibrated data via Bluetooth.
4. **Bluetooth Data Reading** — Receive and store transmitted data.
5. **Visualization** — Plot and analyze gyroscope performance.

---

## 🧩 **Workflow Details**

### 1. Data Collection

* **File:** `main.cpp`
* Captures raw gyroscope data and publishes it via the serial monitor.
* Serial output is saved to the file **`gyro_data`** for further processing.

---

### 2. Calibration

* **File:** `calibration.py`
* Processes the collected raw data to estimate **bias** and **scale error**.
* Adapts nliaudat’s magnetometer calibration algorithm to gyroscopes, replacing hard/soft iron correction with bias and scaling.
* Saves calibrated data points in the **`gyro_cal`** directory.

---

### 3. Applying Calibration & Bluetooth Transmission

* **File:** `correctedmain.cpp`
* Applies the computed calibration parameters (bias and scale matrix) to the live gyroscope data.
* Publishes corrected values to the serial monitor.
* Simultaneously enables Bluetooth under the device name **“ESP32-gyro”**, broadcasting data via Bluetooth Serial.

---

### 4. Bluetooth Data Reading

* **File:** `ble reader.py`
* Connects to the ESP32 via Bluetooth and continuously reads the transmitted calibrated data.
* Stores the incoming points in a text file for later analysis or visualization.

---

### 5. Data Visualization

* **Files:** `visualizer.py` and `live visualizer.py`
* Used for static and real-time visualization of gyroscope performance.
* Displays calibration quality, drift, and motion trends through live plots.

---

## 📁 **File Structure**

```
├── main.cpp              # Raw gyroscope data collection
├── gyro_data/            # Directory for saved raw data
├── calibration.py        # Calibration algorithm (bias + scale)
├── gyro_cal/             # Calibrated data output
├── correctedmain.cpp     # Applies calibration and handles Bluetooth transmission
├── ble reader.py         # Reads Bluetooth data stream and saves it
├── visualizer.py         # Static data visualization
└── live visualizer.py    # Real-time visualization
```

## 🔗 **Reference**

* Original algorithm: [nliaudat/magnetometer_calibration](https://github.com/nliaudat/magnetometer_calibration)