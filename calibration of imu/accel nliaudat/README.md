# Accelerometer Calibration and Visualization

This project provides a workflow for calibrating a standalone accelerometer, visualizing its data, and enabling Bluetooth data transmission. It is based on a port of code from [nliaudat's magnetometer calibration](https://github.com/nliaudat/magnetometer_calibration).

## Workflow Overview

1. **Data Collection**

   * `main.cpp` collects raw accelerometer data and publishes it to the serial monitor.
   * The serial output is saved to `accel_data_log`.

2. **Calibration**

   * Use `calibration.py` to process `accel_data_log` and calculate **bias** and **scale error** values.
   * This script adapts the *nliaudat* algorithm to compute bias and scale (instead of hard and soft iron matrices).
   * Calibrated data points are saved in the `accel_data_cal` directory.

3. **Applying Calibration and Bluetooth Transmission**

   * `correctedmain.cpp` applies the corrected bias and scale matrix to the raw data.
   * It then publishes corrected data to the serial monitor and enables Bluetooth functionality as **"ESP32-Accel"**, broadcasting data over Bluetooth serial.

4. **1D Kalman Filter for Axis-wise Correction**

   * Each accelerometer axis (X, Y, Z) implements a **1D Kalman filter** to smooth and stabilize sensor readings.
   * The Kalman filter dynamically reduces noise and drift by fusing new sensor data with previous estimates, providing more accurate and stable output values for each axis.
   * This filtering enhances real-time performance and ensures reliable readings, especially under vibration or transient motion.

5. **Bluetooth Data Reading**

   * `ble reader.py` reads the Bluetooth output from `correctedmain.cpp` and saves the received points as a text file.

6. **Data Visualization**

   * Use `visualizer.py` and `live visualizer.py` to plot and visualize the output data as graphs.

## Output Format

All accelerometer readings are expressed in **gravitational units (g)**.

## File Structure

* `main.cpp` — Raw data collection
* `accel_data_log` — Saved raw data
* `calibration.py` — Calibration script
* `accel_data_cal/` — Calibrated data points
* `correctedmain.cpp` — Applies calibration, implements Kalman filtering, and enables Bluetooth
* `ble reader.py` — Reads Bluetooth data
* `visualizer.py`, `live visualizer.py` — Data visualization

## References

* Original calibration algorithm: [nliaudat/magnetometer_calibration](https://github.com/nliaudat/magnetometer_calibration)
