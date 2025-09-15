Bluetooth_IMU
This project aims to use Bluetooth to connect an ESP32 with a Raspberry Pi, enabling real-time reading and plotting of data from an IMU (Inertial Measurement Unit) sensor.

Features
Upload the driver to the ESP32 via a USB cable.

After powering the ESP32 independently, it establishes a Bluetooth connection with the Raspberry Pi.

The Raspberry Pi receives IMU data and plots it in real time.

Technology Stack
Microcontroller: ESP32

Single-Board Computer: Raspberry Pi

Programming Languages: C++ (for ESP32), Python (for Raspberry Pi)

Setup and Execution
Step 1: Upload the ESP32 Driver
Connect the ESP32 to the Raspberry Pi using a USB cable.

Open the MAIN.CPP file.

Upload the driver to the ESP32. This step programs the ESP32 with the capability to transmit data via Bluetooth and read from the IMU.

Step 2: Run Data Reception and Plotting
Once the upload is complete, disconnect the USB cable and power the ESP32 independently.

On your Raspberry Pi, open the terminal.

Execute the plot.py file to begin receiving data and plotting:

python plot.py

The Raspberry Pi will now start receiving IMU data from the ESP32 via Bluetooth and display it in a real-time plot.
