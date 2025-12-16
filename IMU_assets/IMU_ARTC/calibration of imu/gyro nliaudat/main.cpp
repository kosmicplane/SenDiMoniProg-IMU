#include <Wire.h>
#include <LSM6.h>   // Pololu library for LSM6DS33
#include "BluetoothSerial.h"

LSM6 imu;
BluetoothSerial SerialBT;

// Gyro sensitivity: for LSM6DS33 default FS=245 dps -> 0.00875 dps/LSB
// Convert to rad/s: dps * (PI/180)
const float GYRO_SENSITIVITY_DPS_PER_LSB = 0.00875f;
const float DPS_TO_RADS = 3.14159265358979323846f / 180.0f;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Start Bluetooth Classic serial with a device name
  if (!SerialBT.begin("ESP32-Gyro")) {
    Serial.println("Failed to start Bluetooth");
  } else {
    Serial.println("Bluetooth started: ESP32-Gyro");
  }

  if (!imu.init()) {
    Serial.println("Failed to detect LSM6DS33!");
    while (1) delay(1000);
  }

  imu.enableDefault();

  // Optional: keep accelerometer settings the same as previous code
  imu.writeReg(LSM6::CTRL1_XL, 0x40); 
  // ODR = 104 Hz, ±2g, BW = 50 Hz

  Serial.println("IMU ready — sending gyro over Serial and Bluetooth");
}

void loop() {
  imu.read();  // Read all IMU registers

  // Raw gyro
  int16_t gx_raw = imu.g.x;
  int16_t gy_raw = imu.g.y;
  int16_t gz_raw = imu.g.z;

  // Convert raw -> dps -> rad/s
  float gx_rads = gx_raw * GYRO_SENSITIVITY_DPS_PER_LSB * DPS_TO_RADS;
  float gy_rads = gy_raw * GYRO_SENSITIVITY_DPS_PER_LSB * DPS_TO_RADS;
  float gz_rads = gz_raw * GYRO_SENSITIVITY_DPS_PER_LSB * DPS_TO_RADS;

  // CSV line: gx,gy,gz in rad/s
  char csv[128];
  snprintf(csv, sizeof(csv), "%.6f,%.6f,%.6f", gx_rads, gy_rads, gz_rads);

  // Send to Serial and Bluetooth (if connected, BluetoothSerial will buffer)
  Serial.println(csv);
  SerialBT.println(csv);

  delay(50);  // ~20 Hz output
}
