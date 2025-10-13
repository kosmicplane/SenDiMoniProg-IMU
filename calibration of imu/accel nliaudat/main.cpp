#include <Wire.h>
#include <LSM6.h>   // Pololu library for LSM6DS33

LSM6 imu;

void setup() {
  Serial.begin(230400);
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect LSM6DS33!");
    while (1);
  }

  imu.enableDefault();

  // Default ±2g range → sensitivity = 0.061 mg/LSB = 0.000061 g/LSB
  imu.writeReg(LSM6::CTRL1_XL, 0x40); 
  // ODR = 104 Hz, ±2g, BW = 50 Hz
}

void loop() {
  imu.read();  // Read all IMU registers

  // Convert raw accel to g units
  float ax = imu.a.x * 0.000061f;  
  float ay = imu.a.y * 0.000061f;
  float az = imu.a.z * 0.000061f;

  // Print in CSV format
  Serial.printf("%.6f,%.6f,%.6f\n", ax, ay, az);

  delay(50);  // ~20 Hz output
}
