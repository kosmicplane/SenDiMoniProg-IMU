#include <Arduino.h>
#include <Wire.h>
#include <LIS3MDL.h>

LIS3MDL mag;

// Sensitivity for ±4 gauss (default) in µT/LSB
const float LIS3MDL_SENSITIVITY = 0.014f;

void setup() {
  Serial.begin(230400);
  Wire.begin();

  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL!");
    while (1);
  }

  mag.enableDefault();

  // Optional: set full-scale to ±4 gauss (default)
  mag.writeReg(LIS3MDL::CTRL_REG2, 0x00);

  Serial.println("x,y,z (µT) Magnetometer output");
}

void loop() {
  mag.read();

  float mx = mag.m.x * LIS3MDL_SENSITIVITY;
  float my = mag.m.y * LIS3MDL_SENSITIVITY;
  float mz = mag.m.z * LIS3MDL_SENSITIVITY;

  Serial.print(mx, 6); Serial.print(",");
  Serial.print(my, 6); Serial.print(",");
  Serial.println(mz, 6);

  delay(50); // 20 Hz output
}
