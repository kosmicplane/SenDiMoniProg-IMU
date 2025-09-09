#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>
#include "BluetoothSerial.h"

LSM6 imu;
LIS3MDL mag;
LPS ps;
BluetoothSerial BTSerial;

void setup() {
  Serial.begin(230400); // For USB debugging only.
  BTSerial.begin("ESP32_IMU"); // Bluetooth device name.
  Wire.begin();

  if (!imu.init()) {
    BTSerial.println("Failed to initialize LSM6!");
    while (1);
  }
  imu.enableDefault();

  imu.writeReg(LSM6::CTRL1_XL, 0x48);
  imu.writeReg(LSM6::CTRL2_G, 0x48);

  if (!mag.init()) {
    BTSerial.println("Failed to initialize LIS3MDL!");
    while (1);
  }
  mag.enableDefault();

  mag.writeReg(LIS3MDL::CTRL_REG1, 0x7C);

  if (!ps.init()) {
    BTSerial.println("Failed to initialize LPS22DF!");
    while (1);
  }
  ps.enableDefault();

  ps.writeReg(0x10, 0x60);
}

void loop() {
  imu.read();
  mag.read();
  float pressure = ps.readPressureMillibars();
  float temperature = ps.readTemperatureC();
  float altitude = ps.pressureToAltitudeMeters(pressure, 1007.9f);

  float ax = imu.a.x * 0.000122f;
  float ay = imu.a.y * 0.000122f;
  float az = imu.a.z * 0.000122f;

  float gx = imu.g.x * 0.0175f * M_PI / 180.0;
  float gy = imu.g.y * 0.0175f * M_PI / 180.0;
  float gz = imu.g.z * 0.0175f * M_PI / 180.0;

  float mx = mag.m.x * 0.14f;
  float my = mag.m.y * 0.14f;
  float mz = mag.m.z * 0.14f;

  BTSerial.print("A[g]: ");
  BTSerial.print(ax, 3); BTSerial.print(", ");
  BTSerial.print(ay, 3); BTSerial.print(", ");
  BTSerial.print(az, 3);

  BTSerial.print(" | G[rad/s]: ");
  BTSerial.print(gx, 2); BTSerial.print(", ");
  BTSerial.print(gy, 2); BTSerial.print(", ");
  BTSerial.print(gz, 2);

  BTSerial.print(" | M[mG]: ");
  BTSerial.print(mx, 1); BTSerial.print(", ");
  BTSerial.print(my, 1); BTSerial.print(", ");
  BTSerial.print(mz, 1);

  BTSerial.print(" | P[mbar]: ");
  BTSerial.print(pressure, 2);
  BTSerial.print(" | T[C]: ");
  BTSerial.print(temperature, 2);
  BTSerial.print(" | Alt[m]: ");
  BTSerial.println(altitude, 2);

  delay(5);
}