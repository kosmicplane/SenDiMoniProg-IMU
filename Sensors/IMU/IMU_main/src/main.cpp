/*
The sensor outputs provided by the library are the raw
16-bit values obtained by concatenating the 8-bit high and
low accelerometer and gyro data registers. They can be
converted to units of g and dps (degrees per second) using
the conversion factors specified in the datasheet for your
particular device and full scale setting (gain).
*/

#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>
#include "BluetoothSerial.h"   // <-- AÑADIDO

LSM6 imu;
LIS3MDL mag;
LPS ps;
BluetoothSerial BTSerial;      // <-- AÑADIDO

float sampling_rate = 100;
float time_interval = 1000000 / sampling_rate;

float current_time = 0;
float previous_time = 0;

void setup() {
  Serial.begin(230400);            // USB (opcional, solo debug)
  BTSerial.begin("ESP32_IMU");     // <-- AÑADIDO: nombre del dispositivo BT
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to initialize LSM6!");
    BTSerial.println("Failed to initialize LSM6!");   // <-- también por BT
    while (1);
  }
  imu.enableDefault();

  // 設定加速度計 ODR 為 104 Hz
  imu.writeReg(LSM6::CTRL1_XL, 0x48); // 0x4x = 104 Hz,0x40     ±2 g,0x44       ±16 g,0x48      ±4 g,0x4C       ±8 g

  // 設定陀螺儀 ODR 為 104 Hz
  imu.writeReg(LSM6::CTRL2_G, 0x48);  // 0x44 = 104 Hz, ±500 dps 0x40 = 104 Hz, ±250 dps

  // Magnetometer
  if (!mag.init()) {
    Serial.println("Failed to initialize LIS3MDL!");
    BTSerial.println("Failed to initialize LIS3MDL!");  // <-- también por BT
    while (1);
  }
  mag.enableDefault();

  // 設定磁力計 ODR 為 80Hz
  mag.writeReg(LIS3MDL::CTRL_REG1, 0x7C); // ultra-high-performance X/Y, ODR=80Hz

  // Pressure sensor (LPS22DF)
  if (!ps.init()) {
    Serial.println("Failed to initialize LPS22DF!");
    BTSerial.println("Failed to initialize LPS22DF!");  // <-- también por BT
    while (1);
  }
  ps.enableDefault();

  // 設定氣壓計 ODR 為 100Hz
  ps.writeReg(0x10, 0x60); // 0x10 = CTRL_REG1, 0x60 = 100Hz

  previous_time = micros();
}

void loop() {

  if ((micros() - previous_time) >= time_interval)
  {
    previous_time = micros();
    imu.read();
    mag.read();
    float pressure = ps.readPressureMillibars();
    float temperature = ps.readTemperatureC();
    float altitude = ps.pressureToAltitudeMeters(pressure, 1016.9f); // 需要校正

    // Convert raw values to physical units
    float ax = imu.a.x * 0.000122f;   // g
    float ay = imu.a.y * 0.000122f;
    float az = imu.a.z * 0.000122f;

    float gx = imu.g.x *  0.0175f ; // deg/s
    float gy = imu.g.y *  0.0175f ;
    float gz = imu.g.z *  0.0175f ;

    float mx = mag.m.x * 0.00014f;       // mGauss (±4 gauss FS)
    float my = mag.m.y * 0.00014f;
    float mz = mag.m.z * 0.00014f;

    // ---------- SALIDA POR USB (opcional) ----------
    Serial.print(ax, 3); Serial.print(",");
    Serial.print(ay, 3); Serial.print(",");
    Serial.print(az, 3); Serial.print(",");

    Serial.print(gx, 3); Serial.print(",");
    Serial.print(gy, 3); Serial.print(",");
    Serial.print(gz, 3); Serial.print(",");

    Serial.print(mx, 3); Serial.print(",");
    Serial.print(my, 3); Serial.print(",");
    Serial.print(mz, 3); Serial.print(",");

    Serial.print(pressure, 2); Serial.print(",");
    Serial.print(temperature, 2); Serial.print(",");
    Serial.println(altitude, 2);

    // ---------- MISMO CSV POR BLUETOOTH ----------
    BTSerial.print(ax, 3); BTSerial.print(",");
    BTSerial.print(ay, 3); BTSerial.print(",");
    BTSerial.print(az, 3); BTSerial.print(",");

    BTSerial.print(gx, 3); BTSerial.print(",");
    BTSerial.print(gy, 3); BTSerial.print(",");
    BTSerial.print(gz, 3); BTSerial.print(",");

    BTSerial.print(mx, 3); BTSerial.print(",");
    BTSerial.print(my, 3); BTSerial.print(",");
    BTSerial.print(mz, 3); BTSerial.print(",");

    BTSerial.print(pressure, 2); BTSerial.print(",");
    BTSerial.print(temperature, 2); BTSerial.print(",");
    BTSerial.println(altitude, 2);
  }
}

