// Combined IMU program: accel (LSM6), gyro (LSM6), mag (LIS3MDL), pressure/temp (BMP280)
// Applies calibration (bias + scale/soft-iron matrices), moving average and Kalman (for accel)
// Outputs: accel in g, gyro in deg/s, mag in µT, pressure in hPa, temp in °C, altitude in meters

#include <Arduino.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <BluetoothSerial.h>
#include <LPS.h>
#include <math.h>

// Sensor objects
LSM6 imu;           // Accelerometer + Gyro (Pololu LSM6)
LIS3MDL mag;        // Magnetometer
bool mag_initialized = false; // Track if magnetometer is initialized
LPS ps; // Pressure/Temp sensor
bool ps_initialized = false; // Track if LPS is initialized
BluetoothSerial SerialBT;

// --- Accelerometer calibration (from user) ---
float accel_bias[3] = { -0.002109f, -0.008548f, 0.001136f };
float accel_cal[3][3] = {
  { 1.015680055f, 0.001596284f, -0.001487567f },
  { 0.001596284f, 1.012648665f, -0.001156703f },
  { -0.001487567f, -0.001156703f, 0.990847596f }
};

// --- Gyro calibration (from user) ---
float gyro_bias[3] = { 0.244654f, -0.368901f, -0.514036f };
float gyro_cal[3][3] = {
    { 0.304488704f, 0.002138458f, 0.001554167f },
    { 0.002138458f, 0.292025671f, 0.011048566f },
    { 0.001554167f, 0.011048566f, 0.319499281f }
};

// --- Magnetometer calibration (hard + soft iron) ---
float hard_iron_bias[3] = { -43.745391f, 23.743175f, -78.902552f };
float soft_iron[3][3] = {
    {  1.534875f,  0.021354f, 0.000017f },
    {  0.021354f,  1.462022f,  0.035572f },
    {  0.000017f,  0.035572f,  1.655917f }
};

// Moving average filter parameters (shared for simplicity)
#define FILTER_WINDOW 10
float ax_buf[FILTER_WINDOW], ay_buf[FILTER_WINDOW], az_buf[FILTER_WINDOW];
float gx_buf[FILTER_WINDOW], gy_buf[FILTER_WINDOW], gz_buf[FILTER_WINDOW];
int buf_idx = 0;

// Simple 1D Kalman filter for accel
struct Kalman1D {
  float x; float p; float q; float r;
  Kalman1D(float q_=0.001f, float r_=0.01f) : x(0), p(1), q(q_), r(r_) {}
  float update(float measurement) {
    p += q;
    float k = p / (p + r);
    x += k * (measurement - x);
    p *= (1 - k);
    return x;
  }
};
Kalman1D kf_x(0.0005f, 0.005f), kf_y(0.0005f, 0.005f), kf_z(0.0005f, 0.005f);

// Add sample to moving average buffers (updates both accel and gyro buffers as needed)
void addAccelSample(float x, float y, float z, float* fx, float* fy, float* fz) {
  ax_buf[buf_idx] = x; ay_buf[buf_idx] = y; az_buf[buf_idx] = z;
  // compute average
  *fx = *fy = *fz = 0;
  for (int i = 0; i < FILTER_WINDOW; i++) { *fx += ax_buf[i]; *fy += ay_buf[i]; *fz += az_buf[i]; }
  *fx /= FILTER_WINDOW; *fy /= FILTER_WINDOW; *fz /= FILTER_WINDOW;
}

void addGyroSample(float x, float y, float z, float* fx, float* fy, float* fz) {
  gx_buf[buf_idx] = x; gy_buf[buf_idx] = y; gz_buf[buf_idx] = z;
  *fx = *fy = *fz = 0;
  for (int i = 0; i < FILTER_WINDOW; i++) { *fx += gx_buf[i]; *fy += gy_buf[i]; *fz += gz_buf[i]; }
  *fx /= FILTER_WINDOW; *fy /= FILTER_WINDOW; *fz /= FILTER_WINDOW;
  // advance index only once (shared)
  buf_idx = (buf_idx + 1) % FILTER_WINDOW;
}

// Calibration helpers
void calibrateAccelCounts(const float raw_counts[3], float corrected_counts[3]) {
  float temp[3];
  // subtract bias (bias is in g units from python? user provided small numbers; assume counts bias -> user earlier used g bias, so here we'll treat these as g and convert accordingly)
  // To be safe, subtract bias in counts domain: here user earlier used calibrateAccel(raw_counts, corrected) passing raw_counts (counts) -> so biases are counts too. We'll follow that.
  temp[0] = raw_counts[0] - accel_bias[0];
  temp[1] = raw_counts[1] - accel_bias[1];
  temp[2] = raw_counts[2] - accel_bias[2];
  // apply matrix
  corrected_counts[0] = accel_cal[0][0]*temp[0] + accel_cal[0][1]*temp[1] + accel_cal[0][2]*temp[2];
  corrected_counts[1] = accel_cal[1][0]*temp[0] + accel_cal[1][1]*temp[1] + accel_cal[1][2]*temp[2];
  corrected_counts[2] = accel_cal[2][0]*temp[0] + accel_cal[2][1]*temp[1] + accel_cal[2][2]*temp[2];
}

void calibrateGyroCounts(const float raw_counts[3], float corrected_counts[3]) {
  float temp[3];
  temp[0] = raw_counts[0] - gyro_bias[0];
  temp[1] = raw_counts[1] - gyro_bias[1];
  temp[2] = raw_counts[2] - gyro_bias[2];
  corrected_counts[0] = gyro_cal[0][0]*temp[0] + gyro_cal[0][1]*temp[1] + gyro_cal[0][2]*temp[2];
  corrected_counts[1] = gyro_cal[1][0]*temp[0] + gyro_cal[1][1]*temp[1] + gyro_cal[1][2]*temp[2];
  corrected_counts[2] = gyro_cal[2][0]*temp[0] + gyro_cal[2][1]*temp[1] + gyro_cal[2][2]*temp[2];
}

void calibrateMagUT(const float raw_uT[3], float corrected_uT[3]) {
  float shifted[3];
  for (int i = 0; i < 3; i++) shifted[i] = raw_uT[i] - hard_iron_bias[i];
  for (int i = 0; i < 3; i++) {
    corrected_uT[i] = 0;
    for (int j = 0; j < 3; j++) corrected_uT[i] += soft_iron[i][j] * shifted[j];
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-IMU");
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect LSM6!");
    while (1) delay(1000);
  }
  imu.enableDefault();

  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL!");
    mag_initialized = false;
  } else {
    mag.enableDefault();
    mag.writeReg(LIS3MDL::CTRL_REG2, 0x00); // ±4 gauss
    mag_initialized = true;
  }

  if (!ps.init()) {
    Serial.println("Failed to initialize LPS22DF!");
    ps_initialized = false;
  } else {
    ps.enableDefault();
    ps.writeReg(0x10, 0x60); // CTRL_REG1, 0x60 = 100Hz
    ps_initialized = true;
  }

  // zero buffers
  for (int i = 0; i < FILTER_WINDOW; i++) { ax_buf[i]=ay_buf[i]=az_buf[i]=0; gx_buf[i]=gy_buf[i]=gz_buf[i]=0; }

  Serial.println("Combined IMU started");
  SerialBT.println("Combined IMU started");
}

void loop() {
  imu.read();
  if (mag_initialized) mag.read();

  // ACCEL: raw counts
  float accel_counts[3] = { (float)imu.a.x, (float)imu.a.y, (float)imu.a.z };
  float accel_counts_cal[3];
  calibrateAccelCounts(accel_counts, accel_counts_cal);
  float accel_g_cal[3] = { accel_counts_cal[0]*0.000061f, accel_counts_cal[1]*0.000061f, accel_counts_cal[2]*0.000061f };
  float accel_avg_g[3];
  addAccelSample(accel_g_cal[0], accel_g_cal[1], accel_g_cal[2], &accel_avg_g[0], &accel_avg_g[1], &accel_avg_g[2]);
  float accel_kf_g[3];
  accel_kf_g[0] = kf_x.update(accel_avg_g[0]);
  accel_kf_g[1] = kf_y.update(accel_avg_g[1]);
  accel_kf_g[2] = kf_z.update(accel_avg_g[2]);

  // GYRO: raw counts
  float gyro_counts[3] = { (float)imu.g.x, (float)imu.g.y, (float)imu.g.z };
  float gyro_counts_cal[3];
  calibrateGyroCounts(gyro_counts, gyro_counts_cal);
  float gyro_dps_cal[3] = { gyro_counts_cal[0]*0.00875f, gyro_counts_cal[1]*0.00875f, gyro_counts_cal[2]*0.00875f };
  float gyro_avg_dps[3];
  addGyroSample(gyro_dps_cal[0], gyro_dps_cal[1], gyro_dps_cal[2], &gyro_avg_dps[0], &gyro_avg_dps[1], &gyro_avg_dps[2]);

  // MAG: raw to µT
  float mag_uT_raw[3] = { 0,0,0 };
  float mag_uT_cal[3] = { 0,0,0 };
  if (mag_initialized) {
    mag_uT_raw[0] = mag.m.x * 0.014f;
    mag_uT_raw[1] = mag.m.y * 0.014f;
    mag_uT_raw[2] = mag.m.z * 0.014f;
    calibrateMagUT(mag_uT_raw, mag_uT_cal);
  }

  // LPS: pressure (mbar), temperature (C), altitude (m)
  float pressure = NAN, tempC = NAN, altitude_m = NAN;
  if (ps_initialized) {
    pressure = ps.readPressureMillibars();
    tempC = ps.readTemperatureC();
    altitude_m = ps.pressureToAltitudeMeters(pressure, 1013.25f); // Use sea level pressure or calibrate
  }

  // Print CSV to Serial
  Serial.print(accel_kf_g[0], 6); Serial.print(",");
  Serial.print(accel_kf_g[1], 6); Serial.print(",");
  Serial.print(accel_kf_g[2], 6); Serial.print(",");
  Serial.print(gyro_avg_dps[0], 6); Serial.print(",");
  Serial.print(gyro_avg_dps[1], 6); Serial.print(",");
  Serial.print(gyro_avg_dps[2], 6); Serial.print(",");
  Serial.print(mag_uT_cal[0], 4); Serial.print(",");
  Serial.print(mag_uT_cal[1], 4); Serial.print(",");
  Serial.print(mag_uT_cal[2], 4); Serial.print(",");
  Serial.print(pressure, 2); Serial.print(",");
  Serial.print(altitude_m, 2); Serial.print(",");
  Serial.println(tempC, 2);

  // Print same CSV to Bluetooth
  SerialBT.print(accel_kf_g[0], 6); SerialBT.print(",");
  SerialBT.print(accel_kf_g[1], 6); SerialBT.print(",");
  SerialBT.print(accel_kf_g[2], 6); SerialBT.print(",");
  SerialBT.print(gyro_avg_dps[0], 6); SerialBT.print(",");
  SerialBT.print(gyro_avg_dps[1], 6); SerialBT.print(",");
  SerialBT.print(gyro_avg_dps[2], 6); SerialBT.print(",");
  SerialBT.print(mag_uT_cal[0], 4); SerialBT.print(",");
  SerialBT.print(mag_uT_cal[1], 4); SerialBT.print(",");
  SerialBT.print(mag_uT_cal[2], 4); SerialBT.print(",");
  SerialBT.print(pressure, 2); SerialBT.print(",");
  SerialBT.print(altitude_m, 2); SerialBT.print(",");
  SerialBT.println(tempC, 2);

  delay(50); // ~20 Hz
}
