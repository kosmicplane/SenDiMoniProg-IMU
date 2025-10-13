#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

#include <BluetoothSerial.h> // ESP32 Bluetooth Serial

// IMU objects
LSM6 imu;  // Accelerometer + Gyro
LIS3MDL mag; // Magnetometer (not used here)

BluetoothSerial SerialBT; // Bluetooth Serial object

// Calibration values (from your Python calibration output)
float accel_bias[3] = { -0.002109f, -0.008548f, 0.001136f };
float accel_cal[3][3] = {
  { 1.015680055f, 0.001596284f, -0.001487567f },
  { 0.001596284f, 1.012648665f, -0.001156703f },
  { -0.001487567f, -0.001156703f, 0.990847596f }
};

// Moving average filter parameters
#define FILTER_WINDOW 10
float ax_buf[FILTER_WINDOW], ay_buf[FILTER_WINDOW], az_buf[FILTER_WINDOW];
int buf_idx = 0;

// Simple 1D Kalman filter for each axis
struct Kalman1D {
  float x; // state estimate
  float p; // estimate covariance
  float q; // process noise
  float r; // measurement noise
  Kalman1D(float q_=0.001f, float r_=0.01f) : x(0), p(1), q(q_), r(r_) {}
  float update(float measurement) {
    // Prediction update
    p += q;
    // Measurement update
    float k = p / (p + r);
    x += k * (measurement - x);
    p *= (1 - k);
    return x;
  }
};

Kalman1D kf_x(0.0005f, 0.005f), kf_y(0.0005f, 0.005f), kf_z(0.0005f, 0.005f);

// Function to apply moving average filter  
void addSample(float x, float y, float z, float* fx, float* fy, float* fz) {
  ax_buf[buf_idx] = x;
  ay_buf[buf_idx] = y;
  az_buf[buf_idx] = z;
  buf_idx = (buf_idx + 1) % FILTER_WINDOW;

  *fx = 0; *fy = 0; *fz = 0;
  for (int i = 0; i < FILTER_WINDOW; i++) {
    *fx += ax_buf[i];
    *fy += ay_buf[i];
    *fz += az_buf[i];
  }
  *fx /= FILTER_WINDOW;
  *fy /= FILTER_WINDOW;
  *fz /= FILTER_WINDOW;
}

// Function to apply calibration
void calibrateAccel(float raw[3], float corrected[3]) {
  float temp[3];
  // Subtract bias
  temp[0] = raw[0] - accel_bias[0];
  temp[1] = raw[1] - accel_bias[1];
  temp[2] = raw[2] - accel_bias[2];

  // Multiply by calibration matrix
  corrected[0] = accel_cal[0][0] * temp[0] + accel_cal[0][1] * temp[1] + accel_cal[0][2] * temp[2];
  corrected[1] = accel_cal[1][0] * temp[0] + accel_cal[1][1] * temp[1] + accel_cal[1][2] * temp[2];
  corrected[2] = accel_cal[2][0] * temp[0] + accel_cal[2][1] * temp[1] + accel_cal[2][2] * temp[2];
}

void setup() {

  Serial.begin(115200);
  SerialBT.begin("ESP32-Accel"); // Bluetooth device name
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect LSM6 accelerometer/gyro!");
    while (1);
  }
  imu.enableDefault();

  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL magnetometer!");
  }
  mag.enableDefault();

  // Initialize filter buffers
  for (int i = 0; i < FILTER_WINDOW; i++) {
    ax_buf[i] = 0;
    ay_buf[i] = 0;
    az_buf[i] = 0;
  }

  Serial.println("Accelerometer Calibration Test");
  SerialBT.println("Accelerometer Calibration Test");
}

void loop() {
  imu.read();

  // Read raw integer values
  float raw_counts[3];
  raw_counts[0] = imu.a.x;
  raw_counts[1] = imu.a.y;
  raw_counts[2] = imu.a.z;

  // Convert raw to g
  float raw_g[3];
  raw_g[0] = raw_counts[0] * 0.000061f;
  raw_g[1] = raw_counts[1] * 0.000061f;
  raw_g[2] = raw_counts[2] * 0.000061f;

  // Apply calibration (bias and matrix)
  float corrected[3];
  calibrateAccel(raw_counts, corrected);

  // Convert to g's after calibration
  float corrected_g[3];
  corrected_g[0] = corrected[0] * 0.000061f;
  corrected_g[1] = corrected[1] * 0.000061f;
  corrected_g[2] = corrected[2] * 0.000061f;

  // Apply moving average filter
  float filtered[3];
  addSample(corrected_g[0], corrected_g[1], corrected_g[2], &filtered[0], &filtered[1], &filtered[2]);

  // Apply Kalman filter to each axis
  float kalman[3];
  kalman[0] = kf_x.update(filtered[0]);
  kalman[1] = kf_y.update(filtered[1]);
  kalman[2] = kf_z.update(filtered[2]);

  // Print raw IMU values (in g), calibrated (moving average), and calibrated (Kalman) to Serial
  Serial.print("Raw(g): ");
  Serial.print(raw_g[0], 8); Serial.print(", ");
  Serial.print(raw_g[1], 8); Serial.print(", ");
  Serial.print(raw_g[2], 8);
  Serial.print(" | Calib(g): ");
  Serial.print(filtered[0], 8); Serial.print(", ");
  Serial.print(filtered[1], 8); Serial.print(", ");
  Serial.print(filtered[2], 8);
  Serial.print(" | CalibKalman(g): ");
  Serial.print(kalman[0], 8); Serial.print(", ");
  Serial.print(kalman[1], 8); Serial.print(", ");
  Serial.println(kalman[2], 8);

  // // Bluetooth: send all values not text
  // //SerialBT.print("Raw(g): ");
  // SerialBT.print("( ");SerialBT.print(raw_g[0], 8); SerialBT.print(", ");
  // SerialBT.print(raw_g[1], 8); SerialBT.print(", ");
  // SerialBT.print(raw_g[2], 8); SerialBT.print(", ");
  // //SerialBT.print(" | Calib(g): ");
  // SerialBT.print(filtered[0], 8); SerialBT.print(", ");
  // SerialBT.print(filtered[1], 8); SerialBT.print(", ");
  // SerialBT.print(filtered[2], 8); SerialBT.print(", ");
  // //SerialBT.print(" | CalibKalman(g): ");
  // SerialBT.print(kalman[0], 8); SerialBT.print(", ");
  // SerialBT.print(kalman[1], 8); SerialBT.print(", ");
  // SerialBT.print(kalman[2], 8);SerialBT.println("), ");

  //   // Bluetooth: only send calibrated Kalman values
  // SerialBT.print(kalman[0], 8); SerialBT.print(", ");
  // SerialBT.print(kalman[1], 8); SerialBT.print(", ");
  // SerialBT.print(kalman[2], 8);SerialBT.println("), ");

  //  // Bluetooth: send raw avg and calibrated Kalman values
  // SerialBT.print("Raw(g): ");
  // SerialBT.print("( ");SerialBT.print(raw_g[0], 8); SerialBT.print(", ");
  // SerialBT.print(raw_g[1], 8); SerialBT.print(", ");
  // SerialBT.print(raw_g[2], 8); SerialBT.print(", ");
  // SerialBT.print(" | Calib(g): ");
  // SerialBT.print(filtered[0], 8); SerialBT.print(", ");
  // SerialBT.print(filtered[1], 8); SerialBT.print(", ");
  // SerialBT.print(filtered[2], 8); SerialBT.print(", ");
  // SerialBT.print(" | CalibKalman(g): ");
  // SerialBT.print(kalman[0], 8); SerialBT.print(", ");
  // SerialBT.print(kalman[1], 8); SerialBT.print(", ");
  // SerialBT.print(kalman[2], 8);SerialBT.println("), ");

   // Bluetooth: send raw avg and calibrated Kalman values
  SerialBT.print("Raw(g): ");
  SerialBT.print("( ");SerialBT.print(raw_g[0], 8); SerialBT.print(", ");
  SerialBT.print(raw_g[1], 8); SerialBT.print(", ");
  SerialBT.print(raw_g[2], 8); SerialBT.print(", ");
  SerialBT.print(" | CalibKalman(g): ");
  SerialBT.print(kalman[0], 8); SerialBT.print(", ");
  SerialBT.print(kalman[1], 8); SerialBT.print(", ");
  SerialBT.print(kalman[2], 8);SerialBT.println("), ");
  delay(50); // ~20 Hz update rate
}
