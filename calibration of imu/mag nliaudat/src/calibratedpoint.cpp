#include <Arduino.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <math.h>

LIS3MDL mag;

// ======= Calibration parameters from calibrate.py =========
float hard_iron_bias[3] = {
   -43.745391,
   23.743175,
   -78.902552
};

double soft_iron[3][3] = {
    {  1.534875,  0.021354, 0.000017 },
    {  0.021354,  1.462022,  0.035572 },
    {  0.000017,  0.035572,  1.655917 }
};
// =========================================================

void setup() {
  Serial.begin(230400);
  Wire.begin();

  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL!");
    while (1);
  }
  mag.enableDefault();

  // LIS3MDL default ±4 gauss → 0.014 µT/LSB
  mag.writeReg(LIS3MDL::CTRL_REG2, 0x00);

  Serial.println("Magnetometer initialized with calibration!");
}

void loop() {
  mag.read();

  // Convert raw counts to µT
  float raw[3] = {
    mag.m.x * 0.014f,
    mag.m.y * 0.014f,
    mag.m.z * 0.014f
  };

  // Step 1: Subtract hard iron biases
  float shifted[3];
  for (int i = 0; i < 3; i++) {
    shifted[i] = raw[i] - hard_iron_bias[i];
  }

  // Step 2: Apply soft iron correction matrix
  float calibrated[3];
  for (int i = 0; i < 3; i++) {
    calibrated[i] = 0;
    for (int j = 0; j < 3; j++) {
      calibrated[i] += soft_iron[i][j] * shifted[j];
    }
  }

  // Step 3: Compute heading (optional)
  float heading = atan2(-calibrated[0], calibrated[1]); // atan2(-X, Y)
  if (heading < 0) heading += 2 * PI;
  float headingDegrees = heading * 180.0f / PI;

  // Step 4: Compute magnitudes
  float raw_mag = sqrt(raw[0]*raw[0] + raw[1]*raw[1] + raw[2]*raw[2]);
  float cal_mag = sqrt(calibrated[0]*calibrated[0] + calibrated[1]*calibrated[1] + calibrated[2]*calibrated[2]);

  // Step 5: Print results
  Serial.printf("Raw (µT): %.2f, %.2f, %.2f | |Raw|= %.2f µT | Calibrated (µT): %.2f, %.2f, %.2f | |Cal|= %.2f µT | Heading: %.2f°\n",
                raw[0], raw[1], raw[2], raw_mag,
                calibrated[0], calibrated[1], calibrated[2], cal_mag,
                headingDegrees);

  delay(100);
}
