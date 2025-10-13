#include <Arduino.h>

#include <Wire.h>
#include <LIS3MDL.h>
#include <math.h>
#ifdef ESP32
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif

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


#ifdef ESP32
  SerialBT.begin("MagCalibESP32"); // Bluetooth device name
  Serial.println("Bluetooth started. Device name: MagCalibESP32");
  Serial.println("Waiting for Bluetooth client to connect...");
  // Wait for a Bluetooth client to connect
  unsigned long bt_start = millis();
  while (!SerialBT.hasClient()) {
    delay(100);
    if (millis() - bt_start > 20000) { // 20s timeout
      Serial.println("No Bluetooth client connected after 20s, continuing anyway.");
      break;
    }
  }
  if (SerialBT.hasClient()) {
    Serial.println("Bluetooth client connected!");
    SerialBT.println("Connected to ESP32!");
  }
#endif

  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL!");
#ifdef ESP32
    SerialBT.println("Failed to detect LIS3MDL!");
#endif
    while (1);
  }
  mag.enableDefault();

  // LIS3MDL default ±4 gauss → 0.014 µT/LSB
  mag.writeReg(LIS3MDL::CTRL_REG2, 0x00);

  Serial.println("Magnetometer initialized with calibration!");
#ifdef ESP32
  SerialBT.println("Magnetometer initialized with calibration!");
#endif
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

  char outbuf[256];
  snprintf(outbuf, sizeof(outbuf),
    "Raw (µT): %.2f, %.2f, %.2f | |Raw|= %.2f µT | Calibrated (µT): %.2f, %.2f, %.2f | |Cal|= %.2f µT | Heading: %.2f°",
    raw[0], raw[1], raw[2], raw_mag,
    calibrated[0], calibrated[1], calibrated[2], cal_mag,
    headingDegrees);
  Serial.println(outbuf);
#ifdef ESP32
  if (SerialBT.hasClient()) {
    SerialBT.println(outbuf);
  }
#endif

  delay(100);
}
