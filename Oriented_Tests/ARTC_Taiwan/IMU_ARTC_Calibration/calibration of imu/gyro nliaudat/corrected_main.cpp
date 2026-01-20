#include <Wire.h>
#include <BluetoothSerial.h> // ESP32 Bluetooth Serial

BluetoothSerial SerialBT; // Bluetooth Serial object

// Gyro calibration values (replace with your actual values)
float gyro_bias[3] = { 0.244654f, -0.368901f, -0.514036f };
float gyro_cal[3][3] = {
  { 0.304488704f, 0.002138458f, 0.001554167f },
  { 0.002138458f, 0.292025671f, 0.011048566f },
  { 0.001554167f, 0.011048566f, 0.319499281f }
};

// Function to apply calibration (bias and scale matrix)
void calibrateGyro(float raw[3], float corrected[3]) {
    float temp[3];
    // Subtract bias
    temp[0] = raw[0] - gyro_bias[0];
    temp[1] = raw[1] - gyro_bias[1];
    temp[2] = raw[2] - gyro_bias[2];

    // Multiply by calibration matrix
    corrected[0] = gyro_cal[0][0] * temp[0] + gyro_cal[0][1] * temp[1] + gyro_cal[0][2] * temp[2];
    corrected[1] = gyro_cal[1][0] * temp[0] + gyro_cal[1][1] * temp[1] + gyro_cal[1][2] * temp[2];
    corrected[2] = gyro_cal[2][0] * temp[0] + gyro_cal[2][1] * temp[1] + gyro_cal[2][2] * temp[2];
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32-Gyro"); // Bluetooth device name
    Serial.println("Gyro Calibration Test");
    SerialBT.println("Gyro Calibration Test");
}

void loop() {
    // Check if data is available from Bluetooth Serial
    if (SerialBT.available()) {
        // Example: expecting comma-separated floats: gx,gy,gz\n
        String line = SerialBT.readStringUntil('\n');
        float raw[3] = {0, 0, 0};
        int idx = 0;
        int lastPos = 0;
        for (int i = 0; i < line.length(); i++) {
            if (line[i] == ',' || i == line.length() - 1) {
                String val = line.substring(lastPos, (line[i] == ',' ? i : i+1));
                raw[idx++] = val.toFloat();
                lastPos = i + 1;
                if (idx >= 3) break;
            }
        }

        float corrected[3];
        calibrateGyro(raw, corrected);

        // Print calibrated gyro values
        Serial.print("Raw(deg/s): ");
        Serial.print(raw[0], 6); Serial.print(", ");
        Serial.print(raw[1], 6); Serial.print(", ");
        Serial.print(raw[2], 6);
        Serial.print(" | Calibrated(deg/s): ");
        Serial.print(corrected[0], 6); Serial.print(", ");
        Serial.print(corrected[1], 6); Serial.print(", ");
        Serial.println(corrected[2], 6);

        // Send calibrated values over Bluetooth
        SerialBT.print("Calibrated(deg/s): ");
        SerialBT.print(corrected[0], 6); SerialBT.print(", ");
        SerialBT.print(corrected[1], 6); SerialBT.print(", ");
        SerialBT.println(corrected[2], 6);
    }
    delay(10);
}