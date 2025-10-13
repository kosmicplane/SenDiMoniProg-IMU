#include <Wire.h>
#include <BluetoothSerial.h> // ESP32 Bluetooth Serial
#include <LSM6.h> // Pololu LSM6 library

LSM6 imu;  // Gyro object
BluetoothSerial SerialBT; // Bluetooth Serial object

// Gyro calibration values (replace with your actual values)
float gyro_bias[3] = { 0.244654f, -0.368901f, -0.514036f };
float gyro_cal[3][3] = {
    { 0.304488704f, 0.002138458f, 0.001554167f },
    { 0.002138458f, 0.292025671f, 0.011048566f },
    { 0.001554167f, 0.011048566f, 0.319499281f }
};

#define FILTER_WINDOW 10
float gx_buf[FILTER_WINDOW], gy_buf[FILTER_WINDOW], gz_buf[FILTER_WINDOW];
int buf_idx = 0;

// Moving average filter for gyro
void addSample(float x, float y, float z, float* fx, float* fy, float* fz) {
    gx_buf[buf_idx] = x;
    gy_buf[buf_idx] = y;
    gz_buf[buf_idx] = z;
    buf_idx = (buf_idx + 1) % FILTER_WINDOW;
    *fx = 0; *fy = 0; *fz = 0;
    for (int i = 0; i < FILTER_WINDOW; i++) {
        *fx += gx_buf[i];
        *fy += gy_buf[i];
        *fz += gz_buf[i];
    }
    *fx /= FILTER_WINDOW;
    *fy /= FILTER_WINDOW;
    *fz /= FILTER_WINDOW;
}

// Gyro calibration
void calibrateGyro(float raw[3], float corrected[3]) {
    float temp[3];
    temp[0] = raw[0] - gyro_bias[0];
    temp[1] = raw[1] - gyro_bias[1];
    temp[2] = raw[2] - gyro_bias[2];
    corrected[0] = gyro_cal[0][0] * temp[0] + gyro_cal[0][1] * temp[1] + gyro_cal[0][2] * temp[2];
    corrected[1] = gyro_cal[1][0] * temp[0] + gyro_cal[1][1] * temp[1] + gyro_cal[1][2] * temp[2];
    corrected[2] = gyro_cal[2][0] * temp[0] + gyro_cal[2][1] * temp[1] + gyro_cal[2][2] * temp[2];
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32-Gyro");
    Wire.begin();
    if (!imu.init()) {
        Serial.println("Failed to detect LSM6 gyro!");
        while (1);
    }
    imu.enableDefault();
    for (int i = 0; i < FILTER_WINDOW; i++) {
        gx_buf[i] = 0;
        gy_buf[i] = 0;
        gz_buf[i] = 0;
    }
    Serial.println("Gyro Calibration Test");
    SerialBT.println("Gyro Calibration Test");
}

void loop() {
    imu.read();
    // Read raw gyro values
    float raw_counts[3];
    raw_counts[0] = imu.g.x;
    raw_counts[1] = imu.g.y;
    raw_counts[2] = imu.g.z;

    // Convert raw to deg/s (LSM6: 0.00875 for 245 dps full scale)
    float raw_dps[3];
    raw_dps[0] = raw_counts[0] * 0.00875f;
    raw_dps[1] = raw_counts[1] * 0.00875f;
    raw_dps[2] = raw_counts[2] * 0.00875f;

    // Apply calibration
    float corrected[3];
    calibrateGyro(raw_counts, corrected);

    // Convert to deg/s after calibration
    float corrected_dps[3];
    corrected_dps[0] = corrected[0] * 0.00875f;
    corrected_dps[1] = corrected[1] * 0.00875f;
    corrected_dps[2] = corrected[2] * 0.00875f;

    // Moving average filter
    float filtered[3];
    addSample(corrected_dps[0], corrected_dps[1], corrected_dps[2], &filtered[0], &filtered[1], &filtered[2]);

    // Print raw, calibrated, and filtered gyro values
    Serial.print("Raw(deg/s): ");
    Serial.print(raw_dps[0], 8); Serial.print(", ");
    Serial.print(raw_dps[1], 8); Serial.print(", ");
    Serial.print(raw_dps[2], 8);
    Serial.print(" | Calib(deg/s): ");
    Serial.print(filtered[0], 8); Serial.print(", ");
    Serial.print(filtered[1], 8); Serial.print(", ");
    Serial.println(filtered[2], 8);

    // Bluetooth: send raw and filtered values
    SerialBT.print("Raw(deg/s): ");
    SerialBT.print("( ");SerialBT.print(raw_dps[0], 8); SerialBT.print(", ");
    SerialBT.print(raw_dps[1], 8); SerialBT.print(", ");
    SerialBT.print(raw_dps[2], 8); SerialBT.print(", ");
    SerialBT.print(" | Calib(deg/s): ");
    SerialBT.print(filtered[0], 8); SerialBT.print(", ");
    SerialBT.print(filtered[1], 8); SerialBT.print(", ");
    SerialBT.print(filtered[2], 8);SerialBT.println(")");
    delay(50); // ~20 Hz update rate
}
