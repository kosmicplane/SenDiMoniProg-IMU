import serial

# Change 'COM3' to your ESP32's serial port (e.g., 'COM4', '/dev/ttyUSB0', etc.)
SERIAL_PORT = 'COM4'
BAUD_RATE = 115200

OUTPUT_FILE = 'gyro_cal1.txt'

try:
    print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud...")
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5) as ser, open(OUTPUT_FILE, 'a') as outfile:
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud. Logging to {OUTPUT_FILE}")
        import time
        last_data_time = time.time()
        while True:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(f"[RECEIVED] {line}")
                outfile.write(line + '\n')
                outfile.flush()
                print(f"[SAVED] {line}")
                last_data_time = time.time()
            elif time.time() - last_data_time > 5:
                print("[WARNING] No data received in the last 5 seconds. Retrying...")
                last_data_time = time.time()
except serial.SerialException as e:
    print(f"[ERROR] Serial error: {e}")
except KeyboardInterrupt:
    print("[INFO] Exiting...")