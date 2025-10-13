import serial

# Replace with your ESP32's Bluetooth serial port (e.g., 'COM5' on Windows)
SERIAL_PORT = 'COM4'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    while True:
        if ser.in_waiting:
            data = ser.readline().decode('utf-8').strip()
            print(f"Received: {data}")
except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()