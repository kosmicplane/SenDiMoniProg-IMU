import serial
import re
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import time

# --- CONFIGURATION ---
SERIAL_PORT = 'COM4'  # Change to your ESP32 Bluetooth COM port
BAUD_RATE = 115200 
WINDOW_SIZE = 200  # Number of points to show in the live plot
NO_DATA_TIMEOUT = 5  # seconds to wait before warning about no data

# --- REGEX FOR PARSING ---
line_re = re.compile(r"Raw\(g\): \( *([\-0-9.eE]+), *([\-0-9.eE]+), *([\-0-9.eE]+), *\| CalibKalman\(g\): *([\-0-9.eE]+), *([\-0-9.eE]+), *([\-0-9.eE]+)\)")

# --- DATA BUFFERS ---
raw_x = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
raw_y = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
raw_z = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
cal_x = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
cal_y = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
cal_z = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)

def parse_line(line):
    m = line_re.search(line)
    if m:
        return [float(m.group(i)) for i in range(1, 7)]
    return None

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Could not open serial port {SERIAL_PORT}: {e}")
        return
    plt.ion()
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for ax, label in zip(axs, ["Raw (g)", "Calibrated (g)"]):
        ax.set_ylabel(label)
        ax.set_ylim(-2, 2)
        ax.grid(True)
    axs[1].set_xlabel("Samples (live)")
    l_rawx, = axs[0].plot([], [], 'r-', label='X')
    l_rawy, = axs[0].plot([], [], 'g-', label='Y')
    l_rawz, = axs[0].plot([], [], 'b-', label='Z')
    l_calx, = axs[1].plot([], [], 'r-', label='X')
    l_caly, = axs[1].plot([], [], 'g-', label='Y')
    l_calz, = axs[1].plot([], [], 'b-', label='Z')
    axs[0].legend(loc='upper right')
    axs[1].legend(loc='upper right')
    plt.tight_layout()

    last_data_time = time.time()
    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            data = parse_line(line)
            if data:
                rx, ry, rz, cx, cy, cz = data
                raw_x.append(rx)
                raw_y.append(ry)
                raw_z.append(rz)
                cal_x.append(cx)
                cal_y.append(cy)
                cal_z.append(cz)
                last_data_time = time.time()
            # Always update plots, even if no new data
            l_rawx.set_data(range(len(raw_x)), list(raw_x))
            l_rawy.set_data(range(len(raw_y)), list(raw_y))
            l_rawz.set_data(range(len(raw_z)), list(raw_z))
            l_calx.set_data(range(len(cal_x)), list(cal_x))
            l_caly.set_data(range(len(cal_y)), list(cal_y))
            l_calz.set_data(range(len(cal_z)), list(cal_z))
            axs[0].set_xlim(0, WINDOW_SIZE)
            axs[1].set_xlim(0, WINDOW_SIZE)
            plt.pause(0.01)
            time.sleep(0.01)
            # Warn if no data for a while
            if time.time() - last_data_time > NO_DATA_TIMEOUT:
                print(f"No data received for {NO_DATA_TIMEOUT} seconds. Check connection and COM port.")
                last_data_time = time.time()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
