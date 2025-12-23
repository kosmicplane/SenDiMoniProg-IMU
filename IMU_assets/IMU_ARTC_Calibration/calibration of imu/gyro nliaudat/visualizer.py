import serial
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

# --- CONFIGURATION ---
SERIAL_PORT = 'COM4'  # Change to your Bluetooth serial port
BAUD_RATE = 115200
WINDOW_SIZE = 100      # Number of points to show in the plot

# --- REGEX FOR PARSING ---
pattern = re.compile(
    r'Raw\(deg/s\): \( ([^,]+), ([^,]+), ([^,]+),  \| Calib\(deg/s\): ([^,]+), ([^,]+), ([^,]+)\)'
)

# --- DATA BUFFERS ---
raw_x, raw_y, raw_z = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE), deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE), deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
calib_x, calib_y, calib_z = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE), deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE), deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)

# --- SETUP SERIAL ---
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


# --- SETUP 3D PLOT ---
plt.ion()
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
raw_quiver = None
calib_quiver = None
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([-5, 5])
ax.set_xlabel('X (deg/s)')
ax.set_ylabel('Y (deg/s)')
ax.set_zlabel('Z (deg/s)')
ax.set_title('Gyro 3D Vector Visualization (Raw: Blue, Calibrated: Red)')

def update_plot(rx, ry, rz, cx, cy, cz):
    global raw_quiver, calib_quiver
    ax.cla()
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([-5, 5])
    ax.set_xlabel('X (deg/s)')
    ax.set_ylabel('Y (deg/s)')
    ax.set_zlabel('Z (deg/s)')
    ax.set_title('Gyro 3D Vector Visualization (Raw: Blue, Calibrated: Red)')
    # Draw raw vector (blue)
    ax.quiver(0, 0, 0, rx, ry, rz, color='b', label='Raw', arrow_length_ratio=0.2)
    # Draw calibrated vector (red)
    ax.quiver(0, 0, 0, cx, cy, cz, color='r', label='Calibrated', arrow_length_ratio=0.2)
    ax.legend()
    plt.pause(0.01)

print("Starting 3D visualization. Press Ctrl+C to exit.")
try:
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        match = pattern.search(line)
        if match:
            rx, ry, rz, cx, cy, cz = map(float, match.groups())
            update_plot(rx, ry, rz, cx, cy, cz)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    plt.ioff()
    plt.show()