import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ===========================
# Serial Settings
# ===========================
PORT = "COM9"      # Change to your ESP32 port
BAUD = 230400
DURATION = 120       # seconds to record
OUTPUT_FILE = "mag_out.txt"

# ===========================
# Open Serial
# ===========================
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # wait for ESP32 reset

print("Recording magnetometer data...")
data = []

start_time = time.time()
while (time.time() - start_time) < DURATION:
    line = ser.readline().decode("utf-8").strip()
    if "," in line:
        try:
            x, y, z = map(float, line.split(","))
            data.append([x, y, z])
        except:
            pass

ser.close()
print(f"Finished recording {len(data)} samples.")

# ===========================
# Save to mag_out.txt
# ===========================
np.savetxt(OUTPUT_FILE, data, fmt="%.6f", delimiter=",")
print(f"Saved data to {OUTPUT_FILE}")

# ===========================
# Plot Data
# ===========================
data = np.array(data)
x, y, z = data[:, 0], data[:, 1], data[:, 2]

fig = plt.figure(figsize=(12, 6))

# --- 3D Plot ---
ax = fig.add_subplot(231, projection="3d")
ax.plot(x, y, z, ".", markersize=2)
ax.set_title("Raw Magnetometer 3D")
ax.set_xlabel("X (µT)")
ax.set_ylabel("Y (µT)")
ax.set_zlabel("Z (µT)")

# --- XY ---
ax = fig.add_subplot(232)
ax.plot(x, y, "r.", markersize=2)
ax.set_title("XY Projection")
ax.set_xlabel("X (µT)")
ax.set_ylabel("Y (µT)")

# --- XZ ---
ax = fig.add_subplot(233)
ax.plot(x, z, "g.", markersize=2)
ax.set_title("XZ Projection")
ax.set_xlabel("X (µT)")
ax.set_ylabel("Z (µT)")

# --- YZ ---
ax = fig.add_subplot(234)
ax.plot(y, z, "b.", markersize=2)
ax.set_title("YZ Projection")
ax.set_xlabel("Y (µT)")
ax.set_ylabel("Z (µT)")

# --- Combined View ---
ax = fig.add_subplot(235)
ax.plot(x, y, "r.", markersize=2, label="XY plane")
ax.plot(x, z, "g.", markersize=2, label="XZ plane")
ax.plot(y, z, "b.", markersize=2, label="YZ plane")
ax.legend()
ax.set_title("XYZ Combined")

plt.tight_layout()
plt.show()
