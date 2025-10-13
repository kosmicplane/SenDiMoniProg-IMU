import re
import numpy as np
import matplotlib.pyplot as plt

# File path
filename = "accel_data_cal7temp.txt"

# Regular expression to extract values
pattern = re.compile(
    r"Raw\(g\): \(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*\| CalibKalman\(g\):\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)"
)

raw_x, raw_y, raw_z = [], [], []
cal_x, cal_y, cal_z = [], [], []

with open(filename, "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            rx, ry, rz, cx, cy, cz = map(float, match.groups())
            raw_x.append(rx)
            raw_y.append(ry)
            raw_z.append(rz)
            cal_x.append(cx)
            cal_y.append(cy)
            cal_z.append(cz)

# Convert to numpy arrays for easier handling
raw_x, raw_y, raw_z = map(np.array, (raw_x, raw_y, raw_z))
cal_x, cal_y, cal_z = map(np.array, (cal_x, cal_y, cal_z))

# Plotting
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Raw values (top)
axs[0].plot(raw_x, label='Raw X', color='r')
axs[0].plot(raw_y, label='Raw Y', color='g')
axs[0].plot(raw_z, label='Raw Z', color='b')
axs[0].set_title('Raw Accelerometer Values (g)')
axs[0].set_ylabel('g')
axs[0].legend()
axs[0].grid(True)

# Calibrated values (bottom)
axs[1].plot(cal_x, label='Calibrated X', color='r', linestyle='--')
axs[1].plot(cal_y, label='Calibrated Y', color='g', linestyle='--')
axs[1].plot(cal_z, label='Calibrated Z', color='b', linestyle='--')
axs[1].set_title('Calibrated Accelerometer Values (g)')
axs[1].set_xlabel('Sample')
axs[1].set_ylabel('g')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()