import serial
import re
import numpy as np
import matplotlib.pyplot as plt

# === CONFIGURATION ===
SERIAL_PORT = "COM9"   # change to your ESP32 COM port
BAUD_RATE = 230400
MAX_POINTS = 2000      # max samples to display

# Regex to capture data from Serial output
pattern = re.compile(
    r"Raw \(µT\): ([\-\d\.]+), ([\-\d\.]+), ([\-\d\.]+) \s*\| \s*\|Raw\|\= [\d\.]+ µT \s*\| "
    r"Calibrated \(µT\): ([\-\d\.]+), ([\-\d\.]+), ([\-\d\.]+) \s*\| \s*\|Cal\|\= [\d\.]+ µT"
)


# Buffers
raw_data = []
cal_data = []

# Create figure & subplots once
plt.ion()
fig, axes = plt.subplots(2, 4, figsize=(16, 8))

# Titles
titles = [
    "Raw XY Projection(µT)", 
    "Raw XZ Projection(µT)", 
    "Raw YZ Projection(µT)", 
    "Raw Combined(µT)",
    "Cal XY Projection(µT)", 
    "Cal XZ Projection(µT)", 
    "Cal YZ Projection", 
    "Cal Combined"
]

for ax, title in zip(axes.flatten(), titles):
    ax.set_title(title)
    ax.grid(True)

def update_plot():
    for ax in axes.flatten():
        ax.cla()

    raw = np.array(raw_data)
    cal = np.array(cal_data)

    if len(raw) == 0: 
        return

    # --- RAW (top row) ---
    # XY
    axes[0,0].scatter(raw[:,0], raw[:,1], c='r', s=5, label="X vs Y")
    axes[0,0].set_title("Raw XY Projection(µT)")
    # XZ
    axes[0,1].scatter(raw[:,0], raw[:,2], c='b', s=5, label="X vs Z")
    axes[0,1].set_title("Raw XZ Projection(µT)")
    # YZ
    axes[0,2].scatter(raw[:,1], raw[:,2], c='g', s=5, label="Y vs Z")
    axes[0,2].set_title("Raw YZ Projection(µT)")
    # Combined
    axes[0,3].scatter(raw[:,0], raw[:,1], c='r', s=5, label="X vs Y")
    axes[0,3].scatter(raw[:,0], raw[:,2], c='b', s=5, label="X vs Z")
    axes[0,3].scatter(raw[:,1], raw[:,2], c='g', s=5, label="Y vs Z")
    axes[0,3].set_title("Raw Combined(µT)")
    axes[0,3].legend(loc="upper right", fontsize=8)

    # --- CALIBRATED (bottom row) ---
    # XY
    axes[1,0].scatter(cal[:,0], cal[:,1], c='r', s=5, label="X vs Y")
    axes[1,0].set_title("Cal XY Projection(µT)")
    # XZ
    axes[1,1].scatter(cal[:,0], cal[:,2], c='b', s=5, label="X vs Z")
    axes[1,1].set_title("Cal XZ Projection(µT)")
    # YZ
    axes[1,2].scatter(cal[:,1], cal[:,2], c='g', s=5, label="Y vs Z")
    axes[1,2].set_title("Cal YZ Projection(µT)")
    # Combined
    axes[1,3].scatter(cal[:,0], cal[:,1], c='r', s=5, label="X vs Y")
    axes[1,3].scatter(cal[:,0], cal[:,2], c='b', s=5, label="X vs Z")
    axes[1,3].scatter(cal[:,1], cal[:,2], c='g', s=5, label="Y vs Z")
    axes[1,3].set_title("Cal Combined(µT)")
    axes[1,3].legend(loc="upper right", fontsize=8)

    plt.tight_layout()
    plt.pause(0.001)

def main():
    global raw_data, cal_data

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("Listening on", SERIAL_PORT)

    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        match = pattern.search(line)
        if match:
            raw = [float(match.group(1)), float(match.group(2)), float(match.group(3))]
            cal = [float(match.group(4)), float(match.group(5)), float(match.group(6))]

            raw_data.append(raw)
            cal_data.append(cal)

            if len(raw_data) > MAX_POINTS:
                raw_data = raw_data[-MAX_POINTS:]
                cal_data = cal_data[-MAX_POINTS:]

            update_plot()

if __name__ == "__main__":
    main()
