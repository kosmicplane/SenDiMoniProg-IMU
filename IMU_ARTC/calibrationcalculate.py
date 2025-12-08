import numpy as np
import csv
import re
from pathlib import Path
from tkinter import Tk, filedialog

# ---------------------------------------------------------
# USER PATHS
# ---------------------------------------------------------
from pathlib import Path
from tkinter import Tk, filedialog

root = Tk()
root.withdraw()

raw_path = filedialog.askopenfilename(
    title="選擇 RAW 檔案",
    filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
)
cal_path = filedialog.askopenfilename(
    title="選擇校正矩陣檔案",
    filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
)

root.destroy()

if not raw_path or not cal_path:
    raise SystemExit("未選擇完整檔案，程式結束。")

RAW_FILE = Path(raw_path)
CAL_FILE = Path(cal_path)


# SAVE HERE (fixed output directory)
OUTPUT_DIR = Path(r"/home/badkitten/Desktop/SenDiMoniProg-IMU/IMU_ARTC/data/calibrated_output")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

OUTPUT_FILE = OUTPUT_DIR / (RAW_FILE.stem + "_calibrated.txt")

# ---------------------------------------------------------
# PARSE CALIBRATION FILE
# ---------------------------------------------------------
def parse_vector(line):
    nums = re.findall(r"[-+]?\d*\.\d+|\d+", line)
    nums = [float(x) for x in nums]
    return np.array(nums[:3], dtype=float)

def parse_matrix(lines):
    M = []
    for ln in lines:
        nums = re.findall(r"[-+]?\d*\.\d+|\d+", ln)
        nums = [float(x) for x in nums]
        M.append(nums[:3])
    return np.array(M, dtype=float)

with open(CAL_FILE, "r") as f:
    text = f.read()

acc_bias = parse_vector(re.search(r"\[ACCEL\] bias:\s*\[(.*?)\]", text).group(1))
gyro_bias = parse_vector(re.search(r"\[GYRO\] bias:\s*\[(.*?)\]", text).group(1))
mag_bias = parse_vector(re.search(r"\[MAG\] bias:\s*\[(.*?)\]", text).group(1))

acc_T_block = re.findall(r"\[ACCEL\] T:\s*\[(.*?)\]\s*\[(.*?)\]\s*\[(.*?)\]", text, re.S)[0]
mag_T_block = re.findall(r"\[MAG\] T:\s*\[(.*?)\]\s*\[(.*?)\]\s*\[(.*?)\]", text, re.S)[0]

acc_T = parse_matrix(acc_T_block)
mag_T = parse_matrix(mag_T_block)

print("Loaded calibration:")
print("ACC Bias:", acc_bias)
print("ACC T:\n", acc_T)
print("GYRO Bias:", gyro_bias)
print("MAG Bias:", mag_bias)
print("MAG T:\n", mag_T)

# ---------------------------------------------------------
# READ RAW DATA
# ---------------------------------------------------------
raw_data = []
with open(RAW_FILE, "r") as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        if len(row) < 12:
            continue
        raw_data.append(row)

# ---------------------------------------------------------
# PROCESS + CALIBRATE
# ---------------------------------------------------------
output_rows = []
for row in raw_data:

    ts = row[0]
    ax, ay, az = map(float, row[1:4])
    gx, gy, gz = map(float, row[4:7])
    mx, my, mz = map(float, row[7:10])
    alt = row[10]
    rel_alt = row[11]

    # Convert to vectors
    acc_raw = np.array([ax, ay, az], dtype=float)
    gyro_raw = np.array([gx, gy, gz], dtype=float)
    mag_raw = np.array([mx, my, mz], dtype=float)

    # Apply calibration
    acc_cal = acc_T @ (acc_raw - acc_bias)
    gyro_cal = gyro_raw - gyro_bias
    mag_cal = mag_T @ (mag_raw - mag_bias)

    output_rows.append([
        ts,
        f"{acc_cal[0]:.6f}", f"{acc_cal[1]:.6f}", f"{acc_cal[2]:.6f}",
        f"{gyro_cal[0]:.6f}", f"{gyro_cal[1]:.6f}", f"{gyro_cal[2]:.6f}",
        f"{mag_cal[0]:.6f}", f"{mag_cal[1]:.6f}", f"{mag_cal[2]:.6f}",
        alt, rel_alt
    ])

# ---------------------------------------------------------
# SAVE CALIBRATED FILE
# ---------------------------------------------------------
with open(OUTPUT_FILE, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow([
        "timestamp",
        "acc_x_cal", "acc_y_cal", "acc_z_cal",
        "gyro_x_cal", "gyro_y_cal", "gyro_z_cal",
        "mag_x_cal", "mag_y_cal", "mag_z_cal",
        "altitude", "relative_altitude"
    ])
    w.writerows(output_rows)

print(f"\nSaved calibrated data to:\n{OUTPUT_FILE}")
