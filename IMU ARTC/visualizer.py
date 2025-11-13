#!/usr/bin/env python3
"""
IMU Visualizer (Full Auto Sphere Comparison)
--------------------------------------------
Loads:
  - Latest raw data from /data/samples/imurawdata_*.txt
  - Latest calibration matrices from /calibration_matrices/calibration_matrices_*.txt

Visualizes:
  - Accelerometer (raw vs calibrated)
  - Magnetometer (raw vs calibrated)
  - Combined overlap view
  - Gyroscope (time series)
Also prints and shows sphericity (%) for raw and calibrated datasets.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
import re
import os

# ---------------- PATH CONFIG ----------------
RAW_DIR = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/data/samples")
CAL_DIR = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/calibration_matrices")
# ------------------------------------------------


# === Utility to find latest matching file ===
def find_latest(folder: Path, prefix: str):
    files = sorted(folder.glob(prefix + "_*.txt"))
    if not files:
        raise FileNotFoundError(f"No {prefix} files found in {folder}")
    return files[-1]


# === Load Raw IMU Data ===
def load_raw_data(file_path: Path):
    print(f"[LOAD] Raw data: {file_path.name}")
    valid_rows = []
    with open(file_path, "r", encoding="utf-8") as f:
        next(f, None)
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 12:
                continue
            try:
                nums = [float(x) for x in parts[:12]]
                valid_rows.append(nums)
            except ValueError:
                continue

    data = np.array(valid_rows, dtype=float)
    accel = data[:, 0:3]
    gyro = data[:, 3:6]
    mag = data[:, 6:9]
    return accel, gyro, mag


# === Parse Calibration Matrix File ===
def parse_calibration_file(path: Path):
    print(f"[LOAD] Calibration file: {path.name}")
    text = path.read_text()

    def parse_vector(label):
        match = re.search(rf"\[{label}\] bias:\s*\[([^\]]+)\]", text)
        if match:
            vals = [float(x) for x in match.group(1).split()]
            return np.array(vals)
        return np.zeros(3)

    def parse_matrix(label):
        match = re.search(rf"\[{label}\] T:\s*(\[\[[^\]]+\]\])", text, re.DOTALL)
        if match:
            block = match.group(1)
            rows = re.findall(r"\[([^\]]+)\]", block)
            mat = np.array([[float(x) for x in r.split()] for r in rows])
            return mat
        return np.eye(3)

    acc_bias = parse_vector("ACCEL")
    acc_T = parse_matrix("ACCEL")
    gyro_bias = parse_vector("GYRO")
    mag_bias = parse_vector("MAG")
    mag_T = parse_matrix("MAG")

    return acc_bias, acc_T, gyro_bias, mag_bias, mag_T


# === Apply Calibration ===
def apply_calibration(acc, gyr, mag, acc_bias, acc_T, gyro_bias, mag_bias, mag_T):
    acc_c = (acc - acc_bias) @ acc_T.T
    gyr_c = gyr - gyro_bias
    mag_c = (mag - mag_bias) @ mag_T.T
    return acc_c, gyr_c, mag_c


# === Compute Sphericity ===
def sphericity(points):
    cov = np.cov(points.T)
    eigvals = np.linalg.eigvalsh(cov)
    eigvals = np.clip(eigvals, 1e-9, None)
    spread = np.min(eigvals) / np.max(eigvals)
    rank = np.linalg.matrix_rank(cov)
    score = (rank / 3.0) * np.sqrt(spread) * 100
    return score


# === 3D Scatter Plot Helper ===
def plot_3d(ax, raw, cal, title, sph_raw, sph_cal):
    ax.scatter(raw[:, 0], raw[:, 1], raw[:, 2], s=5, c='r', alpha=0.5, label=f"Raw ({sph_raw:.1f}%)")
    ax.scatter(cal[:, 0], cal[:, 1], cal[:, 2], s=5, c='g', alpha=0.5, label=f"Calibrated ({sph_cal:.1f}%)")
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    ax.set_box_aspect([1, 1, 1])
    ax.grid(True)


# === Main Visualizer ===
def main():
    # Auto-load files
    raw_file = find_latest(RAW_DIR, "imurawdata")
    cal_file = find_latest(CAL_DIR, "calibration_matrices")

    acc, gyr, mag = load_raw_data(raw_file)
    acc_b, acc_T, gyr_b, mag_b, mag_T = parse_calibration_file(cal_file)
    acc_c, gyr_c, mag_c = apply_calibration(acc, gyr, mag, acc_b, acc_T, gyr_b, mag_b, mag_T)

    # Sphericity
    acc_sph_raw, acc_sph_cal = sphericity(acc), sphericity(acc_c)
    mag_sph_raw, mag_sph_cal = sphericity(mag), sphericity(mag_c)

    print(f"[ACCEL] Sphericity: Raw={acc_sph_raw:.1f}%  Cal={acc_sph_cal:.1f}%")
    print(f"[MAG]   Sphericity: Raw={mag_sph_raw:.1f}%  Cal={mag_sph_cal:.1f}%")

    # === Plot ===
    fig = plt.figure(figsize=(14, 10))

    # Accelerometer Sphere
    ax1 = fig.add_subplot(231, projection='3d')
    plot_3d(ax1, acc, acc_c, "Accelerometer (Raw vs Calibrated)", acc_sph_raw, acc_sph_cal)

    # Magnetometer Sphere
    ax2 = fig.add_subplot(232, projection='3d')
    plot_3d(ax2, mag, mag_c, "Magnetometer (Raw vs Calibrated)", mag_sph_raw, mag_sph_cal)

    # Combined overlap
    ax3 = fig.add_subplot(233, projection='3d')
    ax3.scatter(acc_c[:, 0], acc_c[:, 1], acc_c[:, 2], s=5, c='g', alpha=0.5, label="Accel Cal")
    ax3.scatter(mag_c[:, 0], mag_c[:, 1], mag_c[:, 2], s=5, c='b', alpha=0.5, label="Mag Cal")
    ax3.set_title("Combined Calibrated Overlap")
    ax3.legend()
    ax3.set_box_aspect([1, 1, 1])
    ax3.grid(True)

    # Gyroscope X/Y/Z
    t = np.arange(len(gyr))
    ax4 = fig.add_subplot(212)
    ax4.plot(t, gyr[:, 0], 'r--', alpha=0.5)
    ax4.plot(t, gyr[:, 1], 'g--', alpha=0.5)
    ax4.plot(t, gyr[:, 2], 'b--', alpha=0.5)
    ax4.plot(t, gyr_c[:, 0], 'r', label='X Cal')
    ax4.plot(t, gyr_c[:, 1], 'g', label='Y Cal')
    ax4.plot(t, gyr_c[:, 2], 'b', label='Z Cal')
    ax4.set_title("Gyroscope (Raw vs Calibrated)")
    ax4.set_xlabel("Sample")
    ax4.set_ylabel("deg/s")
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
