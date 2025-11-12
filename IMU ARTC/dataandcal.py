#!/usr/bin/env python3
"""
Unified IMU Data Collection + Calibration Tool
----------------------------------------------

Options:
  1) Three-stage calibration data collection (gyro → accel → mag)
  2) Continuous recording mode
  3) Calibration analysis (auto-normalized ellipsoid fit + coverage quality)
     - Saves calibration matrices & biases in /calibration_matrices
     - Does NOT stream live data
"""

import os
import sys
import csv
import time
import threading
import serial
import numpy as np
from datetime import datetime
from pathlib import Path
from scipy import linalg

# ===== USER SETTINGS =====
PORT = "COM3"
BAUD = 230400
TIMEOUT = 1.0
DEFAULT_N_SAMPLES = 1090

SAVE_DIR = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/samples2")
ROOT_DIR = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/data/samples")
CAL_DIR = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/calibration_matrices")
CAL_DIR.mkdir(parents=True, exist_ok=True)

EXPECTED_FIELDS = 12
HZ = 100
CYCLE_SAMPLES = 60 * HZ

# Fixed field indices
AX, AY, AZ = 0, 1, 2
GX, GY, GZ = 3, 4, 5
MX, MY, MZ = 6, 7, 8
ALT_IDX = 11

# ===== Stream State =====
class StreamState:
    def __init__(self):
        self.saving = False
        self.quit_flag = False
        self.current_file = None
        self.filename = None
        self.filepath = None
        self.lock = threading.Lock()
        self.sample_idx = 0
        self.first_alt = None

SS = StreamState()

# ===== Shared Utilities =====
def open_serial():
    print(f"[SER] Opening {PORT} @ {BAUD} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"[ERR] Could not open serial port: {e}")
        sys.exit(1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def safe_float(s):
    try:
        return float(s)
    except Exception:
        return None

def parse_line_to_parts(line: bytes):
    if not line:
        return None
    try:
        text = line.decode("utf-8", errors="ignore").strip()
    except Exception:
        text = line.decode(errors="ignore").strip()
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    return parts if len(parts) >= EXPECTED_FIELDS else None

def ask_int(prompt, default):
    while True:
        s = input(f"{prompt} (default {default}): ").strip()
        if s == "":
            return default
        try:
            v = int(s)
            if v > 0:
                return v
        except ValueError:
            pass
        print("Invalid number, try again.")

# ===== Coverage Metric =====
def coverage_quality(samples):
    if len(samples) < 10:
        return 0.0
    samples = np.array(samples, dtype=float)
    cov = np.cov(samples.T)
    rank = np.linalg.matrix_rank(cov)
    eigvals = np.linalg.eigvalsh(cov)
    spread_ratio = np.min(eigvals) / np.max(eigvals) if np.max(eigvals) > 0 else 0
    return (rank / 3.0) * (spread_ratio ** 0.5) * 100

# ===== Data Collection =====
def collect_n(ser, indices, header, n_samples):
    rows, count = [], 0
    ser.reset_input_buffer()
    while count < n_samples:
        parts = parse_line_to_parts(ser.readline())
        if parts is None:
            continue
        vals = [safe_float(parts[i]) for i in indices]
        if any(v is None for v in vals):
            continue
        rows.append(tuple(vals))
        count += 1
        if count % 100 == 0:
            print(f"Collected {count}/{n_samples}")
    return header, rows

def save_txt(path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        if header:
            w.writerow(header)
        w.writerows(rows)
    print(f"Saved {len(rows)} rows → {path}")

def mode_three_stage(ser):
    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    # Gyroscope
    print("\n[Stage 1: Gyroscope gx, gy, gz]")
    if input("Press s to start collection (q to quit): ").strip().lower() != "s":
        return
    n = ask_int("Enter number of samples", DEFAULT_N_SAMPLES)
    fname = f"gyro_raw_data_{datetime.now():%Y%m%d_%H%M%S}.txt"
    header, rows = collect_n(ser, [GX, GY, GZ], ["gx", "gy", "gz"], n)
    save_txt(SAVE_DIR / fname, header, rows)

    # Accelerometer
    print("\n[Stage 2: Accelerometer ax, ay, az] - 6 face mode")
    samples_per_face = ask_int("Enter samples per face", 25)
    fname = f"accel_raw_data_{datetime.now():%Y%m%d_%H%M%S}.txt"
    all_rows = []
    for face in range(1, 7):
        input(f"Place on face {face}, press Enter to collect...")
        header, rows = collect_n(ser, [AX, AY, AZ], ["ax", "ay", "az"], samples_per_face)
        all_rows.extend(rows)
    save_txt(SAVE_DIR / fname, ["ax", "ay", "az"], all_rows)

    # Magnetometer
    print("\n[Stage 3: Magnetometer mx, my, mz]")
    while True:
        k = input("Press s to start or e to end: ").strip().lower()
        if k == "e":
            break
        n = ask_int("Enter number of samples", DEFAULT_N_SAMPLES)
        fname = f"mag_raw_data_{datetime.now():%Y%m%d_%H%M%S}.txt"
        header, rows = collect_n(ser, [MX, MY, MZ], ["mx", "my", "mz"], n)
        save_txt(SAVE_DIR / fname, header, rows)
        quality = coverage_quality(rows)
        print(f"[MAG] Coverage: {quality:.1f}% → {'Good ✅' if quality>=70 else 'Fair ⚠' if quality>=40 else 'Poor ❌'}")

# ===== Continuous Stream Mode =====
def mode_stream(ser):
    """Automatic continuous recording mode with timestamped filename."""
    os.makedirs(ROOT_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"imurawdata_{timestamp}.txt"
    path = Path(ROOT_DIR) / filename

    print(f"\n[INFO] Continuous recording started automatically.")
    print(f"[INFO] File: {path}")
    print(f"[INFO] Press Ctrl+C to stop.\n")

    try:
        with open(path, "w", encoding="utf-8", buffering=1) as f:
            f.write("ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temperature,altitude\n")
            start_time = time.time()
            count = 0

            while True:
                line = ser.readline()
                if not line:
                    continue
                parts = parse_line_to_parts(line)
                if parts:
                    f.write(",".join(parts[:EXPECTED_FIELDS]) + "\n")
                    count += 1
                    #time.sleep(0.05)  # <-- slows down loop to ~20 samples per second



                    # print every 500th line to console
                    if count % 500 == 0:
                        elapsed = time.time() - start_time
                        print(f"[{count}] samples saved ({elapsed:.1f}s elapsed)")

    except KeyboardInterrupt:
        print("\n[STOP] Recording stopped by user.")
    except Exception as e:
        print(f"[ERR] {e}")
    finally:
        print(f"[INFO] File saved to: {path}")


# ===== Calibration Analysis =====
def latest(prefix):
    files = sorted(SAVE_DIR.glob(prefix + "_*.txt"))
    return files[-1] if files else None

def load_three_col(fname):
    return np.loadtxt(fname, delimiter=",", skiprows=1).reshape(-1, 3)

def ellipsoid_fit(samples):
    s = samples.T
    D = np.array([
        s[0]**2, s[1]**2, s[2]**2,
        2*s[1]*s[2], 2*s[0]*s[2], 2*s[0]*s[1],
        2*s[0], 2*s[1], 2*s[2], np.ones_like(s[0])
    ])
    S = D @ D.T
    S11, S12 = S[:6, :6], S[:6, 6:]
    S21, S22 = S[6:, :6], S[6:, 6:]
    C = np.array([
        [-1, 1, 1, 0, 0, 0],
        [1, -1, 1, 0, 0, 0],
        [1, 1, -1, 0, 0, 0],
        [0, 0, 0, -4, 0, 0],
        [0, 0, 0, 0, -4, 0],
        [0, 0, 0, 0, 0, -4]
    ])
    E = np.linalg.inv(C) @ (S11 - S12 @ np.linalg.inv(S22) @ S21)
    ew, ev = np.linalg.eig(E)
    v1 = ev[:, np.argmax(ew)]
    if v1[0] < 0: v1 = -v1
    v2 = -np.linalg.inv(S22) @ S21 @ v1
    M = np.array([[v1[0],v1[5],v1[4]],[v1[5],v1[1],v1[3]],[v1[4],v1[3],v1[2]]])
    n = np.array([v2[0],v2[1],v2[2]]); d = v2[3]
    return M, n, d

def ellipsoid_calibrate_autoscale(samples):
    quality = coverage_quality(samples)
    if quality < 70:
        bias = np.mean(samples, axis=0); T = np.eye(3)
        print(f"  ⚠ Coverage = {quality:.1f}% ❌ insufficient → offset-only")
        return bias, T, quality
    try:
        M, n, d = ellipsoid_fit(samples)
        Minv = np.linalg.inv(M)
        bias = (-0.5 * Minv @ n).reshape(3)
        k = 0.25 * (n.T @ Minv @ n) - d
        k = max(abs(k), 1e-8)
        A = M / k
        eigvals, eigvecs = np.linalg.eigh(A)
        eigvals = np.clip(eigvals, 1e-9, None)
        A_pd = eigvecs @ np.diag(eigvals) @ eigvecs.T
        L = np.real_if_close(linalg.sqrtm(A_pd))
        v = (L @ (samples - bias).T).T
        norms = np.linalg.norm(v, axis=1)
        scale = np.median(norms[norms > 0]) / np.mean(norms) if np.mean(norms) > 1e-6 else 1.0
        T = L * scale
        print(f"  ✅ Coverage = {quality:.1f}% good → ellipsoid fit applied.")
        return bias, T, quality
    except Exception:
        bias = np.mean(samples, axis=0); T = np.eye(3)
        print(f"  ⚠ Coverage = {quality:.1f}% ❌ fit failed → offset-only")
        return bias, T, quality

def mode_calibrate():
    print("\n[STEP 1] Finding latest raw files...")
    accel = load_three_col(latest("accel_raw_data"))
    gyro = load_three_col(latest("gyro_raw_data"))
    mag = load_three_col(latest("mag_raw_data"))

    print("\n[STEP 2] Calibrating...")
    acc_bias, acc_T, acc_q = ellipsoid_calibrate_autoscale(accel)
    gyro_bias = np.mean(gyro, axis=0)
    mag_bias, mag_T, mag_q = ellipsoid_calibrate_autoscale(mag)

    np.set_printoptions(precision=5, suppress=False, floatmode='fixed')

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = CAL_DIR / f"calibration_matrices_{ts}.txt"
    with open(path, "w") as f:
        f.write("=== IMU Calibration Matrices ===\n")
        f.write(f"[ACCEL] bias: {acc_bias}\n")
        f.write(f"[ACCEL] T:\n{acc_T}\n")
        f.write(f"[GYRO] bias: {gyro_bias}\n")
        f.write(f"[MAG] bias: {mag_bias}\n")
        f.write(f"[MAG] T:\n{mag_T}\n")
        f.write(f"[ACCEL] Coverage: {acc_q:.1f}%\n[MAG] Coverage: {mag_q:.1f}%\n")
    print(f"\n✅ Calibration saved to: {path}")

# ===== Entry Menu =====
def main():
    ser = open_serial()
    try:
        print("\n=== Mode Selection ===")
        print("1) Three-stage calibration data collection")
        print("2) Continuous recording mode")
        print("3) IMU Calibration (auto-normalized, no live stream)")
        mode = input("Enter 1, 2, or 3: ").strip()
        if mode == "1":
            mode_three_stage(ser)
        elif mode == "2":
            mode_stream(ser)
        elif mode == "3":
            ser.close()
            mode_calibrate()
        else:
            print("Invalid choice.")
    finally:
        try: ser.close()
        except: pass
        print("[INFO] Exiting.")

if __name__ == "__main__":
    main()
