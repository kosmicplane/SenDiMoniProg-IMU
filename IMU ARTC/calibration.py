#!/usr/bin/env python3
"""
IMU Calibration + Live Output (with Coverage Quality Check)
-----------------------------------------------------------

Now includes:
- 3D coverage analysis for accelerometer & magnetometer
- Automatic fallback to offset-only if coverage < 70%
- 5-decimal live calibrated output
"""

import os
import csv
import time
import serial
import argparse
import numpy as np
from pathlib import Path
from datetime import datetime
from scipy import linalg

# ---------------- USER SETTINGS ----------------
RAW_DIR = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/samples")
OUTPUT_DIR = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/calibrated")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

DEFAULT_PORT = "COM3"
BAUD = 230400
EXPECTED_FIELDS = 12
# ------------------------------------------------


def latest(prefix: str) -> Path:
    """Find the latest file matching prefix_*"""
    files = sorted(RAW_DIR.glob(prefix + "_*.txt"))
    if not files:
        print(f"[ERR] No files found for {prefix}_*.txt in {RAW_DIR}")
        raise SystemExit(1)
    return files[-1]


def load_three_col(fname: Path) -> np.ndarray:
    """Load a text file with 3 columns of floats"""
    data = np.loadtxt(fname, delimiter=",", skiprows=1)
    data = np.asarray(data, dtype=float).reshape(-1, 3)
    return data


# ===== Coverage Quality Evaluation =====
def coverage_quality(samples: np.ndarray) -> float:
    """Returns 0–100 coverage quality based on data spread & rank"""
    cov = np.cov(samples.T)
    rank = np.linalg.matrix_rank(cov)
    eigvals = np.linalg.eigvalsh(cov)
    spread_ratio = np.min(eigvals) / np.max(eigvals) if np.max(eigvals) > 0 else 0
    spread_ratio = np.clip(spread_ratio, 0, 1)
    # Combine rank and isotropy to 0–100 score
    score = (rank / 3.0) * (spread_ratio ** 0.5) * 100
    return score


# ===== Ellipsoid Calibration (auto-normalized) =====
def ellipsoid_fit(samples: np.ndarray):
    """Fit an ellipsoid using least squares"""
    s = samples.T
    D = np.array([
        s[0]**2, s[1]**2, s[2]**2,
        2*s[1]*s[2], 2*s[0]*s[2], 2*s[0]*s[1],
        2*s[0], 2*s[1], 2*s[2],
        np.ones_like(s[0])
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
    ], dtype=float)

    E = np.linalg.inv(C) @ (S11 - S12 @ np.linalg.inv(S22) @ S21)
    ew, ev = np.linalg.eig(E)
    v1 = ev[:, np.argmax(ew)]
    if v1[0] < 0:
        v1 = -v1
    v2 = -np.linalg.inv(S22) @ S21 @ v1

    M = np.array([
        [v1[0], v1[5], v1[4]],
        [v1[5], v1[1], v1[3]],
        [v1[4], v1[3], v1[2]]
    ], dtype=float)
    n = np.array([v2[0], v2[1], v2[2]], dtype=float)
    d = float(v2[3])
    return M, n, d


def ellipsoid_calibrate_autoscale(samples: np.ndarray):
    """Perform ellipsoid calibration with stability protection."""
    quality = coverage_quality(samples)

    if quality < 70:
        print(f"  ⚠ Coverage = {quality:.1f}% ❌ insufficient — using offset-only calibration.")
        bias = np.mean(samples, axis=0)
        T = np.eye(3)
        return bias, T, quality

    try:
        M, n, d = ellipsoid_fit(samples)
        Minv = np.linalg.inv(M)
        bias = (-0.5 * Minv @ n).reshape(3)
        k = 0.25 * (n.T @ Minv @ n) - d

        if k <= 1e-8 or not np.isfinite(k):
            print("  [WARN] small k, clamping to safe minimum")
            k = max(abs(k), 1e-8)

        A = M / k
        # ensure positive-definite A
        eigvals, eigvecs = np.linalg.eigh(A)
        eigvals = np.clip(eigvals, 1e-9, None)
        A_pd = eigvecs @ np.diag(eigvals) @ eigvecs.T

        # real sqrtm
        L = np.real_if_close(linalg.sqrtm(A_pd))
        if not np.all(np.isfinite(L)):
            raise ValueError("sqrtm produced non-finite entries")

        samples_centered = samples - bias
        v = (L @ samples_centered.T).T
        norms = np.linalg.norm(v, axis=1)
        mean_norm = np.mean(norms)

        # avoid zero division
        target_radius = np.median(norms[norms > 0]) if np.any(norms > 0) else 1.0
        scale = target_radius / mean_norm if mean_norm > 1e-6 else 1.0

        T = L * scale
        if not np.all(np.isfinite(T)):
            raise ValueError("T matrix not finite")

        print(f"  ✅ Coverage = {quality:.1f}% good — full ellipsoid calibration applied.")
        return bias, T, quality

    except Exception as e:
        print(f"  ⚠ Coverage = {quality:.1f}% ❌ fit failed ({e}) — using offset-only calibration.")
        bias = np.mean(samples, axis=0)
        T = np.eye(3)
        return bias, T, quality



def parse_line(line: bytes):
    try:
        vals = [float(x) for x in line.decode(errors="ignore").strip().split(",")]
        return vals if len(vals) == EXPECTED_FIELDS else None
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port for live streaming")
    args = parser.parse_args()

    print("\n[STEP 1] Finding latest raw files...")
    accel_file = latest("accel_raw_data")
    gyro_file  = latest("gyro_raw_data")
    mag_file   = latest("mag_raw_data")
    print(f" ACC: {accel_file}\n GYR: {gyro_file}\n MAG: {mag_file}")

    print("\n[STEP 2] Loading samples...")
    accel = load_three_col(accel_file)
    gyro  = load_three_col(gyro_file)
    mag   = load_three_col(mag_file)

    print("\n[STEP 3] Calibrating (auto-normalized ellipsoid)...")
    acc_bias, acc_T, acc_q = ellipsoid_calibrate_autoscale(accel)
    gyro_bias = np.mean(gyro, axis=0)
    mag_bias, mag_T, mag_q = ellipsoid_calibrate_autoscale(mag)

    np.set_printoptions(precision=5, suppress=True)

    
   # --- Format numeric outputs to 5 decimals ---
    np.set_printoptions(precision=5, suppress=False, floatmode='fixed')

    print("\n[ACCEL] bias:", np.round(acc_bias, 5))
    print("[ACCEL] T:\n", np.round(acc_T, 5))

    print("\n[GYRO] bias:", np.round(gyro_bias, 5))

    print("\n[MAG] bias:", np.round(mag_bias, 5))
    print("[MAG] T:\n", np.round(mag_T, 5))

    print(f"[MAG] Coverage Quality: {mag_q:.1f}%")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_file = OUTPUT_DIR / f"imucalibrateddata_{ts}.txt"
    out = open(out_file, "w", newline="")
    writer = csv.writer(out)
    writer.writerow(["ax","ay","az","gx","gy","gz","mx","my","mz","pressure","temperature","altitude"])

    print("\n[STEP 4] Opening Serial...")
    ser = serial.Serial(args.port, BAUD, timeout=1)
    ser.reset_input_buffer()

    print("\n✅ LIVE CALIBRATED STREAM STARTED (Ctrl+C to stop)\n")

    try:
        count = 0
        while True:
            vals = parse_line(ser.readline())
            if vals is None:
                continue
            count += 1
            ax, ay, az, gx, gy, gz, mx, my, mz, p, t, a = vals
            acc_c = acc_T @ (np.array([ax, ay, az]) - acc_bias)
            gyro_c = np.array([gx, gy, gz]) - gyro_bias
            mag_c = mag_T @ (np.array([mx, my, mz]) - mag_bias)

            if count % 10 == 0:  # print every 10th sample
                print(f"{acc_c[0]:+.5f},{acc_c[1]:+.5f},{acc_c[2]:+.5f},"
                      f"{gyro_c[0]:+.5f},{gyro_c[1]:+.5f},{gyro_c[2]:+.5f},"
                      f"{mag_c[0]:+.5f},{mag_c[1]:+.5f},{mag_c[2]:+.5f},"
                      f"{p:.5f},{t:.5f},{a:.5f}")

            writer.writerow([
                f"{acc_c[0]:.5f}", f"{acc_c[1]:.5f}", f"{acc_c[2]:.5f}",
                f"{gyro_c[0]:.5f}", f"{gyro_c[1]:.5f}", f"{gyro_c[2]:.5f}",
                f"{mag_c[0]:.5f}", f"{mag_c[1]:.5f}", f"{mag_c[2]:.5f}",
                f"{p:.5f}", f"{t:.5f}", f"{a:.5f}"
            ])

    except KeyboardInterrupt:
        print("\n[STOP]")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            out.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
