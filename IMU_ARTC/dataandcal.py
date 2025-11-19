#!/usr/bin/env python3
"""
cal2_cleaned_optionB.py

Unified IMU Data Collection + Calibration Tool
Option B: Ellipsoid calibrator, magnetometer autoscale to mean raw magnitude.

Modes:
  1) Three-stage calibration data collection (gyro -> accel 6 faces combined -> mag)
       After collection, shows coverage, asks whether to calibrate (yes/no).
       If yes: run ellipsoid accelerometer, ellipsoid+autoscale magnetometer,
       compute gyro bias, save calibration file and show sphericity numbers.
  2) Continuous recording mode (enter filename interactively)
       Save dotted timestamp ddmmyy_HH.MM.SS.mmm and relative altitude.
"""

import os
import sys
import csv
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy import linalg

# ------------- USER SETTINGS -------------
PORT = "COM8"
BAUD = 230400
TIMEOUT = 1.0
DEFAULT_N_SAMPLES = 1090

PROJECT_ROOT = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py")
SAVE_DIR = PROJECT_ROOT / "samples2"
ROOT_DIR = PROJECT_ROOT / "data" / "samples"
CAL_DIR = PROJECT_ROOT / "calibration_matrices"
OUT_DIR = PROJECT_ROOT / "calibrated output"

SAVE_DIR.mkdir(parents=True, exist_ok=True)
ROOT_DIR.mkdir(parents=True, exist_ok=True)
CAL_DIR.mkdir(parents=True, exist_ok=True)
OUT_DIR.mkdir(parents=True, exist_ok=True)

EXPECTED_FIELDS = 12  # timestamp not included; raw lines from IMU have 12 fields before timestamp usage
# Column indices for collect_n() and other 3-col loaders:
AX, AY, AZ = 0, 1, 2
GX, GY, GZ = 3, 4, 5
MX, MY, MZ = 6, 7, 8

# -----------------------------------------

def open_serial_port():
    """Open serial if available; return None if cannot open."""
    try:
        import serial
    except Exception:
        print("[WARN] pyserial not installed or unavailable; serial modes will fail.")
        return None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(0.2)
        try:
            ser.reset_input_buffer(); ser.reset_output_buffer()
        except Exception:
            pass
        print(f"[SER] Opened {PORT} @ {BAUD}")
        return ser
    except Exception as e:
        print(f"[WARN] Could not open serial port {PORT}: {e}")
        return None

def safe_float(s):
    try:
        return float(s)
    except Exception:
        return None

def parse_line_to_parts(line_bytes: bytes):
    """Parse a raw serial line bytes into parts list; return None if invalid."""
    if not line_bytes:
        return None
    try:
        text = line_bytes.decode("utf-8", errors="ignore").strip()
    except Exception:
        try:
            text = line_bytes.decode(errors="ignore").strip()
        except Exception:
            return None
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    return parts if len(parts) >= EXPECTED_FIELDS else None

# ---------- Data collection helpers ----------
def collect_n(ser, indices, header_labels, n_samples):
    """
    Collect n_samples lines from serial port 'ser' and extract columns indices.
    Returns list of tuples.
    """
    rows = []
    if ser is None:
        print("[ERR] Serial port not available.")
        return header_labels, rows
    ser.reset_input_buffer()
    count = 0
    while count < n_samples:
        line = ser.readline()
        parts = parse_line_to_parts(line)
        if parts is None:
            continue
        vals = [safe_float(parts[i]) for i in indices]
        if any(v is None for v in vals):
            continue
        rows.append(tuple(vals))
        count += 1

        time.sleep(0.05)   # 20 Hz sampling for calibration mode

        if count % 100 == 0:
            print(f"  Collected {count}/{n_samples}")
    return header_labels, rows

def write_csv(path: Path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if header:
            w.writerow(header)
        w.writerows(rows)

# ---------- Coverage and sphericity ----------
def coverage_quality(samples):
    """
    Coverage quality 0..100 similar to earlier function.
    """
    arr = np.asarray(samples, dtype=float)
    if arr.shape[0] < 10:
        return 0.0
    cov = np.cov(arr.T)
    rank = np.linalg.matrix_rank(cov)
    eigvals = np.linalg.eigvalsh(cov)
    if np.max(eigvals) <= 0:
        return 0.0
    spread_ratio = np.min(eigvals) / np.max(eigvals)
    spread_ratio = float(np.clip(spread_ratio, 0.0, 1.0))
    score = (rank / 3.0) * (spread_ratio ** 0.5) * 100.0
    return float(score)

def sphericity_pct(data):
    """Return 0..100 sphericity metric: 100*(1 - std(norm)/mean(norm))."""
    arr = np.asarray(data, dtype=float)
    if arr.size == 0:
        return 0.0
    norms = np.linalg.norm(arr, axis=1)
    m = np.mean(norms)
    if m == 0:
        return 0.0
    std = np.std(norms)
    s = 100.0 * max(0.0, 1.0 - (std / m))
    return float(np.clip(s, 0.0, 100.0))

def sphere_center_from_calibrated(data):
    """
    Fit sphere center (cx,cy,cz) + radius to calibrated data cloud.
    Only least-squares sphere fitting (not ellipsoid).
    """
    data = np.asarray(data, dtype=float)
    X = data[:,0]
    Y = data[:,1]
    Z = data[:,2]

    A = np.column_stack([2*X, 2*Y, 2*Z, np.ones(len(X))])
    b = X*X + Y*Y + Z*Z

    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, cz, k = sol

    center = np.array([cx, cy, cz], dtype=float)
    radius = np.sqrt(k + cx*cx + cy*cy + cz*cz)

    return center, float(radius)


# ---------- Ellipsoid math (shared) ----------
def ellipsoid_fit(samples):
    """Fit ellipsoid parameters M, n, d for equation x^T M x + n^T x + d = 0."""
    s = samples.T  # 3 x N
    D = np.array([s[0]**2.0, s[1]**2.0, s[2]**2.0,
                  2.0*s[1]*s[2], 2.0*s[0]*s[2], 2.0*s[0]*s[1],
                  2.0*s[0], 2.0*s[1], 2.0*s[2], np.ones_like(s[0])])
    S = D @ D.T
    S11 = S[:6, :6]; S12 = S[:6, 6:]; S21 = S[6:, :6]; S22 = S[6:, 6:]
    C = np.array([[-1, 1, 1, 0, 0, 0],
                  [1, -1, 1, 0, 0, 0],
                  [1, 1, -1, 0, 0, 0],
                  [0, 0, 0, -4, 0, 0],
                  [0, 0, 0, 0, -4, 0],
                  [0, 0, 0, 0, 0, -4]])
    E = np.linalg.inv(C) @ (S11 - S12 @ np.linalg.inv(S22) @ S21)
    ew, ev = np.linalg.eig(E)
    v1 = ev[:, np.argmax(ew)]
    if v1[0] < 0:
        v1 = -v1
    v2 = -np.linalg.inv(S22) @ S21 @ v1
    M = np.array([[v1[0], v1[5], v1[4]],
                  [v1[5], v1[1], v1[3]],
                  [v1[4], v1[3], v1[2]]], dtype=float)
    n = np.array([[v2[0]],[v2[1]],[v2[2]]], dtype=float)
    d = float(v2[3])
    return M, n, d

# ---------- Calibration algorithms ----------
def calibrate_accelerometer_ellipsoid(samples, target_radius=1.0):
    """
    Pure ellipsoid-based accel calibration:
    Returns bias (center), T (3x3), coverage %, success flag.
    Uses the correct center formula with Minv and k.
    """
    quality = coverage_quality(samples)
    if quality < 70:
        # fallback to offset-only
        bias = np.mean(samples, axis=0)
        T = np.eye(3)
        return bias, T, quality, False
    try:
        M, n, d = ellipsoid_fit(samples)
        Minv = linalg.inv(M)
        center = (-0.5 * Minv @ n).reshape(3)
        k = 0.25 * (n.T @ Minv @ n).item() - d
        if not np.isfinite(k) or k <= 1e-12:
            bias = np.mean(samples, axis=0)
            T = np.eye(3)
            return bias, T, quality, False
        A = M / k
        eigvals, eigvecs = np.linalg.eigh(A)
        eigvals = np.clip(eigvals, 1e-9, None)
        A_pd = eigvecs @ np.diag(eigvals) @ eigvecs.T
        L = np.real_if_close(linalg.sqrtm(A_pd))
        L = np.asarray(L, dtype=float)
        v = (L @ (samples - center).T).T
        norms = np.linalg.norm(v, axis=1)
        mean_norm = np.mean(norms)
        if mean_norm <= 1e-9:
            bias = np.mean(samples, axis=0)
            T = np.eye(3)
            return bias, T, quality, False
        scale = target_radius / mean_norm
        T = L * scale
        return center, T, quality, True
    except Exception as e:
        bias = np.mean(samples, axis=0)
        T = np.eye(3)
        return bias, T, coverage_quality(samples), False

def calibrate_magnetometer_ellipsoid_autoscale(samples):
    """
    Ellipsoid-based mag calibration with autoscale (Option B).
    Compute hard-iron bias and soft-iron matrix A1, then scale A1 so that
    mean calibrated magnitude == mean raw magnitude.
    Returns bias, A1, coverage, ok flag.
    """
    quality = coverage_quality(samples)
    if quality < 40:
        bias = np.mean(samples, axis=0)
        A1 = np.eye(3)
        return bias, A1, quality, False
    try:
        M, n, d = ellipsoid_fit(samples)
        Minv = linalg.inv(M)
        b = (-Minv @ n).reshape(3)
        denom = (n.T @ Minv @ n).item() - d
        if denom <= 0 or not np.isfinite(denom):
            bias = b
            A1 = np.eye(3)
            return bias, A1, quality, False
        k = float(np.sqrt(1.0 / denom))
        A1 = np.real(k * linalg.sqrtm(M))
        A1 = np.asarray(A1, dtype=float)
        # autoscale
        raw_norms = np.linalg.norm(samples, axis=1)
        mean_raw = np.mean(raw_norms)
        test = ((samples - b) @ A1.T)
        mean_cal = np.mean(np.linalg.norm(test, axis=1))
        if mean_cal <= 1e-9 or not np.isfinite(mean_cal):
            bias = b
            A1 = np.eye(3)
            return bias, A1, quality, False
        scale_factor = mean_raw / mean_cal
        A1 *= scale_factor
        return b, A1, quality, True
    except Exception:
        bias = np.mean(samples, axis=0)
        A1 = np.eye(3)
        return bias, A1, coverage_quality(samples), False

# ---------- File helpers ----------
def latest_raw_file(prefix="imurawdata_"):
    files = sorted(ROOT_DIR.glob(prefix + "*.txt"))
    return files[-1] if files else None

def latest_cal_file(prefix="calibration_matrices_"):
    files = sorted(CAL_DIR.glob(prefix + "*.txt"))
    return files[-1] if files else None

def robust_load_three_col(fname):
    """Load a file with 3 columns; skip header if alphabetic tokens present."""
    text = Path(fname).read_text()
    lines = [ln for ln in text.splitlines() if ln.strip() != ""]
    if not lines:
        raise ValueError("Empty file.")
    start = 0
    # detect header if contains non-numeric token
    first_tokens = lines[0].split(",")
    if any(any(c.isalpha() for c in t) for t in first_tokens):
        start = 1
    arr = np.genfromtxt(lines[start:], delimiter=",")
    arr = np.atleast_2d(arr)
    if arr.shape[1] != 3:
        raise ValueError(f"Expected 3 columns in {fname}, got {arr.shape}")
    return arr.astype(float)

# ---------- Mode 1: three-stage then calibrate ----------
def mode_three_stage_and_calibrate(ser):
    """Collect gyro, accelerometer (6 faces -> single file), magnetometer, then ask to calibrate."""
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    print("\n==== 3-Stage IMU Data Collection ====\n")

    # --- GYRO ---
    print("[Stage 1: Gyroscope gx, gy, gz]")
    cmd = input("Press s to start (q to quit): ").strip().lower()
    if cmd != "s":
        print("[INFO] Skipping gyro collection.")
        return
    n_gyro = DEFAULT_N_SAMPLES
    try:
        n_gyro = int(input(f"Enter number of gyro samples (default {DEFAULT_N_SAMPLES}): ") or DEFAULT_N_SAMPLES)
    except Exception:
        n_gyro = DEFAULT_N_SAMPLES
    header, gyro_rows = collect_n(ser, [GX, GY, GZ], ["gx", "gy", "gz"], n_gyro)
    gyro_path = SAVE_DIR / f"gyro_raw_data_{datetime.now():%Y%m%d_%H%M%S}.txt"
    write_csv(gyro_path, header, gyro_rows)
    print(f"[SAVE] Wrote {len(gyro_rows)} rows to {gyro_path}")

    # --- ACCEL - 6 faces combined into one file ---
    print("\n[Stage 2: Accelerometer ax, ay, az] - 6 faces")
    try:
        samples_per_face = int(input("Enter samples per face (default 25): ") or 25)
    except Exception:
        samples_per_face = 25

    all_accel_rows = []
    for face in range(1, 7):
        input(f"Place IMU on face {face} then press Enter...")
        header, rows = collect_n(ser, [AX, AY, AZ], ["ax", "ay", "az"], samples_per_face)
        all_accel_rows.extend(rows)
    accel_path = SAVE_DIR / f"accel_raw_data_{datetime.now():%Y%m%d_%H%M%S}.txt"
    write_csv(accel_path, ["ax","ay","az"], all_accel_rows)
    print(f"[SAVE] Wrote {len(all_accel_rows)} rows to {accel_path}")

    # --- MAG ---
    print("\n[Stage 3: Magnetometer mx, my, mz] — rotate IMU freely")
    cmd = input("Press s to start magnetometer collection (q to quit): ").strip().lower()
    if cmd != "s":
        print("[INFO] Skipping magnetometer collection.")
        return
    try:
        n_mag = int(input(f"Enter number of mag samples (default {DEFAULT_N_SAMPLES}): ") or DEFAULT_N_SAMPLES)
    except Exception:
        n_mag = DEFAULT_N_SAMPLES
    header, mag_rows = collect_n(ser, [MX, MY, MZ], ["mx","my","mz"], n_mag)
    mag_path = SAVE_DIR / f"mag_raw_data_{datetime.now():%Y%m%d_%H%M%S}.txt"
    write_csv(mag_path, header, mag_rows)
    print(f"[SAVE] Wrote {len(mag_rows)} rows to {mag_path}")

    # Show coverage percentages
    try:
        accel_np = np.asarray(all_accel_rows, dtype=float)
        mag_np = np.asarray(mag_rows, dtype=float)
    except Exception:
        print("[ERR] Could not convert collected data to numeric arrays.")
        return

    acc_cov = coverage_quality(accel_np)
    mag_cov = coverage_quality(mag_np)
    print(f"\n[INFO] Coverage: ACCEL = {acc_cov:.1f}%, MAG = {mag_cov:.1f}%")

    # Ask whether to calibrate or recollect
    while True:
        choice = input("Do calibration now? (y = calibrate / r = recollect / q = quit): ").strip().lower()
        if choice == "r":
            print("[INFO] Recollection requested. Restarting mode 1.")
            return mode_three_stage_and_calibrate(ser)
        if choice == "q":
            print("[INFO] Exiting without calibration.")
            return
        if choice == "y":
            break
        print("Please enter y, r, or q.")

    # Perform calibrations (ellipsoid-only for accel, ellipsoid+autoscale for mag)
    print("\n[CAL] Starting ellipsoid calibration (Option B: mag autoscale)...")
    acc_bias, acc_T, acc_cov2, acc_ok = calibrate_accelerometer_ellipsoid(accel_np, target_radius=1.0)
    gyro_bias = np.mean(np.asarray(gyro_rows, dtype=float), axis=0) if len(gyro_rows)>0 else np.zeros(3)
    mag_bias, mag_T, mag_cov2, mag_ok = calibrate_magnetometer_ellipsoid_autoscale(mag_np)

    # Save calibration file
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    cal_path = CAL_DIR / f"calibration_matrices_{ts}.txt"
    with cal_path.open("w", encoding="utf-8") as f:
        f.write("=== IMU Calibration Matrices ===\n")
        f.write("[ACCEL] bias: [{}]\n".format(", ".join(f"{x:.5f}" for x in acc_bias)))
        f.write("[ACCEL] T:\n")
        for row in acc_T:
            f.write("[" + ", ".join(f"{x:.5f}" for x in row) + "]\n")
        f.write("[GYRO] bias: [{}]\n".format(", ".join(f"{x:.5f}" for x in gyro_bias)))
        f.write("[MAG] bias: [{}]\n".format(", ".join(f"{x:.5f}" for x in mag_bias)))
        f.write("[MAG] T:\n")
        for row in mag_T:
            f.write("[" + ", ".join(f"{x:.5f}" for x in row) + "]\n")
        f.write(f"[ACCEL] Coverage: {acc_cov:.1f}% ({'ellipsoid_ok' if acc_ok else 'offset-only'})\n")
        f.write(f"[MAG] Coverage: {mag_cov:.1f}% ({'ellipsoid_ok' if mag_ok else 'offset-only'})\n")
    print(f"\n✅ Calibration saved to: {cal_path}")

    # compute sphericity metrics raw vs calibrated for both sensors and print
    accel_cal = (acc_T @ (accel_np - acc_bias.reshape(1,3)).T).T
    mag_cal = (mag_T @ (mag_np - mag_bias.reshape(1,3)).T).T

    acc_raw_sph = sphericity_pct(accel_np)
    acc_cal_sph = sphericity_pct(accel_cal)
    mag_raw_sph = sphericity_pct(mag_np)
    mag_cal_sph = sphericity_pct(mag_cal)

    print("\nSphericity (0..100) — raw vs calibrated:")
    print(f"  ACCEL  RAW: {acc_raw_sph:.2f}%   CAL: {acc_cal_sph:.2f}%")
    print(f"  MAG    RAW: {mag_raw_sph:.2f}%   CAL: {mag_cal_sph:.2f}%")

    # --------------------------------------------------------
    #  POST-CALIBRATION SPHERE CENTER VALIDATION (requested)
    # --------------------------------------------------------
    acc_center, acc_radius = sphere_center_from_calibrated(accel_cal)
    mag_center, mag_radius = sphere_center_from_calibrated(mag_cal)

    def mag(v): return float(np.linalg.norm(v))

    print("\n=== POST-CALIBRATION SPHERE CENTER VALIDATION ===")

    print("\nACCELEROMETER (CALIBRATED SPHERE CENTER):")
    print(f"   Center X = {acc_center[0]:.6f}")
    print(f"   Center Y = {acc_center[1]:.6f}")
    print(f"   Center Z = {acc_center[2]:.6f}")
    print(f" → Distance of calibrated center from true origin = {mag(acc_center):.6f}")
    print(f" → Calibrated radius = {acc_radius:.6f}")

    print("\nMAGNETOMETER (CALIBRATED SPHERE CENTER):")
    print(f"   Center X = {mag_center[0]:.6f}")
    print(f"   Center Y = {mag_center[1]:.6f}")
    print(f"   Center Z = {mag_center[2]:.6f}")
    print(f" → Distance of calibrated center from true origin = {mag(mag_center):.6f}")
    print(f" → Calibrated radius = {mag_radius:.6f}\n")


    return

# ---------- Mode 2: continuous recording with filename input ----------
def mode_stream_interactive(ser):
    """Continuous mode where user supplies filename (or default)"""
    ROOT_DIR.mkdir(parents=True, exist_ok=True)
    suggested = datetime.now().strftime("imurawdata_%Y%m%d_%H%M%S")
    fname = input(f"Enter filename (without extension) [{suggested}]: ").strip() or suggested
    if not fname.endswith(".txt"):
        fname = fname + ".txt"
    path = ROOT_DIR / fname
    print(f"\n[INFO] Recording to: {path}")
    print("[INFO] Press Ctrl+C to stop.\n")
    try:
        with open(path, "w", encoding="utf-8", buffering=1) as f:
            f.write("timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,altitude,relative_altitude\n")
            prev_alt = None
            count = 0
            while True:
                line = ser.readline()
                if not line:
                    continue
                parts = parse_line_to_parts(line)
                if not parts:
                    continue
                try:
                    ax, ay, az, gx, gy, gz, mx, my, mz, _, _, alt = map(float, parts[:EXPECTED_FIELDS])
                except Exception:
                    continue
                rel_alt = 0.0 if prev_alt is None else alt - prev_alt
                prev_alt = alt
                ts = datetime.now().strftime("%d%m%y_%H.%M.%S.%f")[:-3]
                f.write(f"{ts},{ax:.5f},{ay:.5f},{az:.5f},{gx:.5f},{gy:.5f},{gz:.5f},{mx:.5f},{my:.5f},{mz:.5f},{alt:.5f},{rel_alt:.5f}\n")
                count += 1
                
                time.sleep(0.01)   # 100 Hz logging

                if count % 500 == 0:
                    elapsed = time.time()
                    print(f"[INFO] {count} samples written...")
    except KeyboardInterrupt:
        print("\n[STOP] Recording stopped by user.")
    except Exception as e:
        print(f"[ERR] {e}")
    finally:
        print(f"[INFO] File saved to: {path}")

# ---------- Main entry ----------
def main():
    ser = open_serial_port()
    try:
        print("\n=== Mode Selection ===")
        print("1) Three-stage calibration data collection (auto-run ellipsoid calibration after collection)")
        print("2) Continuous recording mode (enter filename interactively)")
        mode = input("Enter 1 or 2: ").strip()
        if mode == "1":
            if ser is None:
                print("[ERR] Serial not available; mode 1 requires serial port.")
                return
            mode_three_stage_and_calibrate(ser)
        elif mode == "2":
            if ser is None:
                print("[ERR] Serial not available; mode 2 requires serial port.")
                return
            mode_stream_interactive(ser)
        else:
            print("Invalid choice.")
    finally:
        try:
            if ser:
                ser.close()
        except Exception:
            pass
        print("[INFO] Exiting.")

if __name__ == "__main__":
    main()
