#!/usr/bin/env python3
"""
Accelerometer calibration using ellipsoid fitting (correct formulas).

- Reads a plain text file with 3 columns (X,Y,Z). (default path is the one you gave)
- Fits an ellipsoid, obtains center (bias) and symmetric correction matrix.
- Produces:
  * bias vector (3)
  * 3x3 calibration matrix T such that: corrected = T @ (raw - bias)
  * C arrays you can paste into your ESP32 code
  * JSON file with calibration
  * simple sanity stats (mean |raw|, mean |calibrated|, std)
"""
import os
import json
import numpy as np
from scipy import linalg

# ==== USER CHANGEABLE ====
INPUT_FILE = "C:\\Users\\aadis\\OneDrive\\Documents\\PlatformIO\\Projects\\accel test nilduat\\accel_data.txt"
TARGET_MAG = 1.0    # target magnitude after calibration (1.0 for 'g' units, 9.80665 for m/s^2)
OUT_JSON = "accel_calibration.json"
# =========================

def load_txt(filename):
    # handles CSV with commas
    data = np.loadtxt(filename, delimiter=",")
    if data.ndim == 1:
        if data.size == 3:
            data = data.reshape(1,3)
        else:
            raise ValueError("File doesn't look like 3-column data")
    if data.shape[1] != 3:
        raise ValueError("Expecting 3 columns (X Y Z)")
    return data.astype(float)


def ellipsoid_fit(samples):
    """
    Fit ellipsoid to samples (shape N x 3).
    Returns M (3x3 symmetric), n (3x1), d (scalar) solving:
       x^T M x + n^T x + d = 0
    Implementation follows the Li & Griffiths / Teslabs style (same D matrix).
    """
    s = samples.T  # shape 3 x N
    D = np.array([ s[0]**2.0,
                   s[1]**2.0,
                   s[2]**2.0,
                   2.0*s[1]*s[2],
                   2.0*s[0]*s[2],
                   2.0*s[0]*s[1],
                   2.0*s[0],
                   2.0*s[1],
                   2.0*s[2],
                   np.ones_like(s[0]) ])
    S = D @ D.T
    S11 = S[:6,:6]; S12 = S[:6,6:]; S21 = S[6:, :6]; S22 = S[6:,6:]

    C = np.array([[-1,  1,  1,  0,  0,  0],
                  [ 1, -1,  1,  0,  0,  0],
                  [ 1,  1, -1,  0,  0,  0],
                  [ 0,  0,  0, -4,  0,  0],
                  [ 0,  0,  0,  0, -4,  0],
                  [ 0,  0,  0,  0,  0, -4] ])

    # Solve for v1
    E = np.linalg.inv(C) @ (S11 - S12 @ np.linalg.inv(S22) @ S21)
    ew, ev = np.linalg.eig(E)
    v1 = ev[:, np.argmax(ew)]
    if v1[0] < 0:
        v1 = -v1
    v2 = -np.linalg.inv(S22) @ S21 @ v1

    # compose quadratic form parameters (matching the D layout)
    M = np.array([[v1[0], v1[5], v1[4]],
                  [v1[5], v1[1], v1[3]],
                  [v1[4], v1[3], v1[2]]], dtype=float)
    n = np.array([[v2[0]],[v2[1]],[v2[2]]], dtype=float)   # linear term vector
    d = float(v2[3])
    return M, n, d

def calibrate(samples, target_radius=1.0):
    """Return bias (3,), matrix T (3x3) such that corrected = T @ (raw - bias).
       Uses the correct 0.5 factor for center and k = 0.25 n^T M^-1 n - d.
    """
    M, n, d = ellipsoid_fit(samples)
    M_inv = linalg.inv(M)

    # center (bias)
    center = (-0.5 * (M_inv @ n)).reshape(3)

    # k constant (positive)
    k = 0.25 * (n.T @ M_inv @ n).item() - d
    if k <= 0:
        raise RuntimeError(f"Computed k <= 0 (k={k}). Fit bad or data degenerate.")

    # A (ellipsoid -> 1): (x-center)^T * A * (x-center) = 1
    A = M / k

    # matrix that maps (x-center) to the unit sphere: L = sqrtm(A)
    L = linalg.sqrtm(A)
    L = np.real_if_close(L, tol=1e6)   # clean small imag noise
    L = np.asarray(L, dtype=float)

    # scale to target_radius (usually 1.0 for 'g')
    v = (L @ (samples - center).T).T
    mags = np.linalg.norm(v, axis=1)
    mean_mag = np.mean(mags)
    if mean_mag == 0:
        raise RuntimeError("Mean magnitude after initial transform is 0 (bad).")
    scale_factor = target_radius / mean_mag
    T = L * scale_factor   # final transform

    return center, T, {
        "M": M, "n": n.flatten().tolist(), "d": d,
        "k": float(k), "mean_mag_before": float(np.mean(np.linalg.norm(samples,axis=1))),
        "mean_mag_after": float(np.mean(np.linalg.norm((T @ (samples - center).T).T, axis=1))),
        "std_after": float(np.std(np.linalg.norm((T @ (samples - center).T).T,axis=1))),
        "scale_factor": float(scale_factor)
    }

def print_c_arrays(center, T):
    print("\n// --- C arrays for Arduino / ESP32 (paste into your sketch) ---")
    print("float accel_bias[3] = { %0.6ff, %0.6ff, %0.6ff };" % tuple(center))
    print("float accel_cal[3][3] = {")
    for i in range(3):
        row = ", ".join("%0.9ff" % v for v in T[i,:])
        print("  { %s }%s" % (row, "," if i<2 else ""))
    print("};")


def main():
    print("Loading", INPUT_FILE)
    data = load_txt(INPUT_FILE)
    print("Loaded", data.shape[0], "samples.")

    center, T, meta = calibrate(data, target_radius=TARGET_MAG)

    print("\n=== Calibration result ===")
    print("Bias (center):\n", center)
    print("\nCalibration matrix T (applied as: corrected = T @ (raw - bias)):\n", T)
    print("\nSanity:")
    print("  mean |raw|   = %.4f" % meta["mean_mag_before"])
    print("  mean |cal|   = %.4f" % meta["mean_mag_after"])
    print("  std  |cal|   = %.4f" % meta["std_after"])
    print("  scale_factor = %.6f" % meta["scale_factor"])
    if abs(meta["mean_mag_after"] - TARGET_MAG) < 0.5 and meta["std_after"] < 0.1:
        print("  -> calibration looks reasonable (magnitude near TARGET).")
    else:
        print("  -> WARNING: calibrate result may be poor. Check data coverage and TARGET_MAG.")

    print_c_arrays(center, T)

if __name__ == "__main__":
    main()
