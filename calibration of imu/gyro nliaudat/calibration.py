#!/usr/bin/env python3
"""
Gyroscope Calibration Tool (Ellipsoid Fitting)

Reads gyro data from gyro_data1.txt (CSV: gx,gy,gz in rad/s), fits an ellipsoid to estimate bias and scale,
and prints Python code for applying the calibration:
    gyro_calibrated = scale_matrix @ (gyro_raw - bias)
"""

import numpy as np
import os
from scipy import linalg

GYRO_FILE = 'gyro_data1.txt'

# --- Ellipsoid fitting (same math as magnetometer, but for gyro) ---
def ellipsoid_fit(s):
    D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                  2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                  2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])
    S = np.dot(D, D.T)
    S_11 = S[:6,:6]
    S_12 = S[:6,6:]
    S_21 = S[6:,:6]
    S_22 = S[6:,6:]
    C = np.array([[-1,  1,  1,  0,  0,  0],
                  [ 1, -1,  1,  0,  0,  0],
                  [ 1,  1, -1,  0,  0,  0],
                  [ 0,  0,  0, -4,  0,  0],
                  [ 0,  0,  0,  0, -4,  0],
                  [ 0,  0,  0,  0,  0, -4]])
    E = np.dot(linalg.inv(C), S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))
    E_w, E_v = np.linalg.eig(E)
    v_1 = E_v[:, np.argmax(E_w)]
    if v_1[0] < 0: v_1 = -v_1
    v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)
    M = np.array([[v_1[0], v_1[5], v_1[4]],
                  [v_1[5], v_1[1], v_1[3]],
                  [v_1[4], v_1[3], v_1[2]]])
    n = np.array([[v_2[0]], [v_2[1]], [v_2[2]]])
    d = v_2[3]
    return M, n, d

def print_c_arrays(center, T):
    print("\n// --- C arrays for Arduino / ESP32 (paste into your sketch) ---")
    print("float gyro_bias[3] = { %0.6ff, %0.6ff, %0.6ff };" % tuple(center))
    print("float gyro_cal[3][3] = {")
    for i in range(3):
        row = ", ".join("%0.9ff" % v for v in T[i,:])
        print("  { %s }%s" % (row, "," if i<2 else ""))
    print("};")

# --- Main calibration routine ---
def main():
    if not os.path.exists(GYRO_FILE):
        print(f"File '{GYRO_FILE}' not found.")
        return
    print(f"Loading gyro data from {GYRO_FILE}...")
    data = np.loadtxt(GYRO_FILE, delimiter=',')
    if data.shape[1] != 3:
        print(f"Expected 3 columns, got {data.shape[1]}")
        return
    print(f"Loaded {len(data)} samples.")
    print("First 5 samples:")
    print(data[:5])

    # Fit ellipsoid
    s = data.T
    M, n, d = ellipsoid_fit(s)
    M_1 = linalg.inv(M)
    bias = (-M_1 @ n).flatten()
    # Scale matrix: normalize so that the mean norm is 1 (unit sphere)
    norm_factor = np.sqrt(np.dot(n.T, M_1 @ n) - d)
    scale_matrix = np.real(linalg.sqrtm(M)) / norm_factor

    print("\nGyro bias (rad/s):")
    print(bias)
    print("\nGyro scale matrix:")
    print(scale_matrix)

    # Output C arrays in your desired format
    print_c_arrays(bias, scale_matrix)


if __name__ == "__main__":
    main()