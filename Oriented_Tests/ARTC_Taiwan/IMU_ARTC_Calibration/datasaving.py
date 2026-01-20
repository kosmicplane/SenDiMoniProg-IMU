#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IMU RAW logger (Scheme A - Jetson/Linux)
- 連續讀取 IMU（裝置吐多少就讀多少，不 sleep）
- 用 list.append 暫存到 RAM
- 按 Ctrl+C 結束後才存檔（先存 CSV；若有 pandas/openpyxl 再存 XLSX）
"""

import time
import csv
from pathlib import Path
from datetime import datetime

try:
    import serial
except ImportError:
    serial = None


# =========================
# USER SETTINGS
# =========================
PORT = "/dev/ttyUSB0"       # ✅ 修正：Linux 通常是 /dev/ttyUSB0 或 /dev/ttyACM0
BAUD = 230400
SER_TIMEOUT = 0.1
EXPECTED_FIELDS = 12        # ax..mz + 3個額外欄位（最後一個當 altitude）

BASE_ROOT = Path("/home/badkitten/Desktop/SenDiMoniProg-IMU/IMU_assets/IMU_ARTC/data/")
# 最終存檔：BASE_ROOT / folder_name / raw_data / <filename>.(csv/xlsx)


# =========================
# Helpers
# =========================
def parse_line_to_parts(line_bytes):
    """把一行 serial bytes 解析成 list[str]（逗號分隔），解析失敗回 None。"""
    try:
        s = line_bytes.decode("utf-8", errors="ignore").strip()
    except Exception:
        return None

    if not s:
        return None

    low = s.lower()
    if any(k in low for k in ["error", "nan", "inf"]):
        return None

    s = s.strip().strip("[]{}()")
    parts = [p.strip() for p in s.split(",")]
    if len(parts) < EXPECTED_FIELDS:
        return None

    return parts


def make_project_dirs():
    folder_name = input("Enter folder name (project folder): ").strip()
    if not folder_name:
        raise ValueError("Folder name cannot be empty.")

    project_root = BASE_ROOT / folder_name
    raw_dir = project_root / "raw_data"
    raw_dir.mkdir(parents=True, exist_ok=True)
    return raw_dir


def open_serial():
    if serial is None:
        raise RuntimeError("pyserial not installed. Run: pip3 install pyserial")

    ser = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
    time.sleep(0.3)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    return ser


# =========================
# Main logging (Scheme A)
# =========================
def stream_scheme_a_ctrlc(ser, root_dir: Path):
    suggested = datetime.now().strftime("imurawdata_%Y%m%d_%H%M%S")
    fname = input(f"Enter filename (without extension) [{suggested}]: ").strip() or suggested

    csv_path = root_dir / f"{fname}.csv"
    xlsx_path = root_dir / f"{fname}.xlsx"

    print(f"\n[INFO] Recording... (press Ctrl+C to stop)")
    print(f"[INFO] CSV path : {csv_path}")
    print(f"[INFO] XLSX path: {xlsx_path} (optional)\n")

    rows = []
    prev_alt = None
    count = 0

    t_start = time.time()
    t_last_report = t_start

    try:
        while True:
            line = ser.readline()
            if not line:
                continue

            parts = parse_line_to_parts(line)
            if parts is None:
                continue

            # 你目前格式：ax,ay,az,gx,gy,gz,mx,my,mz,?, ?, alt(最後一個)
            try:
                ax, ay, az, gx, gy, gz, mx, my, mz, _, _, alt = map(float, parts[:EXPECTED_FIELDS])
            except Exception:
                continue

            rel_alt = 0.0 if prev_alt is None else (alt - prev_alt)
            prev_alt = alt

            ts = datetime.now().strftime("%y-%m-%d_%H.%M.%S.%f")[:-3]
            rows.append([ts, ax, ay, az, gx, gy, gz, mx, my, mz, alt, rel_alt])
            count += 1

            if count % 500 == 0:
                now = time.time()
                dt = now - t_last_report
                total_dt = now - t_start
                rate = 500.0 / dt if dt > 0 else 0.0
                avg_rate = count / total_dt if total_dt > 0 else 0.0
                print(f"[INFO] {count} samples | recent ~{rate:.1f} Hz | avg ~{avg_rate:.1f} Hz")
                t_last_report = now

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C received. Saving...")


    # ====== 先存 CSV（最穩、最少依賴）======
    header = [
        "timestamp",
        "ax", "ay", "az",
        "gx", "gy", "gz",
        "mx", "my", "mz",
        "altitude",
        "relative_altitude"
    ]

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    print(f"[INFO] Saved CSV : {csv_path}")

    # ====== 可選：存 XLSX（需要 pandas + openpyxl）======
    try:
        import pandas as pd
        df = pd.DataFrame(rows, columns=header)
        df.to_excel(xlsx_path, index=False)
        print(f"[INFO] Saved XLSX: {xlsx_path}")
    except Exception as e:
        print(f"[INFO] Skip XLSX (need pandas/openpyxl). Reason: {e}")


def main():
    raw_dir = make_project_dirs()
    ser = open_serial()
    try:
        stream_scheme_a_ctrlc(ser, raw_dir)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[INFO] Serial closed. Done.")


if __name__ == "__main__":
    main()
