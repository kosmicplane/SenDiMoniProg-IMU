#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IMU RAW logger (Scheme A)
- 連續讀取 IMU (約 100 Hz or device-defined)
- 用 list.append 暫存到 RAM
- 按 'q' 結束後一次輸出 .xlsx
"""

import time
from pathlib import Path
from datetime import datetime

# Windows: 按鍵偵測（不需 Enter）
import msvcrt

try:
    import serial
except ImportError:
    serial = None


# =========================
# USER SETTINGS
# =========================
PORT = "COM5"
BAUD = 230400
SER_TIMEOUT = 0.1          # 讓 readline 不會永遠卡住，才能檢查 q
EXPECTED_FIELDS = 12       # 依你輸出格式：ax..mz + 3個額外欄位（最後一個當 altitude）

BASE_ROOT = Path(r"C:\Users\Bryan\Desktop\IMU\SenDiMoniProg-IMU\SenDiMoniProg-IMU\IMU_assets\IMU_ARTC")
# 最終存檔：BASE_ROOT / folder_name / raw_data / <filename>.xlsx


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

    # 常見雜訊/標頭排除
    low = s.lower()
    if any(k in low for k in ["error", "nan", "inf"]):
        # 你也可以選擇不排除 nan/inf，但通常是壞資料
        return None

    # 有些裝置會包 [] 或 {}
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
        raise RuntimeError("pyserial not installed. Run: pip install pyserial")

    ser = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
    # 開啟後先讓裝置穩一下（有些板子會 reset）
    time.sleep(0.3)
    try:
        ser.reset_input_buffer()   # 清掉殘留資料
    except Exception:
        pass
    return ser


# =========================
# Main logging (Scheme A)
# =========================
def stream_to_xlsx_scheme_a(ser, root_dir: Path):
    suggested = datetime.now().strftime("imurawdata_%Y%m%d_%H%M%S")
    fname = input(f"Enter filename (without extension) [{suggested}]: ").strip() or suggested
    if not fname.lower().endswith(".xlsx"):
        fname += ".xlsx"

    out_path = root_dir / fname
    print(f"\n[INFO] Recording... (press 'q' to stop)")
    print(f"[INFO] Output path: {out_path}\n")

    # RAM 暫存
    rows = []
    prev_alt = None
    count = 0

    # 用來看即時速度（可選）
    t_start = time.time()
    t_last_report = t_start

    try:
        while True:
            # ✅ 按 q 結束（不需 Enter）
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch.lower() == "q":
                    print("\n[STOP] 'q' pressed. Saving file...")
                    break

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

            # timestamp 用電腦當下時間（跟你原本一致）
            ts = datetime.now().strftime("%y-%m-%d_%H.%M.%S.%f")[:-3]

            rows.append([ts, ax, ay, az, gx, gy, gz, mx, my, mz, alt, rel_alt])
            count += 1

            # 每 500 筆回報一次
            if count % 500 == 0:
                now = time.time()
                dt = now - t_last_report
                total_dt = now - t_start
                rate = 500.0 / dt if dt > 0 else 0.0
                avg_rate = count / total_dt if total_dt > 0 else 0.0
                print(f"[INFO] {count} samples | recent ~{rate:.1f} Hz | avg ~{avg_rate:.1f} Hz")
                t_last_report = now

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C. Saving file...")

    # 結束後一次寫 excel
    import pandas as pd
    df = pd.DataFrame(rows, columns=[
        "timestamp",
        "ax", "ay", "az",
        "gx", "gy", "gz",
        "mx", "my", "mz",
        "altitude",
        "relative_altitude"
    ])
    df.to_excel(out_path, index=False)
    print(f"[INFO] Saved: {out_path}")


def main():
    raw_dir = make_project_dirs()
    ser = open_serial()
    try:
        stream_to_xlsx_scheme_a(ser, raw_dir)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[INFO] Serial closed. Done.")


if __name__ == "__main__":
    main()
