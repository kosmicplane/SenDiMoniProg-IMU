#!/usr/bin/env python3
"""
cal2_mode2_only.py

Part 2 獨立版：
 - 只做連續 IMU 錄影 (~100 Hz)
 - 問你 folder_name，檔案存在該資料夾底下的 raw_data/ 裡
 - 檔名可以自己輸入，不輸入就用預設時間字串
"""

import time
from pathlib import Path
from datetime import datetime

# ---------- USER SETTINGS ----------
PORT = "//dev/ttyUSB0"          # 依你的實際序列埠修改
BAUD = 230400
TIMEOUT = 1.0

# 建立專案資料夾結構
folder_name = input("Enter folder name: ").strip()

BASE_ROOT = Path("/home/badkitten/Desktop/SenDiMoniProg-IMU/IMU_ARTC/data")
PROJECT_ROOT = BASE_ROOT / folder_name
ROOT_DIR = PROJECT_ROOT / "raw_data"

ROOT_DIR.mkdir(parents=True, exist_ok=True)

EXPECTED_FIELDS = 12  # ax,ay,az,gx,gy,gz,mx,my,mz,alt1,alt2,alt3(或相似結構)

# ---------- Serial ----------
def open_serial_port():
    try:
        import serial
    except Exception:
        print("[WARN] pyserial not available; serial modes won't run.")
        return None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(0.15)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        print(f"[SER] Opened {PORT} @ {BAUD}")
        return ser
    except Exception as e:
        print(f"[WARN] Could not open {PORT}: {e}")
        return None

def parse_line_to_parts(line_bytes):
    """把一行 bytes 轉成 list[str]，長度不夠就丟掉。"""
    if not line_bytes:
        return None
    try:
        s = line_bytes.decode("utf-8", errors="ignore").strip()
    except Exception:
        try:
            s = line_bytes.decode(errors="ignore").strip()
        except Exception:
            return None
    if not s:
        return None
    parts = [p.strip() for p in s.split(",")]
    return parts if len(parts) >= EXPECTED_FIELDS else None

# ---------- Part 2: 連續記錄 ----------
def mode_stream_interactive(ser):
    if ser is None:
        print("[ERR] Serial required for streaming mode.")
        return

    suggested = datetime.now().strftime("imurawdata_%Y%m%d_%H%M%S")
    fname = input(
        f"Enter filename (without extension) [{suggested}]: "
    ).strip() or suggested

    if not fname.endswith(".txt"):
        fname = fname + ".txt"

    path = ROOT_DIR / fname
    print(f"\n[INFO] Recording to: {path} (press Ctrl+C to stop)")

    sample_rate_hz = 100
    min_period = 1.0 / sample_rate_hz

    try:
        with open(path, "w", encoding="utf-8", buffering=1) as f:
            # 這一列是輸出的欄位名稱
            f.write(
                "timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,altitude,relative_altitude\n"
            )
            prev_alt = None
            count = 0

            while True:
                line = ser.readline()
                parts = parse_line_to_parts(line)
                if parts is None:
                    continue

                try:
                    # 依照你的 EXPECTED_FIELDS(12) 取前 12 個轉 float
                    ax, ay, az, gx, gy, gz, mx, my, mz, _, _, alt = map(
                        float, parts[:EXPECTED_FIELDS]
                    )
                except Exception:
                    # 有問題就跳過
                    continue

                # relative altitude = 本次 altitude - 上一次 altitude
                rel_alt = 0.0 if prev_alt is None else alt - prev_alt
                prev_alt = alt

                # 時間戳：ddmmyy_HH.MM.SS.mmm
                ts = datetime.now().strftime("%d%m%y_%H.%M.%S.%f")[:-3]

                f.write(
                    f"{ts},"
                    f"{ax:.5f},{ay:.5f},{az:.5f},"
                    f"{gx:.5f},{gy:.5f},{gz:.5f},"
                    f"{mx:.5f},{my:.5f},{mz:.5f},"
                    f"{alt:.5f},{rel_alt:.5f}\n"
                )

                count += 1
                if count % 500 == 0:
                    print(f"[INFO] {count} samples written...")

                # 簡單節流到 ~100 Hz
                time.sleep(min_period * 0.98)

    except KeyboardInterrupt:
        print("\n[STOP] Recording stopped by user.")
    finally:
        print(f"[INFO] File saved to: {path}")

# ---------- Main ----------
def main():
    ser = open_serial_port()
    if ser is None:
        print("[ERR] Serial not available; cannot start streaming.")
        return

    try:
        mode_stream_interactive(ser)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[INFO] Exiting.")

if __name__ == "__main__":
    main()
