# save_imu_root_with_input_filename_fixed_time.py
import os
import threading
import serial
import time
from datetime import datetime

# === 串口設定（依需要修改） ===
PORT = "/dev/ttyUSB0"   # Windows: "COM3" / macOS: "/dev/tty.usbserial-xxxx"
BAUD = 230400
TIMEOUT_SEC = 1.0
# ===========================

ROOT_DIR = "/home/badkitten/Desktop/SenDiMoniProg-IMU/IMU/data/"

saving = False
quit_flag = False
current_file = None
filename = None
filepath = None
lock = threading.Lock()

# 100 Hz 序列時間計數（每筆 +0.01，1..6000 → 0.01..60.00）
sample_idx = 0
HZ = 100
CYCLE_SAMPLES = 60 * HZ  # 6000

# 相對高度用：記錄第一筆高度 alt(x1)
first_alt = None

def ensure_dir():
    os.makedirs(ROOT_DIR, exist_ok=True)

def open_file():
    """依目前 filename 開啟檔案，必要時寫入標頭。"""
    global current_file, filepath
    ensure_dir()
    filepath = os.path.join(ROOT_DIR, filename)
    f = open(filepath, "a", buffering=1, encoding="utf-8")
    if f.tell() == 0:
        f.write("time,ax,ay,az,gx,gy,gz,mx,my,mz,altitude_m,ral_m\n")
    return f

def input_thread():
    global saving, quit_flag, current_file, filename, filepath, sample_idx, first_alt

    print("\n=== 操作 ===")
    print(f"儲存資料夾：{ROOT_DIR}")
    print("1) 先輸入檔名（例如 data.csv）→ Enter")
    print("2) s: 開始存檔   3) e: 結束存檔   4) q: 離開\n")

    while not quit_flag:
        try:
            cmd = input("> ").strip()
        except EOFError:
            cmd = "q"

        if not cmd:
            continue

        c = cmd.lower()

        # 若尚未設定檔名，第一個輸入視為檔名（自動補 .csv）
        if filename is None:
            name = cmd
            if not os.path.splitext(name)[1]:
                name += ".csv"
            with lock:
                filename = name
            print(f"[INFO] 檔名：{filename}")
            continue

        if c == "s":
            if saving:
                print("[WARN] 已在存檔中")
                continue
            try:
                f = open_file()
                with lock:
                    current_file = f
                    saving = True
                    sample_idx = 0      # time 從 0.01
                    first_alt = None    # 重新開始時尚未有 x1
                print(f"[INFO] 開始存檔 → {filepath}")
            except Exception as e:
                print(f"[ERR] 開啟檔案失敗：{e}")

        elif c == "e":
            if not saving:
                print("[WARN] 尚未在存檔")
                continue
            with lock:
                saving = False
                if current_file:
                    try:
                        current_file.flush()
                        current_file.close()
                        print(f"[INFO] 已關閉：{filepath}")
                    except Exception as e:
                        print(f"[ERR] 關檔失敗：{e}")
                current_file = None

        elif c == "q":
            with lock:
                quit_flag = True
            print("[INFO] 退出中…")
            break

        else:
            if saving:
                print("[WARN] 正在存檔中，先按 e 停止再改檔名")
            else:
                name = cmd
                if not os.path.splitext(name)[1]:
                    name += ".csv"
                with lock:
                    filename = name
                print(f"[INFO] 檔名改為：{filename}")

def main():
    global saving, quit_flag, current_file, sample_idx, first_alt

    # 啟動輸入執行緒
    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    # 連線序列埠
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_SEC)
        print(f"[INFO] 連線：{PORT} @ {BAUD}")
    except Exception as e:
        print(f"[ERR] 無法開啟序列埠 {PORT}：{e}")
        return

    try:
        while not quit_flag:
            try:
                line = ser.readline()
            except Exception:
                time.sleep(0.01)
                continue

            if not line:
                continue

            # 不顯示到螢幕 —— 僅寫檔
            try:
                text = line.decode("utf-8", errors="ignore").strip()
            except Exception:
                text = line.decode(errors="ignore").strip()

            if not text:
                continue

            # 期望 ESP32 輸出順序：
            # ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temperature,altitude
            parts = [p.strip() for p in text.split(",")]
            if len(parts) < 12:
                continue

            # 解析需要的欄位：ax..mz (0..8) 與 altitude (index 11)
            axayaz_gxgygz_mxmymz = parts[0:9]
            alt_str = parts[11]

            # 轉成浮點以便計算 ral；轉換失敗就跳過該筆
            try:
                alt = float(alt_str)
            except ValueError:
                continue

            with lock:
                if saving and current_file is not None:
                    # 第一次有效高度 → 設為 x1
                    if first_alt is None:
                        first_alt = alt

                    # 固定 100Hz 的 time 欄位：0.01..60.00 循環
                    sample_idx = (sample_idx % CYCLE_SAMPLES) + 1    # 1..6000
                    t_sec = sample_idx / 100.0                       # 0.01..60.00

                    # ral = alt(xn) - alt(x1)
                    ral = alt - first_alt

                    # 寫入：time + 9軸 + altitude + ral
                    row = (
                        f"{t_sec:.2f}," +
                        ",".join(axayaz_gxgygz_mxmymz) + "," +
                        f"{alt:.2f}," +
                        f"{ral:.5f}"
                    )
                    current_file.write(row + "\n")

        # 收尾
        with lock:
            if current_file:
                try:
                    current_file.flush()
                    current_file.close()
                except Exception:
                    pass
            current_file = None

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C 收到，收尾…")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[INFO] 結束")

if __name__ == "__main__":
    main()
