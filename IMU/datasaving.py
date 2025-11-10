# imu_collector.py
# 統合：三階段校正收集 + 連續紀錄模式（含相對高度 ral）
import os
import sys
import csv
import time
import threading
from pathlib import Path
import serial

# ===== 共用參數 =====
PORT = "/dev/ttyUSB0"                 # Windows: "COM3" / macOS: "/dev/tty.usbserial-xxxx"
BAUD = 230400
TIMEOUT = 1.0
DEFAULT_N_SAMPLES = 1090              # ← 預設值（可於每次收集前改）

SAVE_DIR = Path("/home/badkitten/Desktop/SenDiMoniProg-IMU/IMU/Correction/")   # 三階段輸出目錄（.txt）
ROOT_DIR = "/home/badkitten/Desktop/SenDiMoniProg-IMU/IMU/data/"               # 連續紀錄輸出目錄（.txt）

# 固定欄位索引（0-based）：ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temperature,altitude
AX, AY, AZ = 0, 1, 2
GX, GY, GZ = 3, 4, 5
MX, MY, MZ = 6, 7, 8
ALT_IDX = 11
EXPECTED_FIELDS = 12

# ===== 連續紀錄模式參數 =====
HZ = 100
CYCLE_SAMPLES = 60 * HZ  # 6000（time 欄位 0.01..60.00 循環）

# 連續紀錄控制用
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

# ---------------- 共用工具 ----------------
def open_serial():
    print(f"[SER] 開啟 {PORT} @ {BAUD} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT, rtscts=False, dsrdtr=False, xonxoff=False)
    except serial.SerialException as e:
        print(f"[ERR] 無法開啟序列埠：{e}")
        sys.exit(1)
    # 避免 DTR/RTS 觸發重置/下載模式
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass
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
    if len(parts) < EXPECTED_FIELDS:
        return None
    return parts

def ask_int(prompt: str, default: int):
    """輸入整數；空白用預設；非法重問。"""
    while True:
        s = input(f"{prompt}（預設 {default}）：").strip()
        if s == "":
            return default
        try:
            v = int(s)
            if v > 0:
                return v
            print("請輸入正整數。")
        except ValueError:
            print("格式錯誤，請輸入數字。")

# ---------------- (A) 三階段校正收集 ----------------
def ask_filename_txt():
    name = input("請輸入檔名（不含副檔名）：").strip()
    if not name:
        print("未輸入檔名，取消此次收集。")
        return None
    if not name.lower().endswith(".txt"):
        name += ".txt"
    return name

def wait_key(prompt):
    while True:
        k = input(prompt).strip().lower()
        if k in ("s", "e", "q"):
            return k

def collect_n(ser, indices, header, n_samples):
    rows, count = [], 0
    ser.reset_input_buffer()
    while count < n_samples:
        parts = parse_line_to_parts(ser.readline())
        if parts is None:
            continue
        vals = []
        ok = True
        for idx in indices:
            v = safe_float(parts[idx])
            if v is None:
                ok = False
                break
            vals.append(v)
        if not ok:
            continue
        rows.append(tuple(vals))
        count += 1
        if count % 100 == 0:
            print(f"已收集 {count}/{n_samples} 筆")
    return header, rows

def save_txt(path: Path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)  # 仍以逗號分隔，只是副檔名為 .txt
        if header:
            w.writerow(header)
        w.writerows(rows)
    print(f"完成！寫入 {len(rows)} 筆 → {path}")

def mode_three_stage(ser):
    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    print("\n【第一階段：陀螺儀 gx, gy, gz】")
    k = wait_key("按 s 開始收集；q 離開模式：")
    if k == "q":
        print("離開三階段模式。")
        return "done"
    if k == "s":
        # << 這裡可自定本輪筆數 >>
        n_samples = ask_int("請輸入本輪收集筆數 N", DEFAULT_N_SAMPLES)
        fname = ask_filename_txt()
        if fname is None:
            return "done"
        header, rows = collect_n(ser, [GX, GY, GZ], ["gx","gy","gz"], n_samples)
        save_txt(SAVE_DIR / fname, header, rows)
        print("陀螺儀收集完成。\n")

    print("【第二階段：加速度 ax, ay, az】（可多輪）")
    while True:
        k = wait_key("按 s 開始收集；e 結束加速度階段：")
        if k == "e":
            print("結束加速度階段。"); break
        n_samples = ask_int("請輸入本輪收集筆數 N", DEFAULT_N_SAMPLES)
        fname = ask_filename_txt()
        if fname is None:
            continue
        header, rows = collect_n(ser, [AX, AY, AZ], ["ax","ay","az"], n_samples)
        save_txt(SAVE_DIR / fname, header, rows)
        print("本輪加速度收集完成。\n")

    print("【第三階段：磁力計 mx, my, mz】（可多輪）")
    while True:
        k = wait_key("按 s 開始收集；e 結束磁力計階段：")
        if k == "e":
            print("結束磁力計階段。"); break
        n_samples = ask_int("請輸入本輪收集筆數 N", DEFAULT_N_SAMPLES)
        fname = ask_filename_txt()
        if fname is None:
            continue
        header, rows = collect_n(ser, [MX, MY, MZ], ["mx","my","mz"], n_samples)
        save_txt(SAVE_DIR / fname, header, rows)
        print("本輪磁力計收集完成。\n")

    print("三階段校正模式完成。")
    yn = input("是否要接著進入【連續紀錄模式（含 ral）】？(y/n)：").strip().lower()
    return "stream" if yn == "y" else "done"

# ---------------- (B) 連續紀錄模式（含 ral） ----------------
def ensure_dir_root():
    os.makedirs(ROOT_DIR, exist_ok=True)

def open_stream_file():
    ensure_dir_root()
    SS.filepath = os.path.join(ROOT_DIR, SS.filename)
    f = open(SS.filepath, "a", buffering=1, encoding="utf-8")
    if f.tell() == 0:
        f.write("time,ax,ay,az,gx,gy,gz,mx,my,mz,altitude_m,ral_m\n")
    return f

def input_thread():
    print("\n=== 連續紀錄操作 ===")
    print(f"儲存資料夾：{ROOT_DIR}")
    print("1) 先輸入檔名（例如 data.txt）→ Enter")
    print("2) s: 開始存檔   3) e: 結束存檔   4) q: 離開模式\n")

    while not SS.quit_flag:
        try:
            cmd = input("> ").strip()
        except EOFError:
            cmd = "q"

        if not cmd:
            continue
        c = cmd.lower()

        # 第一個非空輸入視為檔名（自動補 .txt）
        if SS.filename is None:
            name = cmd
            if not os.path.splitext(name)[1]:
                name += ".txt"
            with SS.lock:
                SS.filename = name
            print(f"[INFO] 檔名：{SS.filename}")
            continue

        if c == "s":
            if SS.saving:
                print("[WARN] 已在存檔中")
                continue
            try:
                f = open_stream_file()
                with SS.lock:
                    SS.current_file = f
                    SS.saving = True
                    SS.sample_idx = 0
                    SS.first_alt = None
                print(f"[INFO] 開始存檔 → {SS.filepath}")
            except Exception as e:
                print(f"[ERR] 開檔失敗：{e}")

        elif c == "e":
            if not SS.saving:
                print("[WARN] 尚未在存檔")
                continue
            with SS.lock:
                SS.saving = False
                if SS.current_file:
                    try:
                        SS.current_file.flush()
                        SS.current_file.close()
                        print(f"[INFO] 已關閉：{SS.filepath}")
                    except Exception as e:
                        print(f"[ERR] 關檔失敗：{e}")
                SS.current_file = None

        elif c == "q":
            with SS.lock:
                SS.quit_flag = True
            print("[INFO] 退出中…")
            break

        else:
            # 改檔名（需先 e 停止）
            if SS.saving:
                print("[WARN] 正在存檔中，先按 e 停止再改檔名")
            else:
                name = cmd
                if not os.path.splitext(name)[1]:
                    name += ".txt"
                with SS.lock:
                    SS.filename = name
                print(f"[INFO] 檔名改為：{SS.filename}")

def mode_stream(ser):
    # 啟動輸入執行緒
    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    try:
        while not SS.quit_flag:
            try:
                line = ser.readline()
            except Exception:
                time.sleep(0.005)
                continue

            parts = parse_line_to_parts(line)
            if parts is None:
                continue

            # 解析 altitude
            alt = safe_float(parts[ALT_IDX])
            if alt is None:
                continue

            with SS.lock:
                if SS.saving and SS.current_file is not None:
                    # 第一次有效高度 → 當作 x1
                    if SS.first_alt is None:
                        SS.first_alt = alt

                    # 固定 100 Hz 的 time 欄位：0.01..60.00 循環
                    SS.sample_idx = (SS.sample_idx % CYCLE_SAMPLES) + 1
                    t_sec = SS.sample_idx / 100.0

                    # ral = alt(xn) - alt(x1)
                    ral = alt - SS.first_alt

                    # 取 ax..mz 九軸字串（不改動原值）
                    nine_axes = ",".join(parts[0:9])

                    row = f"{t_sec:.2f},{nine_axes},{alt:.2f},{ral:.5f}"
                    SS.current_file.write(row + "\n")

        # 收尾
        with SS.lock:
            if SS.current_file:
                try:
                    SS.current_file.flush()
                    SS.current_file.close()
                except Exception:
                    pass
            SS.current_file = None

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C 收到，收尾…")

# ---------------- 入口選單 ----------------
def main():
    ser = open_serial()
    try:
        print("\n=== 模式選擇 ===")
        print("1) 三階段校正收集（gxgygz→axayaz→mxmymz，各輪筆數可自訂，輸出 .txt）")
        print("2) 連續紀錄模式（time + 9 軸 + altitude + ral，輸出 .txt）")
        mode = input("請輸入 1 或 2：").strip()

        if mode == "1":
            result = mode_three_stage(ser)
            if result == "stream":
                # 直接接著跑連續紀錄模式
                mode_stream(ser)
        elif mode == "2":
            mode_stream(ser)
        else:
            print("未選擇有效模式。")

    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[INFO] 結束")

if __name__ == "__main__":
    main()
