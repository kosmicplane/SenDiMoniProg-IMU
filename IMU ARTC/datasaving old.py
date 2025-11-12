# imu_collector.py
# Combined: three-stage calibration collection + continuous recording mode (with relative altitude "ral")
import os
import sys
import csv
import time
import threading
from pathlib import Path
import serial
import numpy as np
from datetime import datetime

# ===== Shared parameters =====
PORT = "COM3"                 # Windows: "COM3" / macOS: "/dev/tty.usbserial-xxxx"
BAUD = 230400
TIMEOUT = 1.0
DEFAULT_N_SAMPLES = 1090              # ← Default values ​​(can be changed before each collection)

SAVE_DIR = Path("C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/samples")   # output dir for three-stage calibration (.txt)
ROOT_DIR = "C:/Users/aadis/OneDrive/Documents/PlatformIO/Projects/IMU py/data/samples"               # output dir for continuous recording (.txt)

# Fixed field indices (0-based): ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temperature,altitude
AX, AY, AZ = 0, 1, 2
GX, GY, GZ = 3, 4, 5
MX, MY, MZ = 6, 7, 8
ALT_IDX = 11
EXPECTED_FIELDS = 12

# ===== Continuous recording mode parameters =====
HZ = 100
CYCLE_SAMPLES = 60 * HZ  # 6000（time 欄位 0.01..60.00 循環）

# Control object for streaming recording
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

# ---------------- Shared utilities ----------------
def open_serial():
    print(f"[SER] Opening {PORT} @ {BAUD} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT, rtscts=False, dsrdtr=False, xonxoff=False)
    except serial.SerialException as e:
        print(f"[ERR] Could not open serial port: {e}")
        sys.exit(1)
    # Avoid DTR/RTS triggering reset/download mode
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
    """Enter an integer; blank fields use the default; invalid fields will be asked again"""
    while True:
        s = input(f"{prompt} (default {default}): ").strip()
        if s == "":
            return default
        try:
            v = int(s)
            if v > 0:
                return v
            print("Please enter a positive integer.")
        except ValueError:
            print("Invalid format, please enter a number.")

# ---------------- (A) Three-stage calibration collection ----------------
def ask_filename_txt():
    name = input("Enter a filename (without extension): ").strip()
    if not name:
        print("No filename entered — cancelling this collection.")
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
            print(f"Collected {count}/{n_samples} samples")
    return header, rows

def save_txt(path: Path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)  # Still separated by commas, but with .txt extensions
        if header:
            w.writerow(header)
        w.writerows(rows)
    print(f"Done! Wrote {len(rows)} rows -> {path}")

# ---------------- Coverage Quality Utility ----------------
def coverage_quality(samples):
    """Compute 3D data coverage percentage (0–100%)"""
    if len(samples) < 10:
        return 0.0
    samples = np.array(samples, dtype=float)
    cov = np.cov(samples.T)
    rank = np.linalg.matrix_rank(cov)
    eigvals = np.linalg.eigvalsh(cov)
    if np.max(eigvals) <= 0:
        return 0.0
    spread_ratio = np.min(eigvals) / np.max(eigvals)
    spread_ratio = np.clip(spread_ratio, 0, 1)
    score = (rank / 3.0) * (spread_ratio ** 0.5) * 100
    return score


def mode_three_stage(ser):
    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    print("\n[Stage 1: Gyroscope gx, gy, gz]")
    k = wait_key("Press s to start collection; q to quit this mode: ")
    if k == "q":
        print("Exiting three-stage mode.")
        return "done"
    if k == "s":
        #<< You can customize the number of transactions for this round here >>
        n_samples = ask_int("Please enter the number of samples N to be collected in this round", DEFAULT_N_SAMPLES)
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"gyro_raw_data_{timestamp}.txt"
        header, rows = collect_n(ser, [GX, GY, GZ], ["gx","gy","gz"], n_samples)
        save_txt(SAVE_DIR / fname, header, rows)    

        print("Gyroscope collection complete.\n")

    print("[Stage 2: Accelerometer ax, ay, az] - 6 face collection mode")

    # Ask samples per face (default ~25)
    samples_per_face = ask_int("Enter samples per face", 25)

    # Only ask filename ONCE
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"accel_raw_data_{timestamp}.txt"
    all_rows = []  # store data from all 6 faces


    for face in range(1, 7):
        print(f"\nPlace IMU on FACE {face}. Keep still.")
        k = wait_key("Press s to start sampling this face; q to abort accelerometer stage: ")
        if k == "q":
            print("Accelerometer stage aborted early.")
            break

        # Collect N samples for this face
        header, rows = collect_n(ser, [AX, AY, AZ], ["ax","ay","az"], samples_per_face)
        all_rows.extend(rows)
        print(f"✅ Face {face} collected ({samples_per_face} samples).")

        # Ask to rotate for next face
        if face < 6:
            print("➡ Rotate IMU to next face, then press r to continue.")
            while True:
                k2 = input().strip().lower()
                if k2 == "r":
                    break

    # Save one file after all faces collected
    save_txt(SAVE_DIR / fname, ["ax","ay","az"], all_rows)
    print("✅ All 6 accelerometer faces collected.\n")

    # ---------------- Magnetometer Stage (with Coverage %) ----------------
    print("[Stage 3: Magnetometer mx, my, mz] (repeatable)")
    while True:
        k = wait_key("Press s to start collection; e to end magnetometer stage: ")
        if k == "e":
            print("Ending magnetometer stage."); break
        n_samples = ask_int("Enter number of samples for this run N", DEFAULT_N_SAMPLES)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"mag_raw_data_{timestamp}.txt"
        header, rows = collect_n(ser, [MX, MY, MZ], ["mx","my","mz"], n_samples)
        save_txt(SAVE_DIR / fname, header, rows)

        # Calculate coverage
        quality = coverage_quality(rows)
        if quality >= 70:
            print(f"[MAG] Coverage: {quality:.1f}% ✅ Good — suitable for calibration.")
        elif quality >= 40:
            print(f"[MAG] Coverage: {quality:.1f}% ⚠ Fair — consider redoing with more rotation.")
        else:
            print(f"[MAG] Coverage: {quality:.1f}% ❌ Poor — redo recommended (rotate IMU in all axes).")

        print("This magnetometer run is complete.\n")


# ---------------- (B) Continuous recording mode (including ral) ----------------
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
    print("\n=== Continuous recording controls ===")
    print(f"Save directory: {ROOT_DIR}")
    print("1) First enter a filename (e.g. data.txt) -> Enter")
    print("2) s: start saving   3) e: stop saving   4) q: quit mode\n")

    while not SS.quit_flag:
        try:
            cmd = input("> ").strip()
        except EOFError:
            cmd = "q"

        if not cmd:
            continue
        c = cmd.lower()

        # The first non-empty input is treated as the filename (adds .txt if missing)
        if SS.filename is None:
            name = cmd
            if not os.path.splitext(name)[1]:
                name += ".txt"
            with SS.lock:
                SS.filename = name
            print(f"[INFO] Filename: {SS.filename}")
            continue

        if c == "s":
            if SS.saving:
                print("[WARN] Already saving")
                continue
            try:
                f = open_stream_file()
                with SS.lock:
                    SS.current_file = f
                    SS.saving = True
                    SS.sample_idx = 0
                    SS.first_alt = None
                print(f"[INFO] Started saving -> {SS.filepath}")
            except Exception as e:
                print(f"[ERR] Failed to open file: {e}")

        elif c == "e":
            if not SS.saving:
                print("[WARN] Not currently saving")
                continue
            with SS.lock:
                SS.saving = False
                if SS.current_file:
                    try:
                        SS.current_file.flush()
                        SS.current_file.close()
                        print(f"[INFO] Closed: {SS.filepath}")
                    except Exception as e:
                        print(f"[ERR] Failed to close file: {e}")
                SS.current_file = None

        elif c == "q":
            with SS.lock:
                SS.quit_flag = True
            print("[INFO] Quitting...")
            break

        else:
            # Change filename (only allowed when not saving)
            if SS.saving:
                print("[WARN] Currently saving — stop (e) before changing the filename")
            else:
                name = cmd
                if not os.path.splitext(name)[1]:
                    name += ".txt"
                with SS.lock:
                    SS.filename = name
                print(f"[INFO] Filename changed to: {SS.filename}")

def mode_stream(ser):
    # start input thread
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

            # parse altitude
            alt = safe_float(parts[ALT_IDX])
            if alt is None:
                continue

            with SS.lock:
                if SS.saving and SS.current_file is not None:
                    # first valid altitude -> treat as reference x1
                    if SS.first_alt is None:
                        SS.first_alt = alt

                    # fixed 100 Hz time field: 0.01..60.00 cycles
                    SS.sample_idx = (SS.sample_idx % CYCLE_SAMPLES) + 1
                    t_sec = SS.sample_idx / 100.0

                    # ral = alt(xn) - alt(x1)
                    ral = alt - SS.first_alt

                    # take ax..mz nine-axis string (keep original values)
                    nine_axes = ",".join(parts[0:9])

                    row = f"{t_sec:.2f},{nine_axes},{alt:.2f},{ral:.5f}"
                    SS.current_file.write(row + "\n")

        # finalization
        with SS.lock:
            if SS.current_file:
                try:
                    SS.current_file.flush()
                    SS.current_file.close()
                except Exception:
                    pass
            SS.current_file = None

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received, finalizing...")

# ---------------- Entry menu ----------------
def main():
    ser = open_serial()
    try:
        print("\n=== Mode selection ===")
        print("1) Three-stage calibration collection (gxgygz -> axayaz -> mxmymz). Each run can set its sample count. Outputs .txt")
        print("2) Continuous recording mode (time + 9-axis + altitude + ral). Outputs .txt")
        mode = input("Enter 1 or 2: ").strip()

        if mode == "1":
            result = mode_three_stage(ser)
            if result == "stream":
                # immediately start continuous recording mode
                mode_stream(ser)
        elif mode == "2":
            mode_stream(ser)
        else:
            print("No valid mode selected.")

    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[INFO] Exiting")


if __name__ == "__main__":
    main()
