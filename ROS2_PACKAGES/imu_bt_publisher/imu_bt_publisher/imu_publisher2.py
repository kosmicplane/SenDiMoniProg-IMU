#!/usr/bin/env python3
import serial
import time

PORT = "/dev/rfcomm0"
BAUDRATE = 230400

def connect_serial():
    while True:
        try:
            ser = serial.Serial(PORT, BAUDRATE, timeout=1.0)
            print(f"✅ Connected to {PORT} @ {BAUDRATE}", flush=True)
            return ser
        except Exception as e:
            print(f"⚠️  Retrying connection: {e}", flush=True)
            time.sleep(2)

def main():
    ser = connect_serial()
    try:
        while True:
            try:
                # Lee una línea (hasta '\n'); timeout=1.0 evita bloquear infinito
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    print(line, flush=True)

            except (serial.SerialException, OSError) as e:
                print(f"⚠️  Serial error: {e} -> reconnecting...", flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(1)
                ser = connect_serial()

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
