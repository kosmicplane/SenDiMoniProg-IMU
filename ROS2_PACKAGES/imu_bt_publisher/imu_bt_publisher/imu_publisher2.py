#!/usr/bin/env python3
import serial
import time
import sys

PORT = "/dev/rfcomm0"
BAUDRATE = 230400  # en SPP a veces da igual, pero no molesta

def connect_serial():
    while True:
        try:
            ser = serial.Serial(PORT, BAUDRATE, timeout=0)  # no bloqueante
            print(f"✅ Connected to {PORT}", flush=True)
            return ser
        except Exception as e:
            print(f"⚠️  Retrying connection: {e}", flush=True)
            time.sleep(1)

def main():
    ser = connect_serial()
    try:
        while True:
            try:
                n = ser.in_waiting
                if n:
                    data = ser.read(n)
                    txt = data.decode("utf-8", errors="ignore")
                    sys.stdout.write(txt)     # sin esperar newline
                    sys.stdout.flush()
                else:
                    time.sleep(0.005)  # evita 100% CPU
            except (serial.SerialException, OSError) as e:
                print(f"\n⚠️  Serial error: {e} -> reconnecting...", flush=True)
                try: ser.close()
                except: pass
                ser = connect_serial()
    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try: ser.close()
        except: pass

if __name__ == "__main__":
    main()
