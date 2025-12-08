import serial
from datetime import datetime
import os

# === PUERTO SERIAL EN LA JETSON ===
# Verifícalo con:  ls /dev/ttyACM*  o  ls /dev/ttyUSB*
PORT = "/dev/ttyACM0"      # <-- CAMBIO IMPORTANTE: agregar /dev/
BAUD = 115200

# === Cada ejecución genera un archivo nuevo ===
OUT_FILE = datetime.now().strftime("can_log_%Y%m%d_%H%M%S.csv")
print("Log file will be saved as:")
print(os.path.abspath(OUT_FILE))


def parse_can_line(line: str):
    """
    Espera líneas con formato:
    ID: 0x2A6  DLC: 8  Data: 00 00 01 F4 00 00 00 00
    Devuelve (id_hex, dlc, [data bytes]) o None si no coincide.
    """
    line = line.strip()
    if not line:
        return None

    if not line.startswith("ID:"):
        return None

    parts = line.split()
    # Esperado:
    # 0: "ID:"
    # 1: "0x2A6"
    # 2: "DLC:"
    # 3: "8"
    # 4: "Data:"
    # 5~: data bytes
    if len(parts) < 5:
        return None

    try:
        can_id = parts[1]          # "0x2A6"
        dlc = parts[3]             # "8"
        data_bytes = parts[5:]     # ["00","00","01","F4",...]
        return can_id, dlc, data_bytes
    except Exception:
        return None


def main():
    print(f"Opening serial port {PORT} @ {BAUD} ...")
    ser = serial.Serial(PORT, BAUD, timeout=1)

    with open(OUT_FILE, "w", encoding="utf-8") as f:
        # Cabecera CSV
        f.write("timestamp,can_id,dlc,data0,data1,data2,data3,data4,data5,data6,data7\n")
        print(f"Logging to {OUT_FILE} (Ctrl+C para detener)\n")

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                try:
                    line = raw.decode(errors="ignore").strip()
                except UnicodeDecodeError:
                    continue

                if not line:
                    continue

                # >>>> IMPRIMIR SIEMPRE LO QUE LLEGA (para verlo por SSH)
                print("RX:", line)

                # Intentar parsear como línea CAN
                parsed = parse_can_line(line)
                if parsed is None:
                    # No coincide con el formato CAN esperado -> solo se muestra en pantalla
                    continue

                can_id, dlc, data_bytes = parsed

                # Rellenar hasta 8 bytes
                while len(data_bytes) < 8:
                    data_bytes.append("")

                # Timestamp con milisegundos
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                # Construir la fila CSV
                row = ",".join([ts, can_id, dlc] + data_bytes[:8])
                f.write(row + "\n")
                f.flush()

                # También mostrar la fila que se guardó
                print("LOG:", row)

        except KeyboardInterrupt:
            print("\n停止記錄 / Deteniendo log.")

    ser.close()


if __name__ == "__main__":
    main()
