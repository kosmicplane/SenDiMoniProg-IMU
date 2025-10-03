import bluetooth
import os
import re
import select
from multiprocessing import Process, Queue, Event
from datetime import datetime

# --- Bluetooth Connection Setup ---
bd_addr = "70:B8:F6:67:0E:E6"
port = 1

def data_collector(bd_addr, port, data_queue, stop_event):
    sock = None
    try:
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((bd_addr, port))
        sock.setblocking(0)
    except bluetooth.btcommon.BluetoothError:
        stop_event.set()
        return

    data_buffer = ""
    while not stop_event.is_set():
        try:
            ready_to_read, _, _ = select.select([sock], [], [], 0)
            if ready_to_read:
                data = sock.recv(4096)
                data_buffer += data.decode('utf-8', errors='ignore')

            while '\n' in data_buffer:
                line, data_buffer = data_buffer.split('\n', 1)
                match = re.search(
                    r'A\[g\]: (.*?), (.*?), (.*?) \| G\[rad/s\]: (.*?), (.*?), (.*?) \| M\[mG\]: (.*?), (.*?), (.*?) \| P\[mbar\]: .* \| T\[C\]: .* \| Alt\[m\]: (.*)',
                    line
                )
                if match:
                    try:
                        parsed_data = {
                            'ax': float(match.group(1)), 'ay': float(match.group(2)), 'az': float(match.group(3)),
                            'gx': float(match.group(4)), 'gy': float(match.group(5)), 'gz': float(match.group(6)),
                            'mx': float(match.group(7)), 'my': float(match.group(8)), 'mz': float(match.group(9)),
                            'alt': float(match.group(10)),
                            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        }
                        data_queue.put(parsed_data)
                    except (ValueError, IndexError):
                        pass
        except (bluetooth.btcommon.BluetoothError, ValueError):
            stop_event.set()
            break
        
    if sock:
        sock.close()

def data_saver(data_queue, stop_event, save_event, filename):
    data_file = None
    buffer_size = 100
    data_buffer = []

    # 自動抓主目錄並建立存檔資料夾
    save_dir = os.path.expanduser("~/Desktop/SenDiMoniProg-IMU/Bluetooth_IMU/IMU_Data")
    os.makedirs(save_dir, exist_ok=True)
    full_path = os.path.join(save_dir, filename)

    while not stop_event.is_set():
        if save_event.is_set():
            if data_file is None:
                try:
                    data_file = open(full_path, 'w', encoding='utf-8')
                    data_file.write('t,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Alt\n')
                except Exception:
                    save_event.clear()
        else:
            if data_file:
                for data in data_buffer:
                    data_file.write(data)
                data_buffer = []
                data_file.close()
                data_file = None

        if data_file and not data_queue.empty():
            try:
                data = data_queue.get_nowait()
                data_str = f"{data['timestamp']},{data['ax']},{data['ay']},{data['az']},{data['gx']},{data['gy']},{data['gz']},{data['mx']},{data['my']},{data['mz']},{data['alt']}\n"
                data_buffer.append(data_str)
                if len(data_buffer) >= buffer_size:
                    for line in data_buffer:
                        data_file.write(line)
                    data_buffer = []
            except Exception:
                if data_file:
                    data_file.close()
                    data_file = None

    if data_file:
        data_file.close()

# --- Main ---
if __name__ == '__main__':
    data_queue = Queue()
    stop_event = Event()
    save_event = Event()

    collector_process = Process(target=data_collector, args=(bd_addr, port, data_queue, stop_event))
    collector_process.daemon = True
    collector_process.start()

    filename = input("Enter file name (e.g., imu_data.txt): ") or "imu_data.txt"
    
    saver_process = Process(target=data_saver, args=(data_queue, stop_event, save_event, filename))
    saver_process.daemon = True
    saver_process.start()

    try:
        while True:
            user_input = input().strip().lower()
            if user_input == 's':
                save_event.set()
            elif user_input == 'e':
                save_event.clear()
            elif user_input == 'q':
                stop_event.set()
                break
    except KeyboardInterrupt:
        stop_event.set()
        
    collector_process.join()
    saver_process.join()
