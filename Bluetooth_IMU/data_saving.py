import bluetooth
import time
import os
import re
import select
from multiprocessing import Process, Queue, Event
from datetime import datetime

# --- Bluetooth Connection Setup ---
bd_addr = "70:B8:F6:67:0E:E6"
port = 1
# data_buffer is now managed within the data_collector function

def data_collector(bd_addr, port, data_queue, stop_event):
    """
    负责通过蓝牙连接到指定设备，读取数据并放入队列。
    这个函数在独立的进程中运行。
    """
    sock = None
    try:
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((bd_addr, port))
        sock.setblocking(0)
        print("数据收集进程: 蓝牙连接成功!")
    except bluetooth.btcommon.BluetoothError as err:
        print(f"数据收集进程: 蓝牙连接失败: {err}")
        stop_event.set()
        return

    data_buffer = ""
    while not stop_event.is_set():
        try:
            ready_to_read, _, _ = select.select([sock], [], [], 0)
            if ready_to_read:
                data = sock.recv(4096) # 增加缓冲区大小
                data_buffer += data.decode('utf-8', errors='ignore')

            while '\n' in data_buffer:
                line, data_buffer = data_buffer.split('\n', 1)
                # 使用正则表达式进行更健壮的解析
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
                    except (ValueError, IndexError) as e:
                        # 忽略格式错误的数据行
                        print(f"数据收集进程: 忽略不完整的或格式错误的数据行: {line} -> {e}")
                else:
                    # 忽略不匹配的数据行
                    print(f"数据收集进程: 忽略不匹配的数据行: {line}")


        except (bluetooth.btcommon.BluetoothError, ValueError) as e:
            print(f"数据收集进程: 读取或解析错误: {e}")
            stop_event.set()
            break
        
    if sock:
        sock.close()
    print("数据收集进程已停止。")

# --- 数据保存进程 (Data Saver Process) ---
def data_saver(data_queue, stop_event, save_event, filename):
    """
    负责将数据保存到文件。这部分在独立的进程中运行。
    """
    data_file = None
    
    # 增加一个缓冲区来批量写入数据
    buffer_size = 100  # 缓冲区大小，可根据需要调整
    data_buffer = []

    while not stop_event.is_set():
        if save_event.is_set():
            if data_file is None:
                try:
                    full_path = os.path.join("/home/e454/Desktop/SenDiMoniProg-IMU/Bluetooth_IMU/IMU_Data", filename)
                    data_file = open(full_path, 'w', encoding='utf-8')
                    data_file.write('t,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Alt\n')
                    print(f'数据保存器：已开始保存到 {full_path}')
                except Exception as e:
                    print(f'数据保存器：文件打开失败: {e}')
                    save_event.clear()
        else:
            if data_file:
                # 停止时清空缓冲区并关闭文件
                for data in data_buffer:
                    data_file.write(data)
                data_buffer = []
                data_file.close()
                data_file = None
                print('数据保存器：文件已关闭。')

        if data_file and not data_queue.empty():
            try:
                data = data_queue.get_nowait()
                data_str = f"{data['timestamp']},{data['ax']},{data['ay']},{data['az']},{data['gx']},{data['gy']},{data['gz']},{data['mx']},{data['my']},{data['mz']},{data['alt']}\n"
                
                data_buffer.append(data_str)
                # 缓冲区满时批量写入
                if len(data_buffer) >= buffer_size:
                    for line in data_buffer:
                        data_file.write(line)
                    data_buffer = []

            except Exception as e:
                print(f"数据保存器：写入错误: {e}")
                if data_file:
                    data_file.close()
                    data_file = None
        
        # 移除 time.sleep，以提高数据处理速率
        # time.sleep(0.01)

    if data_file:
        data_file.close()
    print("数据保存进程已停止。")

# --- 主程序入口 (Main Program Entry) ---
if __name__ == '__main__':
    data_queue = Queue()
    stop_event = Event()
    save_event = Event()

    collector_process = Process(target=data_collector, args=(bd_addr, port, data_queue, stop_event))
    collector_process.daemon = True
    collector_process.start()

    # 在终端中接收文件名
    filename = input("请输入数据文件名 (例如: imu_data.txt): ") or "imu_data.txt"
    
    saver_process = Process(target=data_saver, args=(data_queue, stop_event, save_event, filename))
    saver_process.daemon = True
    saver_process.start()

    print("程序已启动。")
    print("按 's' 开始保存数据，按 'e' 结束保存，按 'q' 退出。")

    try:
        while True:
            user_input = input().strip().lower()
            if user_input == 's':
                save_event.set()
                print("开始保存数据。")
            elif user_input == 'e':
                save_event.clear()
                print("停止保存数据。")
            elif user_input == 'q':
                stop_event.set()
                break
    except KeyboardInterrupt:
        stop_event.set()
        
    collector_process.join()
    saver_process.join()
    print("程序已完全退出。")
