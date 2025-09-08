import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, TextBox
from collections import deque
from multiprocessing import Process, Queue, Event
from datetime import datetime
import os
import matplotlib
import time

# --- 数据收集进程 ---
def data_collector(port_name, baud_rate, data_queue, stop_event):
    """
    负责从串口读取数据并放入队列。
    """
    ser = None
    try:
        ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)
        print(f"串口 {port_name} 已成功打开。")
    except serial.SerialException as e:
        print(f"无法打开串口 {port_name}: {e}")
        stop_event.set()
        return

    while not stop_event.is_set():
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('A[g]:'):
                parts = line.split('|')
                parsed_data = {
                    'ax': float(parts[0].replace('A[g]:', '').split(',')[0]),
                    'ay': float(parts[0].replace('A[g]:', '').split(',')[1]),
                    'az': float(parts[0].replace('A[g]:', '').split(',')[2]),
                    'gx': float(parts[1].replace('G[rad/s]:', '').split(',')[0]),
                    'gy': float(parts[1].replace('G[rad/s]:', '').split(',')[1]),
                    'gz': float(parts[1].replace('G[rad/s]:', '').split(',')[2]),
                    'mx': float(parts[2].replace('M[mG]:', '').split(',')[0]),
                    'my': float(parts[2].replace('M[mG]:', '').split(',')[1]),
                    'mz': float(parts[2].replace('M[mG]:', '').split(',')[2]),
                    'alt': float(parts[5].replace('Alt[m]:', '').strip()),
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                }
                data_queue.put(parsed_data)
        except (serial.SerialException, ValueError) as e:
            print(f"数据读取或解析错误: {e}")
            break
        
    if ser and ser.is_open:
        ser.close()
    print("数据收集进程已停止。")

# --- 数据保存进程 ---
def data_saver(data_queue, stop_event, save_event, path_queue, filename_queue):
    """
    负责将数据保存到文件。这部分在独立的进程中运行。
    """
    data_file = None
    
    while not stop_event.is_set():
        if save_event.is_set():
            try:
                if not path_queue.empty() and not filename_queue.empty():
                    save_path = path_queue.get()
                    fname = filename_queue.get()
                    full_path = os.path.join(save_path, fname)
                    
                    if data_file is not None:
                        data_file.close()
                
                    data_file = open(full_path, 'w', encoding='utf-8')
                    data_file.write('t,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Alt\n')
                    print(f'数据保存器：已开始保存到 {full_path}')
            except Exception as e:
                if "queue" not in str(e):
                    print(f'数据保存器：获取路径/文件名或打开文件失败: {e}')
                    save_event.clear()
        else:
            if data_file:
                data_file.close()
                data_file = None
                print('数据保存器：文件已关闭。')

        if data_file and not data_queue.empty():
            try:
                data = data_queue.get_nowait()
                data_str = f"{data['timestamp']},{data['ax']},{data['ay']},{data['az']},{data['gx']},{data['gy']},{data['gz']},{data['mx']},{data['my']},{data['mz']},{data['alt']}\n"
                data_file.write(data_str)
            except Exception as e:
                print(f"数据保存器：写入错误: {e}")
                data_file.close()
                data_file = None
        
        time.sleep(0.01)

    if data_file:
        data_file.close()
    print("数据保存进程已停止。")


# --- 数据保存 GUI 窗口 (主进程) ---
def data_saver_gui(stop_event, save_event, path_queue, filename_queue):
    """
    负责创建数据保存窗口并处理按钮交互。
    """
    fig = plt.figure(figsize=(5, 3))
    fig.canvas.manager.set_window_title('Data Saver')
    plt.subplots_adjust(bottom=0.4, top=0.9, left=0.1, right=0.9)

    # 路径输入框
    ax_path_input = plt.axes([0.15, 0.5, 0.7, 0.2])
    tb_path_input = TextBox(ax_path_input, 'Path', initial='/home/e454/Desktop/SenDiMoniProg-IMU/IMU/IMUdata')
    
    # 文件名输入框
    ax_fname = plt.axes([0.15, 0.75, 0.7, 0.2])
    tb_fname = TextBox(ax_fname, 'Filename', initial='imu_data.txt')

    # 控制按钮
    ax_btn_start = plt.axes([0.15, 0.05, 0.3, 0.15])
    ax_btn_stop = plt.axes([0.55, 0.05, 0.3, 0.15])
    btn_start = Button(ax_btn_start, 'Start Save')
    btn_stop = Button(ax_btn_stop, 'End Save')
    
    # 计时器文本
    ax_timer = plt.axes([0.45, 0.25, 0.2, 0.1])
    ax_timer.axis('off')
    timer_text = ax_timer.text(0.5, 0.5, '0.00s', ha='center', va='center', fontsize=12)

    # 在窗口打开时立即开始计时
    collect_start_time = time.time()  
    
    # 在这里创建变量绑定，修复nonlocal错误
    timer_animation = None

    def update_timer(frame):
        nonlocal collect_start_time, timer_animation
        if collect_start_time:
            elapsed_time = time.time() - collect_start_time
            timer_text.set_text(f'{elapsed_time:.2f}s')
        else:
            timer_text.set_text('0.00s')
        return timer_text,

    timer_animation = animation.FuncAnimation(fig, update_timer, interval=100, blit=False, cache_frame_data=False)


    def start_save(event):
        nonlocal collect_start_time
        path = tb_path_input.text.strip()
        fname = tb_fname.text.strip()
        
        if not os.path.isdir(path):
            print(f'Error: Path "{path}" does not exist.')
            return
            
        path_queue.put(path)
        filename_queue.put(fname)
        save_event.set()
        
        collect_start_time = time.time()
        
        print('Sent start signal to data saver.')

    def stop_save(event):
        nonlocal collect_start_time, timer_animation
        save_event.clear()
        
        if timer_animation:
            timer_animation.event_source.stop()
            
        collect_start_time = None
        timer_text.set_text('0.00s')
        plt.draw()
        print('Sent end save signal to data saver.')
    
    btn_start.on_clicked(start_save)
    btn_stop.on_clicked(stop_save)

    plt.show()

    stop_event.set()
    save_event.clear()
    print("Data Saver GUI has been closed.")

# --- 主程序入口 ---
if __name__ == '__main__':
    data_queue = Queue()
    stop_event = Event()
    save_event = Event()
    path_queue = Queue()
    filename_queue = Queue()

    collector_process = Process(target=data_collector, args=('/dev/ttyUSB0', 230400, data_queue, stop_event))
    collector_process.daemon = True
    collector_process.start()

    saver_process = Process(target=data_saver, args=(data_queue, stop_event, save_event, path_queue, filename_queue))
    saver_process.daemon = True
    saver_process.start()

    data_saver_gui(stop_event, save_event, path_queue, filename_queue)

    collector_process.join()
    saver_process.join()
    print("Program has fully exited.")
