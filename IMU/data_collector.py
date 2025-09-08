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

# --- Matplotlib 配置 ---
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False 

# --- 数据收集进程 ---
def data_collector(port_name, baud_rate, data_queue, stop_event):
    ser = None
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=1)
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
    data_file = None
    
    while not stop_event.is_set():
        if save_event.is_set():
            try:
                # 检查队列中是否有新的路径和文件名，并取出
                if not path_queue.empty() and not filename_queue.empty():
                    save_path = path_queue.get()
                    fname = filename_queue.get()
                    full_path = os.path.join(save_path, fname)
                    
                    if data_file: # 如果文件已打开，先关闭
                        data_file.close()
                    
                    data_file = open(full_path, 'w', encoding='utf-8')
                    data_file.write('t,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Alt\n')
                    print(f'数据保存器：已开始保存到 {full_path}')
            except Exception as e:
                print(f'数据保存器：获取路径/文件名失败: {e}')
                save_event.clear()
        
        if not save_event.is_set() and data_file is not None:
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


# --- 绘图 GUI 进程 (主进程) ---
def gui_plotter(data_queue, stop_event, save_event, path_queue, filename_queue):
    window = 100
    ax_data = deque([0] * window, maxlen=window)
    ay_data = deque([0] * window, maxlen=window)
    az_data = deque([0] * window, maxlen=window)
    gx_data = deque([0] * window, maxlen=window)
    gy_data = deque([0] * window, maxlen=window)
    gz_data = deque([0] * window, maxlen=window)
    mx_data = deque([0] * window, maxlen=window)
    my_data = deque([0] * window, maxlen=window)
    mz_data = deque([0] * window, maxlen=window)
    alt_data = deque([0] * window, maxlen=window)

    fig, axs = plt.subplots(4, 1, figsize=(8, 10))
    fig.canvas.manager.set_window_title('IMU 实时监控')
    plt.subplots_adjust(bottom=0.32, hspace=0.5)

    l_ax, = axs[0].plot(ax_data, label='Ax')
    l_ay, = axs[0].plot(ay_data, label='Ay')
    l_az, = axs[0].plot(az_data, label='Az')
    axs[0].legend(loc='upper left')
    axs[0].set_title('Accelerometer (g)')
    axs[0].set_ylim(-5, 5)

    l_gx, = axs[1].plot(gx_data, label='Gx')
    l_gy, = axs[1].plot(gy_data, label='Gy')
    l_gz, = axs[1].plot(gz_data, label='Gz')
    axs[1].legend(loc='upper left')
    axs[1].set_title('Gyroscope (rad/s)')
    axs[1].set_ylim(-10, 10)

    l_mx, = axs[2].plot(mx_data, label='Mx')
    l_my, = axs[2].plot(my_data, label='My')
    l_mz, = axs[2].plot(mz_data, label='Mz')
    axs[2].legend(loc='upper left')
    axs[2].set_title('Magnetometer (mG)')
    axs[2].set_ylim(-1000, 1000)

    l_alt, = axs[3].plot(alt_data, label='Altitude')
    axs[3].legend(loc='upper left')
    axs[3].set_title('Altitude (m)')
    axs[3].set_ylim(100, 200)

    ax_path_input = plt.axes([0.15, 0.22, 0.6, 0.05])
    tb_path_input = TextBox(ax_path_input, '路径', initial=os.getcwd())
    
    ax_fname = plt.axes([0.15, 0.15, 0.6, 0.05])
    tb_fname = TextBox(ax_fname, '文件名', initial='imu_data.txt')

    ax_btn_start = plt.axes([0.15, 0.01, 0.18, 0.05])
    ax_btn_stop = plt.axes([0.37, 0.01, 0.18, 0.05])
    ax_btn_quit = plt.axes([0.59, 0.01, 0.18, 0.05])
    btn_start = Button(ax_btn_start, '开始保存')
    btn_stop = Button(ax_btn_stop, '结束保存')
    btn_quit = Button(ax_btn_quit, '退出')

    def start_collect(event):
        path = tb_path_input.text.strip()
        fname = tb_fname.text.strip()
        
        if not os.path.isdir(path):
            print(f'错误：路径 "{path}" 不存在。')
            return
            
        # 将路径和文件名发送给数据保存进程
        path_queue.put(path)
        filename_queue.put(fname)
        save_event.set()
        print('已发送开始信号到数据保存器。')

    def stop_collect(event):
        save_event.clear()
        print('已发送停止信号到数据保存器。')

    def quit_program(event):
        stop_event.set()
        save_event.clear()
        print('已发送退出信号，正在退出所有进程。')
    
    btn_start.on_clicked(start_collect)
    btn_stop.on_clicked(stop_collect)
    btn_quit.on_clicked(quit_program)

    def update_plot(frame):
        while not data_queue.empty():
            try:
                data = data_queue.get_nowait()
                ax_data.append(data['ax'])
                ay_data.append(data['ay'])
                az_data.append(data['az'])
                gx_data.append(data['gx'])
                gy_data.append(data['gy'])
                gz_data.append(data['gz'])
                mx_data.append(data['mx'])
                my_data.append(data['my'])
                mz_data.append(data['mz'])
                alt_data.append(data['alt'])
            except Exception as e:
                print(f"数据更新错误: {e}")
                
        l_ax.set_ydata(ax_data)
        l_ay.set_ydata(ay_data)
        l_az.set_ydata(az_data)
        l_gx.set_ydata(gx_data)
        l_gy.set_ydata(gy_data)
        l_gz.set_ydata(gz_data)
        l_mx.set_ydata(mx_data)
        l_my.set_ydata(my_data)
        l_mz.set_ydata(mz_data)
        l_alt.set_ydata(alt_data)
        
        return l_ax, l_ay, l_az, l_gx, l_gy, l_gz, l_mx, l_my, l_mz, l_alt,

    ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)

    plt.show()

    print("GUI 进程已停止。")

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

    gui_plotter(data_queue, stop_event, save_event, path_queue, filename_queue)

    collector_process.join()
    saver_process.join()
    print("程序已完全退出。")