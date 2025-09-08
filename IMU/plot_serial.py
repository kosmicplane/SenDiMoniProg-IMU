import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
from collections import deque
from datetime import datetime
import os
import matplotlib
import time
import tkinter as tk
from tkinter import filedialog

matplotlib.rcParams['font.sans-serif'] = ['Microsoft JhengHei']  # 微軟正黑體
matplotlib.rcParams['axes.unicode_minus'] = False

# 確保你的 COM port 和 baud rate 設定正確
ser = None
try:
    ser = serial.Serial('COM5', 230400, timeout=1)
    print("序列埠 COM5 已成功開啟。")
except serial.SerialException as e:
    print(f"無法開啟序列埠 COM5: {e}")

if ser:
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

    plt.ion()
    fig, axs = plt.subplots(4, 1, figsize=(8, 10))
    fig.canvas.manager.set_window_title('IMU monitor')

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

    # 路徑顯示和選擇按鈕
    ax_path_display = plt.axes([0.15, 0.22, 0.5, 0.05])
    tb_path_display = TextBox(ax_path_display, 'Path', initial='')
    tb_path_display.set_active(False)
    
    ax_btn_path = plt.axes([0.67, 0.22, 0.1, 0.05])
    btn_path = Button(ax_btn_path, 'Select Path')

    # 檔案名稱輸入欄
    ax_fname = plt.axes([0.15, 0.15, 0.6, 0.05])
    tb_fname = TextBox(ax_fname, 'Filename', initial='imu_data.txt')

    # 控制按鈕
    ax_btn_start = plt.axes([0.15, 0.01, 0.18, 0.05])
    ax_btn_stop = plt.axes([0.37, 0.01, 0.18, 0.05])
    ax_btn_quit = plt.axes([0.59, 0.01, 0.18, 0.05])
    btn_start = Button(ax_btn_start, 'Start')
    btn_stop = Button(ax_btn_stop, 'End')
    btn_quit = Button(ax_btn_quit, 'Quit')
    
    # 程式狀態變數
    stop_flag = [False]
    collect_flag = [False]
    data_file = [None]
    save_path = ['']

    # 計時器顯示區
    ax_plot_timer = plt.axes([0.8, 0.22, 0.15, 0.05])
    ax_plot_timer.set_xticks([])
    ax_plot_timer.set_yticks([])
    ax_plot_timer.set_facecolor('none')
    plot_timer_text = ax_plot_timer.text(0.5, 0.5, 'Time: 0.00s', ha='center', va='center',
                                         fontsize=12, transform=ax_plot_timer.transAxes)
    plot_start_time = time.time() 

    ax_collect_timer = plt.axes([0.8, 0.15, 0.15, 0.05])
    ax_collect_timer.set_xticks([])
    ax_collect_timer.set_yticks([])
    ax_collect_timer.set_facecolor('none')
    collect_timer_text = ax_collect_timer.text(0.5, 0.5, 'Collect: 0.00s', ha='center', va='center',
                                               fontsize=12, transform=ax_collect_timer.transAxes)
    collect_start_time = [None] 

    def select_path(event):
        root = tk.Tk()
        root.withdraw()
        folder_selected = filedialog.askdirectory()
        if folder_selected:
            save_path[0] = folder_selected
            tb_path_display.set_val(folder_selected)
            print(f'已選擇資料夾: {folder_selected}')
        else:
            print('未選擇任何資料夾。')

    def start_collect(event):
        if not collect_flag[0]:
            if not save_path[0]:
                print('請先選擇一個儲存資料的資料夾。')
                return

            fname = tb_fname.text.strip()
            full_path = os.path.join(save_path[0], fname)
            try:
                data_file[0] = open(full_path, 'w', encoding='utf-8')
                data_file[0].write('t,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Alt\n')
                collect_flag[0] = True
                collect_start_time[0] = time.time()
                print(f'開始收集並存檔：{full_path}')
            except Exception as e:
                print(f'檔案開啟失敗: {e}')

    def stop_collect(event):
        if collect_flag[0]:
            collect_flag[0] = False
            collect_start_time[0] = None
            collect_timer_text.set_text('Collect: 0.00s')
            if data_file[0]:
                data_file[0].close()
                data_file[0] = None
            print('結束收集並關閉檔案')

    def quit_program(event):
        stop_flag[0] = True
        stop_collect(event)
        print('程式已停止')

    btn_start.on_clicked(start_collect)
    btn_stop.on_clicked(stop_collect)
    btn_quit.on_clicked(quit_program)
    btn_path.on_clicked(select_path)

    update_count = 0
    while plt.fignum_exists(fig.number) and not stop_flag[0]:
        try:
            line = ser.readline().decode('utf-8', errors='ignore')
        except serial.SerialException as e:
            print(f"序列埠錯誤: {e}")
            break
        
        elapsed_plot_time = time.time() - plot_start_time
        plot_timer_text.set_text(f'Time: {elapsed_plot_time:.2f}s')

        if line.startswith('A[g]:'):
            try:
                parts = line.split('|')
                a_vals = parts[0].replace('A[g]:', '').split(',')
                g_vals = parts[1].replace('G[rad/s]:', '').split(',')
                m_vals = parts[2].replace('M[mG]:', '').split(',')
                alt_val = parts[5].replace('Alt[m]:', '').strip()

                ax_data.append(float(a_vals[0]))
                ay_data.append(float(a_vals[1]))
                az_data.append(float(a_vals[2]))
                gx_data.append(float(g_vals[0]))
                gy_data.append(float(g_vals[1]))
                gz_data.append(float(g_vals[2]))
                mx_data.append(float(m_vals[0]))
                my_data.append(float(m_vals[1]))
                mz_data.append(float(m_vals[2]))
                alt_data.append(float(alt_val))

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
                
                if collect_flag[0] and collect_start_time[0] is not None:
                    elapsed_collect_time = time.time() - collect_start_time[0]
                    collect_timer_text.set_text(f'Collect: {elapsed_collect_time:.2f}s')

                update_count += 1
                if update_count % 10 == 0:
                    for ax in axs:
                        ax.set_xlim(0, window)
                    plt.draw()
                    plt.pause(0.001)

                if collect_flag[0] and data_file[0]:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    data_str = f"{timestamp},{a_vals[0]},{a_vals[1]},{a_vals[2]},{g_vals[0]},{g_vals[1]},{g_vals[2]},{m_vals[0]},{m_vals[1]},{m_vals[2]},{alt_val}\n"
                    data_file[0].write(data_str)
            except Exception as e:
                print('Parse error:', e)
    
    ser.close()
    plt.ioff()
    plt.show()