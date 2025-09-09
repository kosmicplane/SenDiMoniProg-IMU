import time
import bluetooth
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import matplotlib
import re
import select
from multiprocessing import Process, Queue, Event

# --- Matplotlib Configuration ---
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False 

# --- Bluetooth Connection Setup ---
bd_addr = "70:B8:F6:67:0E:E6"
port = 1
data_buffer = ""

def data_collector(bd_addr, port, data_queue, stop_event):
    """
    Responsible for reading data from the Bluetooth port and putting it into a queue.
    This function runs in a separate process.
    """
    sock = None
    try:
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((bd_addr, port))
        sock.setblocking(0)
        print("Data collection process: Bluetooth connection successful!")
    except bluetooth.btcommon.BluetoothError as err:
        print(f"Data collection process: Bluetooth connection failed: {err}")
        stop_event.set()
        return

    data_buffer = ""
    while not stop_event.is_set():
        try:
            ready_to_read, _, _ = select.select([sock], [], [], 0)
            if ready_to_read:
                data = sock.recv(1024)
                data_buffer += data.decode('utf-8', errors='ignore')

            while '\n' in data_buffer:
                line, data_buffer = data_buffer.split('\n', 1)
                match = re.search(
                    r'A\[g\]: (.*?), (.*?), (.*?) \| G\[rad/s\]: (.*?), (.*?), (.*?) \| M\[mG\]: (.*?), (.*?), (.*?) \| P\[mbar\]: .* \| T\[C\]: .* \| Alt\[m\]: (.*)',
                    line
                )
                if match:
                    parsed_data = {
                        'ax': float(match.group(1)), 'ay': float(match.group(2)), 'az': float(match.group(3)),
                        'gx': float(match.group(4)), 'gy': float(match.group(5)), 'gz': float(match.group(6)),
                        'mx': float(match.group(7)), 'my': float(match.group(8)), 'mz': float(match.group(9)),
                        'alt': float(match.group(10)),
                    }
                    data_queue.put(parsed_data)

        except (bluetooth.btcommon.BluetoothError, ValueError) as e:
            print(f"Data collection process: Read or parse error: {e}")
            stop_event.set()
            break
        
    if sock:
        sock.close()
    print("Data collection process has stopped.")

def gui_plotter(data_queue, stop_event):
    """
    Responsible for plotting the charts and handling the GUI.
    This function runs in the main process.
    """
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

    fig, axs = plt.subplots(4, 1, figsize=(10, 12))
    fig.canvas.manager.set_window_title('IMU Real-time Monitor')
    
    l_ax, = axs[0].plot(ax_data, label='Ax', color='cyan')
    l_ay, = axs[0].plot(ay_data, label='Ay', color='magenta')
    l_az, = axs[0].plot(az_data, label='Az', color='yellow')
    axs[0].legend(loc='upper left')
    axs[0].set_title('Accelerometer (g)', color='black')
    axs[0].set_ylim(-5, 5)
    axs[0].set_yticks([-5, -2.5, 0, 2.5, 5])
    
    l_gx, = axs[1].plot(gx_data, label='Gx', color='cyan')
    l_gy, = axs[1].plot(gy_data, label='Gy', color='magenta')
    l_gz, = axs[1].plot(gz_data, label='Gz', color='yellow')
    axs[1].legend(loc='upper left')
    axs[1].set_title('Gyroscope (rad/s)', color='black')
    axs[1].set_ylim(-10, 10)
    axs[1].set_yticks([-10, -5, 0, 5, 10])
    
    l_mx, = axs[2].plot(mx_data, label='Mx', color='cyan')
    l_my, = axs[2].plot(my_data, label='My', color='magenta')
    l_mz, = axs[2].plot(mz_data, label='Mz', color='yellow')
    axs[2].legend(loc='upper left')
    axs[2].set_title('Magnetometer (mG)', color='black')
    axs[2].set_ylim(-1000, 1000)
    axs[2].set_yticks([-1000, -500, 0, 500, 1000])
    
    l_alt, = axs[3].plot(alt_data, label='Altitude (m)', color='lime')
    axs[3].legend(loc='upper left')
    axs[3].set_title('Altitude (m)', color='black')
    axs[3].set_ylim(100, 200)
    axs[3].set_yticks([100, 125, 150, 175, 200])
    
    for ax in axs:
        ax.tick_params(axis='x', colors='black')
        ax.tick_params(axis='y', colors='black')
        ax.spines['left'].set_color('black')
        ax.spines['bottom'].set_color('black')
        ax.spines['right'].set_color('black')
        ax.spines['top'].set_color('black')
    plt.tight_layout()

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
                print(f"GUI process: Data update error: {e}")

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
        
        axs[0].set_ylim(-5, 5)
        axs[1].set_ylim(-10, 10)
        axs[2].set_ylim(-1000, 1000)
        axs[3].set_ylim(100, 200)

        return l_ax, l_ay, l_az, l_gx, l_gy, l_gz, l_mx, l_my, l_mz, l_alt,
    
    ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    stop_event.set()
    print("GUI process has stopped.")

# --- Main Program Entry ---
if __name__ == '__main__':
    data_queue = Queue()
    stop_event = Event()

    collector_process = Process(target=data_collector, args=(bd_addr, port, data_queue, stop_event))
    collector_process.daemon = True
    collector_process.start()
    
    try:
        gui_plotter(data_queue, stop_event)
    except Exception as e:
        print(f"Main process error: {e}")
    finally:
        collector_process.join
