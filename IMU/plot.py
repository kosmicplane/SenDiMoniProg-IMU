import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from collections import deque
from multiprocessing import Process, Queue, Event
import matplotlib
import time

# --- Matplotlib Configuration ---
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False 

# --- Data Collector Function ---
def data_collector(port_name, baud_rate, data_queue, stop_event):
    """
    Responsible for reading data from the serial port and putting it into a queue.
    This function runs in a separate process.
    """
    ser = None
    try:
        ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)
        print(f"Serial port {port_name} opened successfully.")
    except serial.SerialException as e:
        print(f"Could not open serial port {port_name}: {e}")
        stop_event.set()
        return

    while not stop_event.is_set():
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('A[g]:'):
                parts = line.split('|')
                a_vals = parts[0].replace('A[g]:', '').split(',')
                g_vals = parts[1].replace('G[rad/s]:', '').split(',')
                m_vals = parts[2].replace('M[mG]:', '').split(',')
                alt_val = parts[5].replace('Alt[m]:', '').strip()

                parsed_data = {
                    'ax': float(a_vals[0]), 'ay': float(a_vals[1]), 'az': float(a_vals[2]),
                    'gx': float(g_vals[0]), 'gy': float(g_vals[1]), 'gz': float(g_vals[2]),
                    'mx': float(m_vals[0]), 'my': float(m_vals[1]), 'mz': float(m_vals[2]),
                    'alt': float(alt_val),
                }
                data_queue.put(parsed_data)

        except (serial.SerialException, ValueError) as e:
            print(f"Data read or parse error: {e}")
            break
        
    if ser and ser.is_open:
        ser.close()
    print("Data collection process has stopped.")

# --- GUI and Plotting Function ---
def gui_plotter(data_queue, stop_event):
    """
    Responsible for plotting the charts and handling GUI interactions.
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

    fig, axs = plt.subplots(4, 1, figsize=(8, 10))
    fig.canvas.manager.set_window_title('IMU Real-time Monitor')

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

    # Quit button
    ax_btn_quit = plt.axes([0.4, 0.05, 0.2, 0.05])
    btn_quit = Button(ax_btn_quit, 'Quit')

    def quit_program(event):
        stop_event.set()
        print('Quit signal sent, exiting...')
    
    btn_quit.on_clicked(quit_program)

    def update_plot(frame):
        """
        This function is called periodically by FuncAnimation to update plot data.
        """
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
                print(f"Data update error: {e}")
                
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
        
        # Return all artist objects that need to be updated
        return l_ax, l_ay, l_az, l_gx, l_gy, l_gz, l_mx, l_my, l_mz, l_alt,

    # Use FuncAnimation to drive the plot updates
    ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=True, cache_frame_data=False)

    plt.show()

    print("GUI process has stopped.")

# --- Main Program Entry ---
if __name__ == '__main__':
    data_queue = Queue()
    stop_event = Event()

    collector_process = Process(target=data_collector, args=('/dev/ttyUSB0', 230400, data_queue, stop_event))
    collector_process.daemon = True
    collector_process.start()

    gui_plotter(data_queue, stop_event)

    collector_process.join()
    print("Program has fully exited.")