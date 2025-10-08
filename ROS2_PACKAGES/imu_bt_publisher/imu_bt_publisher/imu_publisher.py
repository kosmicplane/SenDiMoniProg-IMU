import rclpy
import re
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Float32
import serial
import time

class BluetoothIMUPublisher(Node):
    def __init__(self):
        super().__init__('bluetooth_imu_publisher')

        # --- Publishers ---
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.pres_pub = self.create_publisher(FluidPressure, 'imu/pressure', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        self.alt_pub = self.create_publisher(Float32, 'imu/altitude', 10)

        # --- Serial (Bluetooth RFCOMM) ---
        self.port = "/dev/rfcomm0"       # creado en el host Jetson
        self.baudrate = 230400           # igual que en el ESP32
        self.ser = None
        self.connect_serial()

        self.timer = self.create_timer(0.02, self.read_data)  # ~50 Hz

    # ---- Conexión Serial ----
    def connect_serial(self):
        while self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
                self.get_logger().info(f"✅ Connected to {self.port}")
            except Exception as e:
                self.get_logger().error(f"Retrying connection: {e}")
                time.sleep(2)

    # ---- Lectura periódica ----
    def read_data(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            self.process_line(line)
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")


    def process_line(self, line):
    try:
        # Extrae todos los números flotantes de la línea
        values = re.findall(r"[-+]?\d*\.\d+|\d+", line)
        if len(values) < 12:
            # Muestra debug si no hay suficientes valores
            print(f"⚠️  Línea incompleta ({len(values)} valores): {line}")
            return

        # Convierte los primeros 12 a float
        ax, ay, az, gx, gy, gz, mx, my, mz, pressure, temperature, altitude = map(float, values[:12])

        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.linear_acceleration.x = ax * 9.80665
        imu_msg.linear_acceleration.y = ay * 9.80665
        imu_msg.linear_acceleration.z = az * 9.80665
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        self.imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header = imu_msg.header
        mag_msg.magnetic_field.x = mx * 1e-6
        mag_msg.magnetic_field.y = my * 1e-6
        mag_msg.magnetic_field.z = mz * 1e-6
        self.mag_pub.publish(mag_msg)

        pres_msg = FluidPressure()
        pres_msg.header = imu_msg.header
        pres_msg.fluid_pressure = pressure * 100.0
        self.pres_pub.publish(pres_msg)

        temp_msg = Temperature()
        temp_msg.header = imu_msg.header
        temp_msg.temperature = temperature
        self.temp_pub.publish(temp_msg)

        alt_msg = Float32()
        alt_msg.data = altitude
        self.alt_pub.publish(alt_msg)

        print(f"✅ Publicado: {ax:.3f}, {ay:.3f}, {az:.3f}, {temperature:.2f}°C, {altitude:.1f}m")

    except Exception as e:
        self.get_logger().warn(f"Parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothIMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
