#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Float32
import serial
import time

G_TO_MS2 = 9.80665     # g → m/s²
UT_TO_T  = 1e-6        # μT → T

# FORMATO NUEVO (12 campos, separados por coma):
# ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, mx_uT, my_uT, mz_uT, pressure(hPa), altitude_m, tempC

class BluetoothIMUPublisher(Node):
    def __init__(self):
        super().__init__('bluetooth_imu_publisher')

        # --- Publishers ---
        self.imu_pub  = self.create_publisher(Imu,           'imu/data_raw',    10)
        self.mag_pub  = self.create_publisher(MagneticField, 'imu/mag',         10)
        self.pres_pub = self.create_publisher(FluidPressure, 'imu/pressure',    10)
        self.temp_pub = self.create_publisher(Temperature,   'imu/temperature', 10)
        self.alt_pub  = self.create_publisher(Float32,       'imu/altitude',    10)

        # --- Serial (Bluetooth RFCOMM) ---
        self.port = "/dev/rfcomm0"   # ajusta si usas otro
        self.baudrate = 230400
        self.ser = None
        self.connect_serial()

        self.timer = self.create_timer(0.02, self.read_data)  # ~50 Hz

    def connect_serial(self):
        while self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
                self.get_logger().info(f"✅ Connected to {self.port}")
            except Exception as e:
                self.get_logger().error(f"Retrying connection: {e}")
                time.sleep(2)

    def read_data(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            self.process_line(line)
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

    def process_line(self, line: str):
        try:
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 12:
                return  # línea incompleta o ruido

            # Mapeo EXACTO según el nuevo formato
            ax_g, ay_g, az_g = map(float, parts[0:3])
            gx_dps, gy_dps, gz_dps = map(float, parts[3:6])
            mx_uT, my_uT, mz_uT = map(float, parts[6:9])
            pressure_hpa = float(parts[9])
            altitude_m   = float(parts[10])
            tempC        = float(parts[11])

            now = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.header.stamp = now
            # Aceleración en m/s²
            imu_msg.linear_acceleration.x = ax_g * G_TO_MS2
            imu_msg.linear_acceleration.y = ay_g * G_TO_MS2
            imu_msg.linear_acceleration.z = az_g * G_TO_MS2
            # Velocidad angular (se deja en deg/s como en tu código original)
            imu_msg.angular_velocity.x = gx_dps
            imu_msg.angular_velocity.y = gy_dps
            imu_msg.angular_velocity.z = gz_dps
            self.imu_pub.publish(imu_msg)

            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            mag_msg.magnetic_field.x = mx_uT * UT_TO_T
            mag_msg.magnetic_field.y = my_uT * UT_TO_T
            mag_msg.magnetic_field.z = mz_uT * UT_TO_T
            self.mag_pub.publish(mag_msg)

            pres_msg = FluidPressure()
            pres_msg.header = imu_msg.header
            pres_msg.fluid_pressure = pressure_hpa * 100.0  # hPa → Pa (si ya viene en Pa, quita *100)
            self.pres_pub.publish(pres_msg)

            temp_msg = Temperature()
            temp_msg.header = imu_msg.header
            temp_msg.temperature = tempC
            self.temp_pub.publish(temp_msg)

            alt_msg = Float32()
            alt_msg.data = altitude_m
            self.alt_pub.publish(alt_msg)

            print(f"✅ Pub: a=({ax_g:.3f},{ay_g:.3f},{az_g:.3f}) g, "
                  f"gyr=({gx_dps:.2f},{gy_dps:.2f},{gz_dps:.2f}) dps, "
                  f"T={tempC:.2f}°C, alt={altitude_m:.2f} m")

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
