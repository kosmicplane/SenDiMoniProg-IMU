#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Float32
import serial
import time
import numpy as np
import math
from ahrs.filters import Madgwick

# Constants
G_TO_MS2 = 9.80665     # g → m/s²
UT_TO_T  = 1e-6        # μT → T

# Expected CSV format:
# ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, mx_uT, my_uT, mz_uT, pressure(hPa), altitude_m, tempC

class BluetoothIMUPublisher(Node):
    def __init__(self):
        super().__init__('bluetooth_imu_publisher')

        # --- Publishers ---
        self.imu_raw_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.imu_fused_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub  = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.pres_pub = self.create_publisher(FluidPressure, 'imu/pressure', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        self.alt_pub  = self.create_publisher(Float32, 'imu/altitude', 10)

        # --- Serial Bluetooth configuration ---
        self.port = "/dev/rfcomm0"
        self.baudrate = 230400
        self.ser = None
        self.connect_serial()

        # --- Madgwick filter initialization ---
        self.madgwick = Madgwick(beta=0.05, frequency=50.0)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # initial quaternion [w, x, y, z]

        # Timer ~50 Hz
        self.timer = self.create_timer(0.02, self.read_data)

        self.get_logger().info("✅ IMU Bluetooth publisher with Madgwick fusion started")

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
                return  # Incomplete or noisy line

            # --- Parse CSV fields ---
            ax_g, ay_g, az_g = map(float, parts[0:3])
            gx_dps, gy_dps, gz_dps = map(float, parts[3:6])
            mx_uT, my_uT, mz_uT = map(float, parts[6:9])
            pressure_hpa = float(parts[9])
            altitude_m   = float(parts[10])
            tempC        = float(parts[11])

            now = self.get_clock().now().to_msg()

            # --- Build raw IMU message (no orientation) ---
            imu_raw = Imu()
            imu_raw.header.frame_id = "imu_link"
            imu_raw.header.stamp = now

            imu_raw.linear_acceleration.x = ax_g * G_TO_MS2
            imu_raw.linear_acceleration.y = ay_g * G_TO_MS2
            imu_raw.linear_acceleration.z = az_g * G_TO_MS2

            # Convert deg/s → rad/s for ROS compatibility
            imu_raw.angular_velocity.x = math.radians(gx_dps)
            imu_raw.angular_velocity.y = math.radians(gy_dps)
            imu_raw.angular_velocity.z = math.radians(gz_dps)

            self.imu_raw_pub.publish(imu_raw)

            # --- Magnetic field message ---
            mag_msg = MagneticField()
            mag_msg.header = imu_raw.header
            mag_msg.magnetic_field.x = mx_uT * UT_TO_T
            mag_msg.magnetic_field.y = my_uT * UT_TO_T
            mag_msg.magnetic_field.z = mz_uT * UT_TO_T
            self.mag_pub.publish(mag_msg)

            # --- Pressure, Temperature, Altitude ---
            pres_msg = FluidPressure()
            pres_msg.header = imu_raw.header
            pres_msg.fluid_pressure = pressure_hpa * 100.0  # hPa → Pa
            self.pres_pub.publish(pres_msg)

            temp_msg = Temperature()
            temp_msg.header = imu_raw.header
            temp_msg.temperature = tempC
            self.temp_pub.publish(temp_msg)

            alt_msg = Float32()
            alt_msg.data = altitude_m
            self.alt_pub.publish(alt_msg)

            # --- Madgwick sensor fusion (accelerometer + gyro + mag) ---
            acc = np.array([ax_g, ay_g, az_g]) * G_TO_MS2
            gyr = np.radians(np.array([gx_dps, gy_dps, gz_dps]))  # rad/s
            mag = np.array([mx_uT, my_uT, mz_uT]) * UT_TO_T       # Tesla

            self.q = self.madgwick.updateIMU(self.q, gyr=gyr, acc=acc)
            
            # --- Build fused IMU message (orientation included) ---
            imu_fused = Imu()
            imu_fused.header = imu_raw.header
            imu_fused.orientation.w = self.q[0]
            imu_fused.orientation.x = self.q[1]
            imu_fused.orientation.y = self.q[2]
            imu_fused.orientation.z = self.q[3]
            imu_fused.angular_velocity = imu_raw.angular_velocity
            imu_fused.linear_acceleration = imu_raw.linear_acceleration

            self.imu_fused_pub.publish(imu_fused)

            # --- Compute and print Euler angles ---
            roll, pitch, yaw = self.quaternion_to_euler(self.q[1], self.q[2], self.q[3], self.q[0])
            roll, pitch, yaw = map(math.degrees, [roll, pitch, yaw])

            print(f"✅ Roll={roll:6.2f}°, Pitch={pitch:6.2f}°, Yaw={yaw:6.2f}° | "
                  f"T={tempC:.2f}°C, Alt={altitude_m:.2f} m")

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def quaternion_to_euler(self, x, y, z, w):
        """Standard ROS quaternion to Euler conversion (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, +1.0), -1.0)
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = BluetoothIMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
