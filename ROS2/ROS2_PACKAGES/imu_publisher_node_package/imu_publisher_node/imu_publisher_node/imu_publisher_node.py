#!/usr/bin/env python3
"""ROS 2 node that reads IMU data from an ESP32 over RFCOMM (/dev/rfcomm0)
and publishes standard ROS 2 IMU-related topics.

This file is adapted from the original imu_publisher.py in the SenDiMoniProg-IMU
project, wrapped as a self-contained ament_python package.
"""

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


class BluetoothIMUPublisher(Node):
    """Node that reads CSV IMU data from /dev/rfcomm0 and publishes ROS topics."""

    def __init__(self):
        super().__init__('bluetooth_imu_publisher')

        # Publishers
        self.imu_raw_pub   = self.create_publisher(Imu,           'imu/data_raw',    10)
        self.imu_fused_pub = self.create_publisher(Imu,           'imu/data',        10)
        self.mag_pub       = self.create_publisher(MagneticField, 'imu/mag',         10)
        self.pres_pub      = self.create_publisher(FluidPressure, 'imu/pressure',    10)
        self.temp_pub      = self.create_publisher(Temperature,   'imu/temperature', 10)
        self.alt_pub       = self.create_publisher(Float32,       'imu/altitude',    10)

        # Serial port configuration
        self.port = "/dev/rfcomm0"
        self.baudrate = 230400
        self.ser = None
        self.connect_serial()

        # State for Madgwick fusion and logging
        self.last_t = time.monotonic()
        self.last_print_ns = 0
        self.print_period_ns = int(1e9 / 10)  # 10 Hz
        self.madgwick = Madgwick(beta=0.2, frequency=50)  # Initial frequency, updated dynamically
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        # Buffer for partial lines coming from serial
        self.rx_buf = ""

        # Timer at 50 Hz (0.02s)
        self.timer = self.create_timer(0.02, self.read_data)
        self.get_logger().info("✅ IMU Bluetooth publisher with Madgwick fusion started")

    def connect_serial(self):
        """Try to connect to the RFCOMM serial port with retries."""
        while self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.0)
                self.get_logger().info(f"✅ Connected to {self.port}")
            except Exception as e:
                self.get_logger().error(f"Retrying connection: {e}")
                time.sleep(2)

    def read_data(self):
        """Read available bytes from the serial buffer and process complete lines."""
        try:
            n = self.ser.in_waiting
            if n == 0:
                return

            data = self.ser.read(n).decode(errors='ignore')
            self.rx_buf += data

            # Process all complete lines and keep the last one
            last_line = None
            while '\n' in self.rx_buf:
                line, self.rx_buf = self.rx_buf.split('\n', 1)
                line = line.strip()
                if line:
                    last_line = line

            if last_line is not None:
                self.process_line(last_line)

        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

    def process_line(self, line: str):
        """Parse one CSV line of IMU data and publish ROS 2 messages."""
        try:
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 12:
                return

            # --- Parse CSV fields ---
            ax_g, ay_g, az_g = map(float, parts[0:3])
            gx_dps, gy_dps, gz_dps = map(float, parts[3:6])
            mx_uT, my_uT, mz_uT = map(float, parts[6:9])

            pressure_hpa = float(parts[9])
            tempC        = float(parts[10])
            altitude_m   = float(parts[11])

            now = self.get_clock().now().to_msg()

            # --- Raw IMU message (SI units) ---
            imu_raw = Imu()
            imu_raw.header.frame_id = "imu_link"
            imu_raw.header.stamp = now

            imu_raw.linear_acceleration.x = ax_g * G_TO_MS2
            imu_raw.linear_acceleration.y = ay_g * G_TO_MS2
            imu_raw.linear_acceleration.z = az_g * G_TO_MS2

            # deg/s → rad/s
            imu_raw.angular_velocity.x = math.radians(gx_dps)
            imu_raw.angular_velocity.y = math.radians(gy_dps)
            imu_raw.angular_velocity.z = math.radians(gz_dps)

            self.imu_raw_pub.publish(imu_raw)

            # --- MagneticField message (Tesla) ---
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

            # --- Madgwick fusion ---
            acc = np.array([ax_g, ay_g, az_g])
            gyr = np.radians(np.array([gx_dps, gy_dps, gz_dps]))  # rad/s
            mag = np.array([mx_uT, my_uT, mz_uT]) * UT_TO_T       # Tesla

            now_t = time.monotonic()
            dt = now_t - self.last_t
            self.last_t = now_t
            # Avoid division by zero
            self.madgwick.frequency = 1.0 / max(dt, 1e-3)

            # Use updateMARG for full IMU + magnetometer fusion
            self.q = self.madgwick.updateMARG(self.q, gyr=gyr, acc=acc, mag=mag)

            imu_fused = Imu()
            imu_fused.header = imu_raw.header
            imu_fused.orientation.w = float(self.q[0])
            imu_fused.orientation.x = float(self.q[1])
            imu_fused.orientation.y = float(self.q[2])
            imu_fused.orientation.z = float(self.q[3])
            imu_fused.angular_velocity = imu_raw.angular_velocity
            imu_fused.linear_acceleration = imu_raw.linear_acceleration
            self.imu_fused_pub.publish(imu_fused)

            # --- Euler angles (for logging, in degrees) ---
            roll, pitch, yaw = self.quaternion_to_euler(
                self.q[1], self.q[2], self.q[3], self.q[0]
            )
            roll_deg, pitch_deg, yaw_deg = map(math.degrees, [roll, pitch, yaw])

            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_print_ns > self.print_period_ns:
                self.last_print_ns = now_ns
                self.get_logger().info(
                    f"Roll={roll_deg:6.2f}°, Pitch={pitch_deg:6.2f}°, Yaw={yaw_deg:6.2f}° | "  # noqa: E501
                    f"[RAW] ax_g={ax_g:+7.3f}, ay_g={ay_g:+7.3f}, az_g={az_g:+7.3f} g | "      # noqa: E501
                    f"gx={gx_dps:+8.3f}, gy={gy_dps:+8.3f}, gz={gz_dps:+8.3f} dps | "         # noqa: E501
                    f"mx={mx_uT:+9.3f}, my={my_uT:+9.3f}, mz={mz_uT:+9.3f} uT | "             # noqa: E501
                    f"p={pressure_hpa:8.2f} hPa, T={tempC:6.2f} C, Alt={altitude_m:7.2f} m"   # noqa: E501
                )

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def quaternion_to_euler(self, x, y, z, w):
        """Convert a quaternion into roll, pitch, yaw (radians)."""
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
    """Main entry point for the ROS 2 node."""
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
