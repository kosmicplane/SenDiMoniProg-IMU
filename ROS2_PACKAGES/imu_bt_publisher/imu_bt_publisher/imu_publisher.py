#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Float32
import serial, serial.serialutil

G_TO_MS2 = 9.80665
DEG2RAD  = math.pi / 180.0
UT_TO_T  = 1e-6

# CSV esperado (12 campos en este orden):
# ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, mx_uT, my_uT, mz_uT, pressure(hPa), altitude_m, tempC

class BluetoothIMUPublisher(Node):
    def __init__(self, port="/dev/rfcomm0", baudrate=230400, frame_id="imu_link"):
        super().__init__('bluetooth_imu_publisher')
        self.port = port
        self.baud = baudrate
        self.frame_id = frame_id

        # Publishers
        self.imu_pub  = self.create_publisher(Imu,           'imu/data_raw',    10)
        self.mag_pub  = self.create_publisher(MagneticField, 'imu/mag',         10)
        self.pres_pub = self.create_publisher(FluidPressure, 'imu/pressure',    10)
        self.temp_pub = self.create_publisher(Temperature,   'imu/temperature', 10)
        self.alt_pub  = self.create_publisher(Float32,       'imu/altitude',    10)

        self.ser = None
        self._connect_serial()
        self.create_timer(0.02, self._read_tick)  # 50 Hz

    def _connect_serial(self):
        while rclpy.ok() and self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                self.get_logger().info(f"✅ Connected to {self.port} @ {self.baud}")
            except Exception as e:
                self.get_logger().error(f"Serial open failed: {e}; retrying in 2s...")
                time.sleep(2)

    def _read_tick(self):
        if self.ser is None:
            self._connect_serial(); return
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            self._process_line(line)
        except (serial.SerialException, serial.serialutil.SerialException) as e:
            self.get_logger().warn(f"Serial error: {e}; reconnecting...")
            try: self.ser.close()
            except: pass
            self.ser = None
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

    def _process_line(self, line: str):
        parts = [p.strip() for p in line.split(',')]
        if len(parts) != 12:
            self.get_logger().debug(f"Ignoro línea (campos={len(parts)}): {line}")
            return
        try:
            ax_g, ay_g, az_g = map(float, parts[0:3])
            gx_dps, gy_dps, gz_dps = map(float, parts[3:6])
            mx_uT, my_uT, mz_uT = map(float, parts[6:9])
            pressure_hpa = float(parts[9])
            altitude_m   = float(parts[10])
            tempC        = float(parts[11])

            now = self.get_clock().now().to_msg()

            # IMU
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = self.frame_id
            imu.linear_acceleration.x = ax_g * G_TO_MS2
            imu.linear_acceleration.y = ay_g * G_TO_MS2
            imu.linear_acceleration.z = az_g * G_TO_MS2
            imu.angular_velocity.x = gx_dps * DEG2RAD
            imu.angular_velocity.y = gy_dps * DEG2RAD
            imu.angular_velocity.z = gz_dps * DEG2RAD
            imu.linear_acceleration_covariance[0] = -1.0
            imu.angular_velocity_covariance[0] = -1.0
            imu.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu)

            # Magnetómetro (T)
            mag = MagneticField()
            mag.header.stamp = now
            mag.header.frame_id = self.frame_id
            mag.magnetic_field.x = mx_uT * UT_TO_T
            mag.magnetic_field.y = my_uT * UT_TO_T
            mag.magnetic_field.z = mz_uT * UT_TO_T
            mag.magnetic_field_covariance[0] = -1.0
            self.mag_pub.publish(mag)

            # Presión (Pa) – si ya viene en Pa, quita *100
            prs = FluidPressure()
            prs.header.stamp = now
            prs.header.frame_id = self.frame_id
            prs.fluid_pressure = pressure_hpa * 100.0
            prs.variance = 0.0
            self.pres_pub.publish(prs)

            # Temperatura (°C)
            tt = Temperature()
            tt.header.stamp = now
            tt.header.frame_id = self.frame_id
            tt.temperature = tempC
            tt.variance = 0.0
            self.temp_pub.publish(tt)

            # Altitud (m)
            alt = Float32()
            alt.data = altitude_m
            self.alt_pub.publish(alt)

        except Exception as e:
            self.get_logger().debug(f"Parse error: {e} | line: {line}")

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/rfcomm0')
    ap.add_argument('--baudrate', type=int, default=230400)
    ap.add_argument('--frame_id', default='imu_link')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = BluetoothIMUPublisher(args.port, args.baudrate, args.frame_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser: node.ser.close()
        except: pass
        node.destroy_node()
