#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Float32
import serial
import serial.serialutil
import time

G_TO_MS2 = 9.80665
DEG2RAD = math.pi / 180.0
UT_TO_T = 1e-6

class BluetoothIMUPublisher(Node):
    def __init__(self):
        super().__init__('bluetooth_imu_publisher')

        # --- Params (puedes override con ros2 run ... --ros-args -p port:=/dev/rfcomm1 -p baudrate:=115200) ---
        self.declare_parameter('port', '/dev/rfcomm0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'imu_link')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- Publishers ---
        self.imu_pub  = self.create_publisher(Imu,            'imu/data_raw',    10)
        self.mag_pub  = self.create_publisher(MagneticField,  'imu/mag',         10)
        self.pres_pub = self.create_publisher(FluidPressure,  'imu/pressure',    10)
        self.temp_pub = self.create_publisher(Temperature,    'imu/temperature', 10)
        self.alt_pub  = self.create_publisher(Float32,        'imu/altitude',    10)

        # --- Serial ---
        self.ser = None
        self.connect_serial()

        # ~20Hz en tu ESP32 (delay 50 ms). Podemos leer a 50 Hz sin problema.
        self.timer = self.create_timer(0.02, self.read_data)

    # ---- Conexión Serial con reintentos ----
    def connect_serial(self):
        while rclpy.ok():
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
                self.get_logger().info(f"✅ Connected to {self.port} @ {self.baudrate}")
                return
            except Exception as e:
                self.get_logger().error(f"Serial open failed: {e}; retrying in 2s...")
                time.sleep(2)

    # ---- Lectura periódica ----
    def read_data(self):
        if self.ser is None:
            self.connect_serial()
            return
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            self.process_line(line)
        except (serial.SerialException, serial.serialutil.SerialException) as e:
            self.get_logger().warn(f"Serial error: {e}; reconnecting...")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

    # ---- Parseo y publicación ----
    def process_line(self, line: str):
        # Esperamos 12 valores CSV exactos
        parts = [p.strip() for p in line.split(',')]
        if len(parts) != 12:
            # Debug mínimo para no inundar logs si hay ruido
            self.get_logger().debug(f"Línea ignorada (campos={len(parts)}): {line}")
            return

        try:
            # Orden EXACTO del ESP32:
            # 0 ax_g, 1 ay_g, 2 az_g,
            # 3 gx_dps, 4 gy_dps, 5 gz_dps,
            # 6 mx_uT, 7 my_uT, 8 mz_uT,
            # 9 pressure, 10 altitude_m, 11 tempC
            ax_g, ay_g, az_g = float(parts[0]), float(parts[1]), float(parts[2])
            gx_dps, gy_dps, gz_dps = float(parts[3]), float(parts[4]), float(parts[5])
            mx_uT, my_uT, mz_uT = float(parts[6]), float(parts[7]), float(parts[8])
            pressure = float(parts[9])      # asumo hPa
            altitude_m = float(parts[10])
            tempC = float(parts[11])

            # --- Mensaje IMU ---
            now = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.frame_id = self.frame_id
            imu_msg.header.stamp = now

            # Aceleración en m/s^2 (de g a SI)
            imu_msg.linear_acceleration.x = ax_g * G_TO_MS2
            imu_msg.linear_acceleration.y = ay_g * G_TO_MS2
            imu_msg.linear_acceleration.z = az_g * G_TO_MS2
            # Covarianzas desconocidas
            imu_msg.linear_acceleration_covariance[0] = -1.0

            # Velocidad angular en rad/s (de dps a SI)  **CORRECCIÓN**
            imu_msg.angular_velocity.x = gx_dps * DEG2RAD
            imu_msg.angular_velocity.y = gy_dps * DEG2RAD
            imu_msg.angular_velocity.z = gz_dps * DEG2RAD
            imu_msg.angular_velocity_covariance[0] = -1.0

            # Sin orientación (no fusion aquí)
            imu_msg.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu_msg)

            # --- Magnetómetro en Teslas ---
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = mx_uT * UT_TO_T
            mag_msg.magnetic_field.y = my_uT * UT_TO_T
            mag_msg.magnetic_field.z = mz_uT * UT_TO_T
            mag_msg.magnetic_field_covariance[0] = -1.0
            self.mag_pub.publish(mag_msg)

            # --- Presión (ROS espera Pascales). Si tu sensor da hPa, *100 ---
            pres_msg = FluidPressure()
            pres_msg.header.stamp = now
            pres_msg.header.frame_id = self.frame_id
            pres_msg.fluid_pressure = pressure * 100.0  # hPa → Pa
            pres_msg.variance = 0.0
            self.pres_pub.publish(pres_msg)

            # --- Temperatura (°C) ---
            temp_msg = Temperature()
            temp_msg.header.stamp = now
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = tempC
            temp_msg.variance = 0.0
            self.temp_pub.publish(temp_msg)

            # --- Altitud (m) ---
            alt_msg = Float32()
            alt_msg.data = altitude_m
            self.alt_pub.publish(alt_msg)

            self.get_logger().debug(
                f"Pub IMU a/g:({ax_g:.3f},{ay_g:.3f},{az_g:.3f}) "
                f"gyro_dps:({gx_dps:.2f},{gy_dps:.2f},{gz_dps:.2f}) "
                f"T:{tempC:.2f}°C alt:{altitude_m:.2f} m"
            )

        except ValueError as e:
            # Algún campo no es float válido
            self.get_logger().debug(f"Parse ValueError: {e} | line: {line}")
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothIMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
