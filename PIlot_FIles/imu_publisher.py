#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
from builtin_interfaces.msg import Time
from ahrs.filters import Madgwick
import numpy as np

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Puerto serial del ESP32 (ajusta al tuyo)
        self.ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)

        # Timer a 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.madgwick = Madgwick()
        self.q = np.array({1.0,0.0,0.0,0.0})

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8').strip()
        if not line:
            return

        try:
            # Espera CSV: ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,alt
            values = list(map(float, line.split(',')))
            ax, ay, az, gx, gy, gz, mx, my, mz, pressure, temp, alt = values

            imu_msg = Imu()

            # Aceleración
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            # Giroscopio
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            # Orientación (ejemplo: usamos magnetómetro como placeholder)
            # ⚠️ Mejor integrar un filtro de fusión como Madgwick/Mahony
            # Orientación (ejemplo: usamos magnetómetro como placeholder)
            qx, qy, qz, qw = self.euler_to_quaternion(mx, my, mz)

            # Paso 2: Normalizar cuaternión
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if norm > 0.0:  # evitar división por cero
                qx /= norm
                qy /= norm
                qz /= norm
                qw /= norm

            imu_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

            # Publicar
            
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            self.publisher_.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convierte ángulos de Euler (rad) a quaternion (x,y,z,w)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
