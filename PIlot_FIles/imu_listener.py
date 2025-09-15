#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.listener_callback,
            10
        )
        self.subscription  # evitar warning de variable sin usar

    def listener_callback(self, msg):
        # Aceleración y giroscopio
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # Quaternion → Euler
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

        self.get_logger().info(
            f"Acc[g]: ({ax:.3f}, {ay:.3f}, {az:.3f}) | "
            f"Gyro[rad/s]: ({gx:.3f}, {gy:.3f}, {gz:.3f}) | "
            f"Euler[rad]: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}"
        )

    def quaternion_to_euler(self, x, y, z, w):
        # Fórmula estándar ROS 2
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = ImuListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
