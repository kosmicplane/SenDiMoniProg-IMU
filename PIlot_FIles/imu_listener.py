#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import numpy as np
from ahrs.filters import Madgwick

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener_fused')

        # --- Subscribe to raw IMU data ---
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.listener_callback, 20)

        # --- Publisher for fused IMU with orientation ---
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # --- Madgwick filter setup ---
        self.madgwick = Madgwick(beta=0.05, frequency=20.0)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # initial quaternion [w, x, y, z]

        self.get_logger().info("IMU fusion node running: /imu/data_raw → /imu/data")

    def listener_callback(self, msg):
        # --- Extract acceleration and gyroscope data ---
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # Convert to numpy arrays
        acc = np.array([ax, ay, az], dtype=float)
        gyr = np.array([gx, gy, gz], dtype=float)

        # Normalize accelerometer vector
        norm_acc = np.linalg.norm(acc)
        if norm_acc > 1e-6:
            acc /= norm_acc

        # --- Update orientation using Madgwick filter ---
        # This uses only accel + gyro. If you have magnetometer, you can extend it with update() instead.
        self.q = self.madgwick.updateIMU(self.q, gyr=gyr, acc=acc)

        # --- Build and publish new IMU message with orientation ---
        fused = Imu()
        fused.header = msg.header
        fused.header.frame_id = "imu_link"

        # Quaternion (convert from w,x,y,z to ROS x,y,z,w)
        fused.orientation.w = self.q[0]
        fused.orientation.x = self.q[1]
        fused.orientation.y = self.q[2]
        fused.orientation.z = self.q[3]

        fused.angular_velocity = msg.angular_velocity
        fused.linear_acceleration = msg.linear_acceleration

        self.pub.publish(fused)

        # --- Compute Euler angles (degrees) ---
        roll, pitch, yaw = self.quaternion_to_euler(self.q[1], self.q[2], self.q[3], self.q[0])
        roll, pitch, yaw = map(math.degrees, [roll, pitch, yaw])

        # --- Console output ---
        self.get_logger().info(
            f"Acc[g]: ({ax:.3f}, {ay:.3f}, {az:.3f}) | "
            f"Gyro[rad/s]: ({gx:.3f}, {gy:.3f}, {gz:.3f}) | "
            f"Euler[deg]: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}"
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)."""
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
    node = ImuListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
