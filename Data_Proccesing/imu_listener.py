#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime

class IMURawLogger(Node):
    def __init__(self):
        super().__init__('imu_raw_logger')

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"/root/data/imu_raw_{timestamp}.csv"
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "ax_m_s2", "ay_m_s2", "az_m_s2",
            "gx_rad_s", "gy_rad_s", "gz_rad_s",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w"
        ])
        self.file.flush()

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.callback,
            10
        )

        self.get_logger().info(f"📡 Listening to /imu/data_raw — saving to {self.filename}")

    def callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ox = msg.orientation.x
        oy = msg.orientation.y
        oz = msg.orientation.z
        ow = msg.orientation.w

        self.writer.writerow([ax, ay, az, gx, gy, gz, ox, oy, oz, ow])
        self.file.flush()

        print(f"📥 IMU → ax:{ax:.3f}, ay:{ay:.3f}, az:{az:.3f}, gx:{gx:.3f}, gy:{gy:.3f}, gz:{gz:.3f}", flush=True)

    def destroy_node(self):
        super().destroy_node()
        self.file.close()
        self.get_logger().info(f"✅ Log saved to {self.filename}")

def main(args=None):
    rclpy.init(args=args)
    node = IMURawLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
