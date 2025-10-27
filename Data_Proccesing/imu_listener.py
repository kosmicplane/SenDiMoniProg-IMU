#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os
from datetime import datetime

class IMURawLogger(Node):
    def __init__(self):
        super().__init__('imu_raw_logger')

        # --- CSV setup ---
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"/root/data/imu_raw_{timestamp}.csv"
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        # --- CSV header ---
        self.writer.writerow([
            "ax_g", "ay_g", "az_g",
            "gx_dps", "gy_dps", "gz_dps",
            "mx_uT", "my_uT", "mz_uT",
            "pressure_hPa", "altitude_m", "tempC"
        ])
        self.file.flush()

        # --- ROS subscriber ---
        self.subscription = self.create_subscription(
            String,
            '/imu/raw_data',  # debe coincidir con el topic del publisher
            self.callback,
            10
        )

        self.get_logger().info(f"📡 Listening to /imu/raw_data — saving to {self.filename}")

    def callback(self, msg):
        """Cada mensaje es una línea CSV cruda enviada por el Jetson."""
        line = msg.data.strip()
        if not line:
            return

        # Dividir la línea por comas y escribir cada valor en su columna
        parts = [x.strip() for x in line.split(',')]
        if len(parts) == 12:
            self.writer.writerow(parts)
            self.file.flush()
            print(f"📥 Saved row: {parts}")
        else:
            self.get_logger().warn(f"Ignored malformed line ({len(parts)} fields): {line}")

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
