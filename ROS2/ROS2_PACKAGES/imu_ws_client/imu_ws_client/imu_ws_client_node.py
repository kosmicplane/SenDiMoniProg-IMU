#!/usr/bin/env python3
"""ROS 2 node that connects to the IMU WebSocket server and republishes
incoming IMU data as sensor_msgs/Imu.

This node is intended to run on the PC side.

- Connects to: ws://<jetson_ip>:8765 (configurable via parameter)
- Publishes: /imu/data (sensor_msgs/Imu)
"""

import asyncio
import json
import threading
from queue import Queue, Empty
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import websockets
except ImportError:
    websockets = None


class ImuWsClient(Node):
    """ROS 2 node that receives IMU JSON data from a WebSocket and republishes it."""

    def __init__(self) -> None:
        super().__init__('imu_ws_client')

        # Parameters:
        # - ws_url: URL of the WebSocket server on the Jetson
        # - imu_topic: ROS 2 topic to publish Imu messages on the PC
        self.declare_parameter('ws_url', 'ws://127.0.0.1:8765')
        self.declare_parameter('imu_topic', '/imu/data')

        self.ws_url = self.get_parameter('ws_url').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        if websockets is None:
            self.get_logger().error(
                'Python package "websockets" is not installed.\n'
                'Please install it inside the container with:\n'
                '  pip3 install websockets'
            )
            raise RuntimeError('websockets package not available')

        # Publisher for Imu messages on the PC
        self.publisher = self.create_publisher(Imu, imu_topic, 10)

        # Thread-safe queue to move data from the WebSocket thread to the ROS 2 thread
        self._queue: "Queue[dict]" = Queue()

        # Asyncio event loop dedicated to the WebSocket client
        self._loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(
            target=self._run_ws_client,
            daemon=True,
        )
        self._ws_thread.start()

        # Timer to periodically drain the queue and publish messages
        self._timer = self.create_timer(0.001, self._process_queue)

        self.get_logger().info(
            f'IMU WebSocket client initialized.\n'
            f'  WebSocket URL : {self.ws_url}\n'
            f'  Publish topic : {imu_topic}'
        )

    # ------------------------------------------------------------------
    # WebSocket client loop (runs in a dedicated thread)
    # ------------------------------------------------------------------
    def _run_ws_client(self) -> None:
        """Run the WebSocket client in its own asyncio event loop."""
        asyncio.set_event_loop(self._loop)

        async def _client_loop():
            """Main reconnecting loop for the WebSocket client."""
            while True:
                try:
                    self.get_logger().info(
                        f'Connecting to IMU WS server at {self.ws_url}...'
                    )
                    async with websockets.connect(self.ws_url) as ws:
                        self.get_logger().info('Connected to IMU WebSocket server.')
                        async for message in ws:
                            # Each message is expected to be a JSON string
                            try:
                                data = json.loads(message)
                                self._queue.put_nowait(data)
                            except json.JSONDecodeError as exc:
                                self.get_logger().warn(
                                    f'Failed to decode JSON from WS: {exc}'
                                )
                except Exception as exc:
                    # Log the error and try again after a short delay
                    self.get_logger().warn(
                        f'WebSocket connection error: {exc}. Retrying in 2 seconds...'
                    )
                    await asyncio.sleep(2.0)

        self._loop.run_until_complete(_client_loop())

    # ------------------------------------------------------------------
    # ROS 2 timer callback: publish messages from the queue
    # ------------------------------------------------------------------
    def _process_queue(self) -> None:
        """Drain the queue of received IMU dictionaries and publish them."""
        while True:
            try:
                data = self._queue.get_nowait()
            except Empty:
                break

            msg = self._convert_dict_to_imu(data)
            if msg is not None:
                self.publisher.publish(msg)

    # ------------------------------------------------------------------
    # Helper: convert JSON dict to Imu message
    # ------------------------------------------------------------------
    def _convert_dict_to_imu(self, data: dict) -> Optional[Imu]:
        """Convert the incoming dictionary into a sensor_msgs/Imu message.

        Expected schema::

            {
              "t": <float timestamp in seconds>,
              "frame_id": <string>,
              "orientation": {"x": ..., "y": ..., "z": ..., "w": ...},
              "angular_velocity": {"x": ..., "y": ..., "z": ...},
              "linear_acceleration": {"x": ..., "y": ..., "z": ...}
            }
        """
        try:
            msg = Imu()
            # Convert timestamp
            t = float(data.get("t", 0.0))
            msg.header.stamp.sec = int(t)
            msg.header.stamp.nanosec = int((t - int(t)) * 1e9)

            # Frame id
            msg.header.frame_id = str(data.get("frame_id", "imu_link"))

            # Orientation
            ori = data.get("orientation", {})
            msg.orientation.x = float(ori.get("x", 0.0))
            msg.orientation.y = float(ori.get("y", 0.0))
            msg.orientation.z = float(ori.get("z", 0.0))
            msg.orientation.w = float(ori.get("w", 1.0))

            # Angular velocity
            ang = data.get("angular_velocity", {})
            msg.angular_velocity.x = float(ang.get("x", 0.0))
            msg.angular_velocity.y = float(ang.get("y", 0.0))
            msg.angular_velocity.z = float(ang.get("z", 0.0))

            # Linear acceleration
            lin = data.get("linear_acceleration", {})
            msg.linear_acceleration.x = float(lin.get("x", 0.0))
            msg.linear_acceleration.y = float(lin.get("y", 0.0))
            msg.linear_acceleration.z = float(lin.get("z", 0.0))

            return msg
        except Exception as exc:
            self.get_logger().warn(f'Error converting dict to Imu: {exc}')
            return None

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self):  # type: ignore[override]
        """Stop the WebSocket loop on shutdown."""
        try:
            self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    """Entry point for the ROS 2 node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ImuWsClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
