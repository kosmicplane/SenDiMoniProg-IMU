#!/usr/bin/env python3
"""ROS 2 node that subscribes to an IMU topic and exposes the data
over a WebSocket server.

This node is intended to run on the Jetson Nano side.

- Subscribes to: /imu/data (sensor_msgs/Imu)
- Serves: ws://100.91.177.83:8765 by default
- Sends each IMU message as a JSON string to all connected clients.
"""

import asyncio
import json
import threading
from typing import Set

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import websockets
except ImportError:
    websockets = None


class ImuWsServer(Node):
    """ROS 2 node that bridges IMU data to a WebSocket server."""

    def __init__(self) -> None:
        super().__init__('imu_ws_server')

        # Declare configurable parameters with default values
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('ws_host', '100.70.28.30')
        self.declare_parameter('ws_port', 8765)

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.ws_host = self.get_parameter('ws_host').get_parameter_value().string_value
        self.ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value

        if websockets is None:
            # If websockets is missing, fail fast with a clear log message
            self.get_logger().error(
                'Python package "websockets" is not installed.\n'
                'Please install it inside the container with:\n'
                '  pip3 install websockets'
            )
            raise RuntimeError('websockets package not available')

        # ROS 2 subscription to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10,  # QoS queue size
        )

        # Set of currently connected WebSocket clients
        self._clients: Set[websockets.WebSocketServerProtocol] = set()

        # Separate asyncio event loop for the WebSocket server
        self._loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(
            target=self._run_ws_server,
            daemon=True,
        )
        self._ws_thread.start()

        self.get_logger().info(
            f'IMU WebSocket server initialized.\n'
            f'  IMU topic : {imu_topic}\n'
            f'  WebSocket : ws://{self.ws_host}:{self.ws_port}'
        )

    # ------------------------------------------------------------------
    # ROS 2 callback: called every time a new Imu message is received
    # ------------------------------------------------------------------
    def imu_callback(self, msg: Imu) -> None:
        """Convert the Imu message to JSON and broadcast it."""
        # Build a simple JSON-serializable dict
        data = {
            "t": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id,
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }
        text = json.dumps(data)

        async def _broadcast(payload: str) -> None:
            """Asynchronous coroutine that sends the payload to all clients."""
            if not self._clients:
                return
            # Copy to avoid RuntimeError if the set changes while iterating
            clients = list(self._clients)
            for ws in clients:
                try:
                    await ws.send(payload)
                except Exception as exc:
                    self.get_logger().warn(
                        f'Error sending data to WebSocket client: {exc}'
                    )
                    self._clients.discard(ws)

        # Schedule the coroutine on the WebSocket event loop
        asyncio.run_coroutine_threadsafe(_broadcast(text), self._loop)

    # ------------------------------------------------------------------
    # WebSocket server loop (runs in a dedicated thread)
    # ------------------------------------------------------------------
    def _run_ws_server(self) -> None:
        """Set up and run the WebSocket server in its own asyncio loop."""
        asyncio.set_event_loop(self._loop)

        async def handler(websocket, path) -> None:
            """Handler for each connected WebSocket client.

            We do not expect to receive messages from the client side for now,
            so we simply keep the connection open and send IMU data from the
            ROS callback.
            """
            self._clients.add(websocket)
            self.get_logger().info('WebSocket client connected.')
            try:
                async for _ in websocket:
                    # Ignore incoming data from client for this simple bridge
                    pass
            finally:
                self.get_logger().info('WebSocket client disconnected.')
                self._clients.discard(websocket)

        # Create and start the WebSocket server
        start_server = websockets.serve(handler, self.ws_host, self.ws_port)
        self._loop.run_until_complete(start_server)
        self.get_logger().info(
            f'WebSocket server listening on ws://{self.ws_host}:{self.ws_port}'
        )
        self._loop.run_forever()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self):  # type: ignore[override]
        """Cleanly stop the WebSocket loop when shutting down."""
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
        node = ImuWsServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
