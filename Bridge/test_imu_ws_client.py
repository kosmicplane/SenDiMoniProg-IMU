#!/usr/bin/env python3
"""Simple test client for the IMU WebSocket server.

This script does NOT depend on ROS.
It only connects to the WebSocket server and prints incoming JSON
messages to the console.

Usage::

    python3 test_imu_ws_client.py ws://<jetson_ip>:8765

If no URL is provided, it defaults to ``ws://127.0.0.1:8765``.
"""

import asyncio
import sys

import websockets


async def main() -> None:
    """Connect to the WebSocket server and print incoming messages."""
    # Take WebSocket URL from command line argument or use default
    if len(sys.argv) > 1:
        ws_url = sys.argv[1]
    else:
        ws_url = "ws://127.0.0.1:8765"

    print(f"Connecting to {ws_url} ...")
    async with websockets.connect(ws_url) as ws:
        print("Connected. Waiting for IMU messages...\n")
        async for msg in ws:
            print(msg)


if __name__ == "__main__":
    asyncio.run(main())
