#!/usr/bin/env python3
"""
Simple RealSense webserver for:
  - Color stream  (MJPEG)  at /color
  - Depth stream  (MJPEG)  at /depth
  - 3D pointcloud snapshot (JPEG) at /pointcloud

Designed to run on the Jetson (inside Docker), and be viewed
remotely from a PC via Tailscale using a web browser.

Example URLs from the PC:
  http://<JETSON_TAILSCALE_IP>:8000/color
  http://<JETSON_TAILSCALE_IP>:8000/depth
  http://<JETSON_TAILSCALE_IP>:8000/pointcloud
"""

import threading
import time
import io

import numpy as np
import cv2
from flask import Flask, Response

# If pyrealsense2 is missing, this will raise ImportError
import pyrealsense2 as rs

from matplotlib.figure import Figure

# ---------------- Flask app ----------------

app = Flask(__name__)

# ---------------- RealSense global state ----------------

pipeline = None
align = None
colorizer = None
pc = None

last_color = None          # np.ndarray (H, W, 3), BGR
last_depth_color = None    # np.ndarray (H, W, 3), BGR pseudo-color
last_pc_points = None      # np.ndarray (N, 3), XYZ in meters

lock = threading.Lock()
running = True


def init_realsense():
    """
    Initialize the RealSense pipeline, enable color + depth,
    alignment and pointcloud.
    """
    global pipeline, align, colorizer, pc

    pipeline = rs.pipeline()
    config = rs.config()

    # Adjust resolution and fps depending on Jetson performance
    # D405 works well at 640x480, 15 FPS for many applications.
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

    pipeline.start(config)

    # Align depth to color so both share the same frame of reference
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Colorizer for depth visualization
    colorizer = rs.colorizer()

    # Pointcloud helper
    pc = rs.pointcloud()


def capture_loop():
    """
    Background thread: continuously grab frames from RealSense and
    update global buffers for color, depth and pointcloud.
    """
    global last_color, last_depth_color, last_pc_points, running
    global pipeline, align, colorizer, pc

    while running:
        try:
            frames = pipeline.wait_for_frames()
        except Exception:
            # If something goes wrong temporarily, sleep a bit and retry
            time.sleep(0.01)
            continue

        # Align depth to color
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            time.sleep(0.001)
            continue

        # Convert color frame to numpy BGR image
        color_image = np.asanyarray(color_frame.get_data())

        # Colorize depth (pseudo-color, RGB)
        depth_color_frame = colorizer.colorize(depth_frame)
        depth_color_image = np.asanyarray(depth_color_frame.get_data())

        # Compute pointcloud
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)

        # Convert vertices to Nx3 float32 array
        v = np.asanyarray(points.get_vertices())  # Nx1 array of rs.vertex
        pts = np.vstack([[p.x, p.y, p.z] for p in v]).reshape(-1, 3)

        # Subsample to keep the 3D view lightweight
        if pts.shape[0] > 0:
            pts = pts[::50, :]  # keep 1 of every 50 points

        with lock:
            last_color = color_image
            last_depth_color = depth_color_image
            last_pc_points = pts

        # Tiny delay to avoid pegging CPU in case wait_for_frames returns very fast
        time.sleep(0.001)


def mjpeg_generator(get_image_fn, fps=15):
    """
    Generic MJPEG generator. `get_image_fn` must return a BGR uint8 image
    (H, W, 3) or None if no frame is available yet.
    """
    interval = 1.0 / max(fps, 1.0)

    while True:
        start = time.time()
        frame = get_image_fn()
        if frame is None:
            time.sleep(0.01)
            continue

        # Encode to JPEG
        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            continue
        frame_bytes = buffer.tobytes()

        # MJPEG chunk
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )

        # Simple FPS limiting
        elapsed = time.time() - start
        if elapsed < interval:
            time.sleep(interval - elapsed)


# ---------------- Flask endpoints ----------------

@app.route("/")
def index():
    """Simple index page with links to available streams."""
    return (
        "<h1>RealSense Webserver</h1>"
        "<ul>"
        "<li><a href='/color'>Color stream</a> (MJPEG)</li>"
        "<li><a href='/depth'>Depth stream (colorized)</a> (MJPEG)</li>"
        "<li><a href='/pointcloud'>3D pointcloud snapshot</a> (JPEG)</li>"
        "</ul>"
    )


@app.route("/color")
def color_stream():
    """Color MJPEG stream endpoint."""
    def get_color():
        with lock:
            if last_color is None:
                return None
            # Copy to avoid issues if the capture thread overwrites it
            return last_color.copy()

    return Response(
        mjpeg_generator(get_color, fps=15),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/depth")
def depth_stream():
    """Depth (colorized) MJPEG stream endpoint."""
    def get_depth():
        with lock:
            if last_depth_color is None:
                return None
            return last_depth_color.copy()

    return Response(
        mjpeg_generator(get_depth, fps=15),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/pointcloud")
def pointcloud_view():
    """
    Generate a static 3D view of the current pointcloud using matplotlib
    and return it as a JPEG image.

    Note: this is NOT interactive WebGL, but a simple 3D snapshot
    that updates every time you reload the page.
    """
    with lock:
        pts = None if last_pc_points is None else last_pc_points.copy()

    # If we don't have points yet, return a black image
    if pts is None or pts.size == 0:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        ok, buf = cv2.imencode(".jpg", img)
        return Response(buf.tobytes(), mimetype="image/jpeg")

    # Create a small figure for 3D scatter
    fig = Figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection="3d")

    xs = pts[:, 0]
    ys = pts[:, 1]
    zs = pts[:, 2]

    # Small point size to keep it readable
    ax.scatter(xs, ys, zs, s=1)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.view_init(elev=30, azim=45)

    # Tight layout and render to memory
    buf = io.BytesIO()
    fig.tight_layout()
    fig.savefig(buf, format="jpg")
    buf.seek(0)
    return Response(buf.read(), mimetype="image/jpeg")


# ---------------- Main entrypoint ----------------

def main():
    global running

    # Initialize RealSense pipeline
    init_realsense()

    # Start capture thread
    capture_thread = threading.Thread(target=capture_loop, daemon=True)
    capture_thread.start()

    try:
        # Run Flask in threaded mode so multiple clients can connect
        # Use host="0.0.0.0" so it is visible from outside the container
        app.run(host="0.0.0.0", port=8000, threaded=True)
    finally:
        # Graceful shutdown
        running = False
        capture_thread.join(timeout=1.0)

        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
