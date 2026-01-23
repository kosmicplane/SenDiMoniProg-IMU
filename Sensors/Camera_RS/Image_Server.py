#!/usr/bin/env python3
"""
RealSense ROS2 webserver (without pyrealsense2).

- Reads color and depth images from ROS2 topics:
    /d405/color/image_raw
    /d405/depth/image_rect_raw

- Optionally reads pointcloud from:
/d405/depth/color/points  (PointCloud2)

- Serves:
    /color      -> MJPEG stream of color image
    /depth      -> MJPEG stream of depth image (gray)
    /pointcloud -> 3D scatter snapshot (JPEG) built from PointCloud2
"""

import threading
import time
import io

import numpy as np
import cv2
from flask import Flask, Response

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

from matplotlib.figure import Figure


# ---------------- Flask app ----------------

app = Flask(__name__)


# ---------------- ROS2 Node ----------------

class RealSenseRosBridge(Node):
    """
    ROS2 node that subscribes to RealSense topics and stores the latest frames
    to be served via Flask.
    """

    def __init__(self):
        super().__init__('realsense_ros_web_bridge')

        self.bridge = CvBridge()

        # Latest data
        self.last_color = None         # np.ndarray BGR
        self.last_depth_gray = None    # np.ndarray GRAY
        self.last_pc_points = None     # np.ndarray Nx3

        self.lock = threading.Lock()

        # Parameters (could be made configurable)
        color_topic = '/camera/d405/color/image_rect_raw'
        depth_topic = '/camera/d405/depth/image_rect_raw'
        pc_topic    = '/d405/depth/color/points'

        # Subscribers
        self.create_subscription(
            Image, color_topic, self.color_callback, 10
        )
        self.create_subscription(
            Image, depth_topic, self.depth_callback, 10
        )
        self.create_subscription(
            PointCloud2, pc_topic, self.pointcloud_callback, 10
        )

        self.get_logger().info(
            f"RealSenseRosBridge subscribed to:\n"
            f"  Color : {color_topic}\n"
            f"  Depth : {depth_topic}\n"
            f"  PC    : {pc_topic}"
        )

    # ---------- Callbacks ----------

    def color_callback(self, msg: Image):
        """Store latest color image (BGR)."""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Error converting color image: {e}")
            return

        with self.lock:
            self.last_color = cv_img

    def depth_callback(self, msg: Image):
        """Store latest depth image that already arrived colorized (RGB8)."""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Error converting depth image: {e}")
            return

        with self.lock:
            self.last_depth_gray = cv_img  # ahora es BGR 'bonito'

    def pointcloud_callback(self, msg: PointCloud2):
        """
        Convert ROS2 PointCloud2 to Nx3 array (XYZ in meters).
        We will downsample for a lightweight 3D view.
        """
        try:
            pts = pointcloud2_to_xyz_array(msg)
        except Exception as e:
            self.get_logger().warn(f"Error parsing PointCloud2: {e}")
            return

        if pts is None or pts.size == 0:
            return

        # Subsample
        if pts.shape[0] > 0:
            pts = pts[::50, :]  # keep 1/50 points

        with self.lock:
            self.last_pc_points = pts


# ---------------- PointCloud2 helper ----------------

def pointcloud2_to_xyz_array(cloud: PointCloud2):
    """
    Convert a sensor_msgs/PointCloud2 into an Nx3 numpy array (XYZ).
    Assumes fields named 'x', 'y', 'z'.
    """
    if cloud.height == 0 or cloud.width == 0:
        return None

    # Based on sensor_msgs.point_cloud2.read_points, but inline to avoid extra deps
    dtype_list = []
    for field in cloud.fields:
        if field.name in ['x', 'y', 'z']:
            # Only handle float32 x,y,z
            assert field.datatype == 7  # FLOAT32
            dtype_list.append((field.name, np.float32))

    if not dtype_list:
        return None

    # Full dtype for all fields in the message
    # Each point is a row
    dtype_full = np.dtype(dtype_list)

    # Interpret raw buffer as array of points
    data = np.frombuffer(cloud.data, dtype=dtype_full)
    xyz = np.vstack([data['x'], data['y'], data['z']]).T
    return xyz


# ---------------- Global node + helper accessors ----------------

ros_node: RealSenseRosBridge = None  # global reference used by Flask


def get_latest_color():
    global ros_node
    if ros_node is None:
        return None
    with ros_node.lock:
        if ros_node.last_color is None:
            return None
        return ros_node.last_color.copy()


def get_latest_depth_gray():
    global ros_node
    if ros_node is None:
        return None
    with ros_node.lock:
        if ros_node.last_depth_gray is None:
            return None
        # Convert gray to BGR for consistent MJPEG (3-channel)
        gray = ros_node.last_depth_gray
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def get_latest_pc_points():
    global ros_node
    if ros_node is None:
        return None
    with ros_node.lock:
        if ros_node.last_pc_points is None:
            return None
        return ros_node.last_pc_points.copy()


# ---------------- MJPEG generator ----------------

def mjpeg_generator(get_image_fn, fps=10):
    """Generic MJPEG generator from an image getter."""
    interval = 1.0 / max(fps, 1.0)

    while True:
        start = time.time()
        frame = get_image_fn()
        if frame is None:
            time.sleep(0.05)
            continue

        ok, buffer = cv2.imencode('.jpg', frame)
        if not ok:
            continue

        frame_bytes = buffer.tobytes()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )

        elapsed = time.time() - start
        if elapsed < interval:
            time.sleep(interval - elapsed)


# ---------------- Flask endpoints ----------------

@app.route("/")
def index():
    return (
        "<h1>RealSense ROS2 Webserver</h1>"
        "<ul>"
        "<li><a href='/color'>Color stream</a> (MJPEG)</li>"
        "<li><a href='/depth'>Depth stream (MJPEG)</a></li>"
        "<li><a href='/pointcloud'>3D pointcloud snapshot</a> (JPEG)</li>"
        "</ul>"
    )


@app.route("/color")
def color_stream():
    return Response(
        mjpeg_generator(get_latest_color, fps=10),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/depth")
def depth_stream():
    return Response(
        mjpeg_generator(get_latest_depth_gray, fps=10),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/pointcloud")
def pointcloud_view():
    pts = get_latest_pc_points()
    if pts is None or pts.size == 0:
        # blank image
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        ok, buf = cv2.imencode(".jpg", img)
        return Response(buf.tobytes(), mimetype="image/jpeg")

    # Remove NaNs / inf
    mask = np.isfinite(pts).all(axis=1)
    pts = pts[mask]
    if pts.size == 0:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        ok, buf = cv2.imencode(".jpg", img)
        return Response(buf.tobytes(), mimetype="image/jpeg")

    # Optionally downsample (keep every Nth point)
    if pts.shape[0] > 20000:
        pts = pts[::20, :]

    xs = pts[:, 0]
    ys = pts[:, 1]
    zs = pts[:, 2]

    # Create a small 3D scatter figure
    fig = Figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(xs, ys, zs, s=1)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    # ---- LIMIT VIEW TO D405 WORKSPACE ----
    # D405 is 0.07–0.5 m in front of the camera
    ax.set_xlim(-0.2, 0.2)
    ax.set_ylim(-0.2, 0.2)
    ax.set_zlim(0.0, 0.5)

    ax.view_init(elev=30, azim=45)
    fig.tight_layout()

    buf = io.BytesIO()
    fig.savefig(buf, format="jpg")
    buf.seek(0)
    return Response(buf.read(), mimetype="image/jpeg")


# ---------------- Main: start ROS2 + Flask ----------------

def ros_spin_thread():
    """
    Thread to spin the ROS2 node while Flask runs in the main thread.
    """
    global ros_node
    rclpy.init(args=None)
    ros_node = RealSenseRosBridge()
    try:
        rclpy.spin(ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


def main():
    # Start ROS2 spin in a background thread
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    # Start Flask server
    app.run(host="100.70.28.30", port=8000, threaded=True)


if __name__ == "__main__":
    main()
