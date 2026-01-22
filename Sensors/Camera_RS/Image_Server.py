#!/usr/bin/env python3
import threading
import time

import numpy as np
import pyrealsense2 as rs
import cv2

from flask import Flask, Response, send_file
import io
from matplotlib.figure import Figure

app = Flask(__name__)

# ---------- RealSense setup ----------
pipeline = rs.pipeline()
config = rs.config()

# Ajusta resolución / fps según lo que aguante tu Jetson / red
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)
colorizer = rs.colorizer()
pc = rs.pointcloud()

# Para compartir últimos frames entre hilos
last_color = None
last_depth_color = None
last_pc_points = None

lock = threading.Lock()
running = True

def capture_loop():
    global last_color, last_depth_color, last_pc_points, running

    while running:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Color frame
        color_image = np.asanyarray(color_frame.get_data())

        # Depth colorizada (pseudo-color)
        depth_color_frame = colorizer.colorize(depth_frame)
        depth_color_image = np.asanyarray(depth_color_frame.get_data())

        # Nube de puntos básica
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        # Convert to numpy array (x,y,z), subsample to keep it light
        v = np.asanyarray(points.get_vertices())  # Nx1, each has .x .y .z
        pts = np.vstack([ [p.x, p.y, p.z] for p in v ]).reshape(-1, 3)
        pts = pts[::50, :]  # SUBSAMPLING: 1 de cada 50 puntos

        with lock:
            last_color = color_image
            last_depth_color = depth_color_image
            last_pc_points = pts

        # Evita saturar CPU si algo se cuelga
        time.sleep(0.001)

# Hilo de captura
capture_thread = threading.Thread(target=capture_loop, daemon=True)
capture_thread.start()

# ---------- Utilidades MJPEG ----------
def mjpeg_generator(get_image_fn, fps=15):
    interval = 1.0 / fps
    while True:
        start = time.time()
        with lock:
            frame = get_image_fn()
        if frame is None:
            time.sleep(0.01)
            continue

        # Codificar a JPEG
        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            continue
        frame_bytes = buffer.tobytes()

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")

        elapsed = time.time() - start
        if elapsed < interval:
            time.sleep(interval - elapsed)

# ---------- Rutas Flask ----------

@app.route("/")
def index():
    return (
        "<h1>RealSense Webserver</h1>"
        "<ul>"
        "<li><a href='/color'>Color stream</a></li>"
        "<li><a href='/depth'>Depth stream (colorized)</a></li>"
        "<li><a href='/pointcloud'>3D view (2D projection)</a></li>"
        "</ul>"
    )

@app.route("/color")
def color_stream():
    def get_color():
        return last_color
    return Response(
        mjpeg_generator(get_color, fps=15),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/depth")
def depth_stream():
    def get_depth():
        return last_depth_color
    return Response(
        mjpeg_generator(get_depth, fps=15),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/pointcloud")
def pointcloud_view():
    """
    Renderiza la nube de puntos actual como imagen 2D usando matplotlib.
    No es interactivo 3D (tipo WebGL), pero da una vista 3D estática
    actualizada cada vez que recargas.
    """
    with lock:
        pts = last_pc_points
    if pts is None or pts.size == 0:
        # Devuelve imagen negra si aún no hay datos
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        ok, buf = cv2.imencode(".jpg", img)
        return Response(buf.tobytes(), mimetype="image/jpeg")

    # Creamos una figura pequeña
    fig = Figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection="3d")

    xs = pts[:, 0]
    ys = pts[:, 1]
    zs = pts[:, 2]

    ax.scatter(xs, ys, zs, s=1)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.view_init(elev=30, azim=45)

    # Convertimos la figura a JPEG en memoria
    buf = io.BytesIO()
    fig.tight_layout()
    fig.savefig(buf, format="jpg")
    buf.seek(0)
    return Response(buf.read(), mimetype="image/jpeg")

# ---------- Main ----------
if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=8000, threaded=True)
    finally:
        running = False
        capture_thread.join(timeout=1.0)
        pipeline.stop()
