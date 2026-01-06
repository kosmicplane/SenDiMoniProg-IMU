#!/usr/bin/env python3
import time
import os
from collections import deque
from datetime import datetime
import threading
import queue
import json

import numpy as np
import cv2

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd

import paho.mqtt.client as mqtt

# ----------------------------
# MQTT configuration
# ----------------------------
BROKER_HOST = "test.mosquitto.org"
BROKER_PORT = 1883
MQTT_USER = ""      # optional
MQTT_PASS = ""      # optional

TOPIC_RAW   = "imu/jetson01/raw"           # IMU: CSV 12 fields
TOPIC_COLOR = "cam/jetson01/color_jpg"     # Camera: JPEG bytes
TOPIC_DEPTH = "cam/jetson01/depth_png16"   # ✅ Metric depth: uint16 PNG (Z16 units)
TOPIC_META  = "cam/jetson01/meta"          # ✅ JSON meta: contains depth_scale + intrinsics

SHOW_DEPTH = True  # set False if you don't publish depth

# ----------------------------
# UI / logging configuration
# ----------------------------
PLOT_HZ = 20.0
WINDOW_SEC = 10.0

# IMU logging rate to DataFrame to avoid huge files.
# - Set STORE_HZ = 0 to store EVERY received IMU sample (can be large).
STORE_HZ = 50.0
STORE_PERIOD = (1.0 / STORE_HZ) if STORE_HZ > 0 else 0.0

# Camera decode/display control (helps reduce lag)
CAM_FPS = 30                    # decode/display at most this rate
CAM_PERIOD = 1.0 / CAM_FPS

# ----------------------------
# Tracking / depth points config
# ----------------------------
TRACK_MAX_POINTS = 40           # max tracked features
TRACK_REDETECT_SEC = 1.5        # re-detect features periodically
TRACK_MIN_DIST_PX = 15          # feature spacing
TRACK_WIN_SIZE = (21, 21)
TRACK_MAX_LEVEL = 3

# Depth visualization (for the depth panel only)
DEPTH_CLIP_M = 6.0              # visualize 0..DEPTH_CLIP_M meters

# Expected CSV layout: 12 values per line
FIELDS = [
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "mx_uT", "my_uT", "mz_uT",
    "p_hpa", "t_C", "alt_m"
]


def parse_csv12(line: str):
    """Parse one CSV line with 12 float fields. Return dict or None if invalid."""
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 12:
        return None
    try:
        vals = list(map(float, parts))
        return dict(zip(FIELDS, vals))
    except ValueError:
        return None


def make_mqtt_client(imu_queue: "queue.Queue[str]", frame_lock: threading.Lock, frames: dict):
    """
    Create an MQTT client:
      - pushes IMU CSV payloads into imu_queue
      - stores ONLY latest bytes for color/depth/meta in `frames`
        (decode + processing happen in the UI loop to avoid backlog/lag).
    """
    client_id = f"pc-plot-{int(time.time())}"
    c = mqtt.Client(client_id=client_id, clean_session=True, protocol=mqtt.MQTTv311)

    if MQTT_USER:
        c.username_pw_set(MQTT_USER, MQTT_PASS)

    c.reconnect_delay_set(min_delay=1, max_delay=30)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT connected", flush=True)
            client.subscribe(TOPIC_RAW, qos=0)
            client.subscribe(TOPIC_COLOR, qos=0)
            client.subscribe(TOPIC_META, qos=0)
            if SHOW_DEPTH:
                client.subscribe(TOPIC_DEPTH, qos=0)

            print(f"✅ Subscribed IMU : {TOPIC_RAW}", flush=True)
            print(f"✅ Subscribed CAM : {TOPIC_COLOR}", flush=True)
            print(f"✅ Subscribed META: {TOPIC_META}", flush=True)
            if SHOW_DEPTH:
                print(f"✅ Subscribed DEPTH: {TOPIC_DEPTH}", flush=True)
        else:
            print(f"⚠️  MQTT connect failed rc={rc}", flush=True)

    def on_disconnect(client, userdata, rc):
        print(f"⚠️  MQTT disconnected rc={rc} (will retry)", flush=True)

    def on_message(client, userdata, msg):
        try:
            # IMU: CSV text
            if msg.topic == TOPIC_RAW:
                payload = msg.payload.decode("utf-8", errors="ignore").strip()
                if payload:
                    # Keep queue bounded (real-time view prefers latest)
                    if imu_queue.qsize() > 200:
                        try:
                            while imu_queue.qsize() > 50:
                                imu_queue.get_nowait()
                        except queue.Empty:
                            pass
                    imu_queue.put_nowait(payload)
                return

            # META: JSON text
            if msg.topic == TOPIC_META:
                payload = msg.payload.decode("utf-8", errors="ignore").strip()
                if payload:
                    with frame_lock:
                        frames["meta_json"] = payload
                        frames["meta_ts"] = time.monotonic()
                return

            # COLOR: store latest bytes ONLY (decode in UI loop)
            if msg.topic == TOPIC_COLOR:
                with frame_lock:
                    frames["color_jpg"] = msg.payload
                    frames["color_ts"] = time.monotonic()
                return

            # DEPTH: store latest bytes ONLY (decode in UI loop)
            if SHOW_DEPTH and msg.topic == TOPIC_DEPTH:
                with frame_lock:
                    frames["depth_png16"] = msg.payload
                    frames["depth_ts"] = time.monotonic()
                return

        except Exception:
            pass

    c.on_connect = on_connect
    c.on_disconnect = on_disconnect
    c.on_message = on_message
    return c


def decode_jpg_to_rgb(jpg_bytes: bytes):
    """Decode JPEG bytes to RGB numpy array, or None."""
    if not jpg_bytes:
        return None
    arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def decode_png16_to_u16(png_bytes: bytes):
    """Decode PNG16 bytes to uint16 (HxW), or None."""
    if not png_bytes:
        return None
    arr = np.frombuffer(png_bytes, dtype=np.uint8)
    u16 = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if u16 is None:
        return None
    if u16.dtype != np.uint16:
        return None
    return u16


def depth_u16_to_vis_rgb(depth_u16: np.ndarray, depth_scale: float):
    """
    Convert uint16 depth to a nice RGB visualization.
    depth_u16: raw Z16 units
    depth_scale: meters per unit
    """
    if depth_u16 is None or depth_scale is None:
        return None

    depth_m = depth_u16.astype(np.float32) * float(depth_scale)
    # Normalize for visualization: 0..DEPTH_CLIP_M -> 0..255
    d = np.clip(depth_m, 0.0, float(DEPTH_CLIP_M))
    vis8 = (255.0 * (d / float(DEPTH_CLIP_M))).astype(np.uint8)
    vis8 = cv2.applyColorMap(vis8, cv2.COLORMAP_TURBO)  # BGR
    rgb = cv2.cvtColor(vis8, cv2.COLOR_BGR2RGB)
    return rgb


def sample_depth_m(depth_u16: np.ndarray, depth_scale: float, x: int, y: int):
    """
    Return metric depth (meters) at (x,y). If invalid (0), search a small neighborhood.
    """
    if depth_u16 is None or depth_scale is None:
        return None
    h, w = depth_u16.shape[:2]
    if x < 0 or y < 0 or x >= w or y >= h:
        return None

    v = int(depth_u16[y, x])
    if v > 0:
        return float(v) * float(depth_scale)

    # Small neighborhood search (avoid zero holes)
    r = 2
    x0, x1 = max(0, x - r), min(w, x + r + 1)
    y0, y1 = max(0, y - r), min(h, y + r + 1)
    patch = depth_u16[y0:y1, x0:x1]
    nz = patch[patch > 0]
    if nz.size == 0:
        return None
    return float(np.median(nz)) * float(depth_scale)


def main():
    # Thread-safe queue for IMU MQTT -> UI
    imu_queue: "queue.Queue[str]" = queue.Queue()

    # Latest frames/meta shared between MQTT callback and Matplotlib update()
    frame_lock = threading.Lock()
    frames = {
        "color_jpg": None,
        "depth_png16": None,
        "meta_json": None,

        "color_ts": 0.0,
        "depth_ts": 0.0,
        "meta_ts": 0.0,
    }

    mqttc = make_mqtt_client(imu_queue, frame_lock, frames)

    # Connect MQTT (retry loop)
    while True:
        try:
            mqttc.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
            break
        except Exception as e:
            print(f"⚠️  MQTT retry: {e}", flush=True)
            time.sleep(2)

    mqttc.loop_start()

    # Latest parsed IMU sample
    last_sample = None

    # Circular buffers for plotting (sliding window)
    maxlen = int(WINDOW_SEC * PLOT_HZ) + 10
    t0 = time.monotonic()
    t = deque(maxlen=maxlen)

    acc_x = deque(maxlen=maxlen); acc_y = deque(maxlen=maxlen); acc_z = deque(maxlen=maxlen)
    gyr_x = deque(maxlen=maxlen); gyr_y = deque(maxlen=maxlen); gyr_z = deque(maxlen=maxlen)
    mag_x = deque(maxlen=maxlen); mag_y = deque(maxlen=maxlen); mag_z = deque(maxlen=maxlen)

    # IMU logging storage
    records = []
    last_store_t = time.monotonic()

    # Meta state (from TOPIC_META)
    meta = {
        "depth_scale_m_per_unit": None,
        "fx": None, "fy": None, "ppx": None, "ppy": None,
        "seq": None, "pub_hz": None,
        "w": None, "h": None
    }
    last_meta_ts_seen = 0.0

    # ---- Figure layout: text + images + 3 plots ----
    fig = plt.figure()
    gs = fig.add_gridspec(5, 1, height_ratios=[1.2, 3.0, 2.0, 2.0, 2.0])

    ax_text = fig.add_subplot(gs[0])
    ax_text.axis("off")
    text_artist = ax_text.text(
        0.01, 0.5, "Waiting for MQTT data...",
        va="center", ha="left", fontsize=12, family="monospace"
    )

    # Images row
    if SHOW_DEPTH:
        gs_img = gs[1].subgridspec(1, 2, wspace=0.02)
        ax_img_color = fig.add_subplot(gs_img[0, 0])
        ax_img_depth = fig.add_subplot(gs_img[0, 1])

        ax_img_depth.axis("off")
        ax_img_depth.set_title("Depth (waiting...)")
        depth_artist = ax_img_depth.imshow(np.zeros((240, 320, 3), dtype=np.uint8))

        depth_info_text = ax_img_depth.text(
            0.02, 0.98, "",
            transform=ax_img_depth.transAxes,
            va="top", ha="left",
            fontsize=9, family="monospace",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="black", alpha=0.5)
        )
    else:
        ax_img_color = fig.add_subplot(gs[1])
        ax_img_depth = None
        depth_artist = None
        depth_info_text = None

    ax_img_color.axis("off")
    ax_img_color.set_title("Color (waiting...)")
    color_artist = ax_img_color.imshow(np.zeros((240, 320, 3), dtype=np.uint8))

    # IMU plots
    ax_acc = fig.add_subplot(gs[2])
    ax_gyr = fig.add_subplot(gs[3], sharex=ax_acc)
    ax_mag = fig.add_subplot(gs[4], sharex=ax_acc)

    # ACC plot
    acc_l1, = ax_acc.plot([], [], label="ax_g")
    acc_l2, = ax_acc.plot([], [], label="ay_g")
    acc_l3, = ax_acc.plot([], [], label="az_g")
    ax_acc.set_ylabel("acc [g]")
    ax_acc.grid(True)
    ax_acc.legend(loc="upper right")

    # GYR plot
    gyr_l1, = ax_gyr.plot([], [], label="gx_dps")
    gyr_l2, = ax_gyr.plot([], [], label="gy_dps")
    gyr_l3, = ax_gyr.plot([], [], label="gz_dps")
    ax_gyr.set_ylabel("gyro [dps]")
    ax_gyr.grid(True)
    ax_gyr.legend(loc="upper right")

    # MAG plot
    mag_l1, = ax_mag.plot([], [], label="mx_uT")
    mag_l2, = ax_mag.plot([], [], label="my_uT")
    mag_l3, = ax_mag.plot([], [], label="mz_uT")
    ax_mag.set_ylabel("mag [uT]")
    ax_mag.set_xlabel("time [s]")
    ax_mag.grid(True)
    ax_mag.legend(loc="upper right")

    fig.suptitle("Jetson IMU + RealSense via MQTT — Live Plot + Tracking + CSV logger", fontsize=14)

    def maybe_store(sample, now_mono):
        """Store IMU samples at STORE_HZ to avoid file bloat. STORE_HZ==0 stores all."""
        nonlocal last_store_t

        if STORE_HZ <= 0:
            records.append({
                "t_wall": datetime.now().isoformat(timespec="milliseconds"),
                "t_mono": now_mono - t0,
                **sample
            })
            return

        if (now_mono - last_store_t) >= STORE_PERIOD:
            last_store_t = now_mono
            records.append({
                "t_wall": datetime.now().isoformat(timespec="milliseconds"),
                "t_mono": now_mono - t0,
                **sample
            })

    def poll_mqtt_imu():
        """
        Drain all queued IMU MQTT messages and keep only the newest valid sample.
        This prevents lag/backlog and keeps the UI real-time.
        """
        nonlocal last_sample
        newest = None
        while True:
            try:
                payload = imu_queue.get_nowait()
            except queue.Empty:
                break
            newest = payload

        if newest is None:
            return

        sample = parse_csv12(newest)
        if sample is not None:
            last_sample = sample
            maybe_store(sample, time.monotonic())

    # Camera decode throttling state
    last_cam_draw = 0.0

    cached_color_rgb = None
    cached_color_ts = 0.0

    cached_depth_u16 = None
    cached_depth_vis_rgb = None
    cached_depth_ts = 0.0

    # Tracking state (we track using COLOR grayscale, then sample depth at those pixel coords)
    prev_gray = None
    points = None          # Nx1x2 float32
    last_detect_t = 0.0

    # Manual points: click on depth panel to add points
    manual_points = []     # list of (x, y)
    manual_mode = True     # if True: track user-picked points (if any), else auto-features

    def on_mouse(event):
        nonlocal manual_points
        if not SHOW_DEPTH:
            return
        if event.inaxes != ax_img_depth:
            return
        if event.xdata is None or event.ydata is None:
            return

        x = int(round(event.xdata))
        y = int(round(event.ydata))

        # Left click: add a point
        if event.button == 1:
            manual_points.append((x, y))
            if len(manual_points) > TRACK_MAX_POINTS:
                manual_points = manual_points[-TRACK_MAX_POINTS:]
        # Right click: clear
        elif event.button == 3:
            manual_points = []

    fig.canvas.mpl_connect("button_press_event", on_mouse)

    def detect_features(gray: np.ndarray):
        """Detect strong corners to track."""
        if gray is None:
            return None
        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=TRACK_MAX_POINTS,
            qualityLevel=0.01,
            minDistance=TRACK_MIN_DIST_PX,
            blockSize=7
        )
        return pts

    def make_points_from_manual():
        """Convert manual_points list into LK format Nx1x2 float32."""
        if not manual_points:
            return None
        arr = np.array(manual_points, dtype=np.float32).reshape(-1, 1, 2)
        return arr

    def update_tracking(gray: np.ndarray, depth_u16: np.ndarray, depth_scale: float):
        """
        Update tracking points using LK optical flow.
        Returns:
          - points_xy: list[(x,y)]
          - distances_m: list[float|None]
        """
        nonlocal prev_gray, points, last_detect_t

        if gray is None:
            prev_gray = None
            points = None
            return [], []

        now_t = time.monotonic()

        # Choose source of points
        desired = make_points_from_manual() if (manual_mode and manual_points) else None

        # If manual points exist, we prioritize them (and track them)
        if desired is not None:
            # Reset flow if different count or prev not set
            if prev_gray is None or points is None or len(desired) != len(points):
                points = desired
                prev_gray = gray.copy()
            else:
                nxt, st, _err = cv2.calcOpticalFlowPyrLK(
                    prev_gray, gray, points, None,
                    winSize=TRACK_WIN_SIZE,
                    maxLevel=TRACK_MAX_LEVEL,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
                )
                if nxt is not None and st is not None:
                    good = st.reshape(-1) == 1
                    points = nxt[good].reshape(-1, 1, 2).astype(np.float32)
                    # also update manual_points to follow
                    manual_points[:] = [(int(p[0][0]), int(p[0][1])) for p in points]
                prev_gray = gray.copy()

        else:
            # Auto-detect / auto-track features
            if points is None or prev_gray is None or (now_t - last_detect_t) >= TRACK_REDETECT_SEC:
                points = detect_features(gray)
                prev_gray = gray.copy()
                last_detect_t = now_t
            else:
                nxt, st, _err = cv2.calcOpticalFlowPyrLK(
                    prev_gray, gray, points, None,
                    winSize=TRACK_WIN_SIZE,
                    maxLevel=TRACK_MAX_LEVEL,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
                )
                if nxt is not None and st is not None:
                    good = st.reshape(-1) == 1
                    points = nxt[good].reshape(-1, 1, 2).astype(np.float32)
                prev_gray = gray.copy()

        if points is None or len(points) == 0:
            return [], []

        # Sample metric depth at each point
        pts_xy = []
        dists = []
        for p in points:
            x = int(round(float(p[0][0])))
            y = int(round(float(p[0][1])))
            pts_xy.append((x, y))
            d = sample_depth_m(depth_u16, depth_scale, x, y)
            dists.append(d)

        return pts_xy, dists

    def update_meta_if_new():
        """Parse meta JSON if updated, store depth_scale + intrinsics."""
        nonlocal last_meta_ts_seen
        with frame_lock:
            meta_json = frames["meta_json"]
            meta_ts = frames["meta_ts"]

        if meta_json and meta_ts != last_meta_ts_seen:
            last_meta_ts_seen = meta_ts
            try:
                m = json.loads(meta_json)
                meta["seq"] = m.get("seq", meta["seq"])
                meta["pub_hz"] = m.get("pub_hz", meta["pub_hz"])
                meta["w"] = m.get("w", meta["w"])
                meta["h"] = m.get("h", meta["h"])
                meta["depth_scale_m_per_unit"] = m.get("depth_scale_m_per_unit", meta["depth_scale_m_per_unit"])

                meta["fx"] = m.get("fx", meta["fx"])
                meta["fy"] = m.get("fy", meta["fy"])
                meta["ppx"] = m.get("ppx", meta["ppx"])
                meta["ppy"] = m.get("ppy", meta["ppy"])
            except Exception:
                pass

    def update(_frame):
        """Matplotlib animation callback: update camera + tracking + IMU plots."""
        nonlocal last_sample, last_cam_draw
        nonlocal cached_color_rgb, cached_color_ts
        nonlocal cached_depth_u16, cached_depth_vis_rgb, cached_depth_ts

        # --- IMU ---
        poll_mqtt_imu()

        # --- META ---
        update_meta_if_new()
        depth_scale = meta.get("depth_scale_m_per_unit", None)

        # --- Camera (decode only latest, at CAM_FPS) ---
        now_m = time.monotonic()
        new_color = False
        new_depth = False

        if (now_m - last_cam_draw) >= CAM_PERIOD:
            last_cam_draw = now_m

            with frame_lock:
                color_jpg = frames["color_jpg"]
                depth_png16 = frames["depth_png16"] if SHOW_DEPTH else None
                color_ts = frames["color_ts"]
                depth_ts = frames["depth_ts"]

            # Decode only if there is something new
            if color_jpg is not None and color_ts != cached_color_ts:
                rgb = decode_jpg_to_rgb(color_jpg)
                if rgb is not None:
                    cached_color_rgb = rgb
                    cached_color_ts = color_ts
                    new_color = True

            if SHOW_DEPTH and depth_png16 is not None and depth_ts != cached_depth_ts:
                u16 = decode_png16_to_u16(depth_png16)
                if u16 is not None:
                    cached_depth_u16 = u16
                    cached_depth_ts = depth_ts
                    # Depth visualization depends on depth_scale
                    cached_depth_vis_rgb = depth_u16_to_vis_rgb(cached_depth_u16, depth_scale) if depth_scale else None
                    new_depth = True

        # --- Tracking (only meaningful when we have a fresh-ish color frame) ---
        tracked_pts = []
        tracked_dists = []
        if cached_color_rgb is not None:
            gray = cv2.cvtColor(cached_color_rgb, cv2.COLOR_RGB2GRAY)
            # We can still update tracking even if not "new_color", but keeping it light is good
            if new_color or prev_gray is None:
                tracked_pts, tracked_dists = update_tracking(gray, cached_depth_u16, depth_scale)
            else:
                # Keep last points; still resample depth for display if we have it
                if points is not None and len(points) > 0:
                    tracked_pts = [(int(p[0][0]), int(p[0][1])) for p in points]
                    tracked_dists = [sample_depth_m(cached_depth_u16, depth_scale, x, y) for (x, y) in tracked_pts]

        # --- Update COLOR panel ---
        if cached_color_rgb is not None:
            color_artist.set_data(cached_color_rgb)
            age = (time.monotonic() - cached_color_ts) if cached_color_ts > 0 else 0.0
            ax_img_color.set_title(f"Color  | age {age:.2f}s  | cam_fps {CAM_FPS:.1f}")
        else:
            ax_img_color.set_title("Color (waiting...)")

        # --- Update DEPTH panel + overlay features ---
        if SHOW_DEPTH and depth_artist is not None:
            if cached_depth_vis_rgb is not None:
                # Draw overlay points on a copy for display
                vis = cached_depth_vis_rgb.copy()

                for i, (x, y) in enumerate(tracked_pts[:TRACK_MAX_POINTS], start=1):
                    cv2.circle(vis, (x, y), 3, (255, 255, 255), -1)   # white dot (RGB)
                    cv2.putText(vis, str(i), (x + 4, y - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

                depth_artist.set_data(vis)

                age = (time.monotonic() - cached_depth_ts) if cached_depth_ts > 0 else 0.0
                ax_img_depth.set_title(f"Depth  | age {age:.2f}s  | points {len(tracked_pts)}")

                # Build the distance list
                lines = []
                lines.append("Tracked points (meters):")
                if depth_scale is None:
                    lines.append("depth_scale: (missing meta)")
                else:
                    lines.append(f"depth_scale: {depth_scale:.6f} m/unit")

                if tracked_pts:
                    for i, ((x, y), d) in enumerate(zip(tracked_pts[:12], tracked_dists[:12]), start=1):
                        if d is None:
                            lines.append(f"{i:02d} @({x:3d},{y:3d}) :  --")
                        else:
                            lines.append(f"{i:02d} @({x:3d},{y:3d}) : {d:5.2f} m")
                else:
                    lines.append("(no points)  left-click to add; right-click to clear")

                if depth_info_text is not None:
                    depth_info_text.set_text("\n".join(lines))

            else:
                ax_img_depth.set_title("Depth (waiting meta/depth...)")
                if depth_info_text is not None:
                    depth_info_text.set_text("Waiting for depth_png16 + meta...")
        # --- Dashboard + IMU plots ---
        if last_sample is None:
            text_artist.set_text(
                f"Waiting for MQTT IMU data...\n"
                f"Broker: {BROKER_HOST}:{BROKER_PORT}\n"
                f"IMU : {TOPIC_RAW}\n"
                f"CAM : {TOPIC_COLOR}\n"
                f"DEP : {TOPIC_DEPTH}\n"
                f"META: {TOPIC_META}"
            )
            artists = [text_artist, color_artist, acc_l1, acc_l2, acc_l3, gyr_l1, gyr_l2, gyr_l3, mag_l1, mag_l2, mag_l3]
            if SHOW_DEPTH and depth_artist is not None:
                artists.append(depth_artist)
            if depth_info_text is not None:
                artists.append(depth_info_text)
            return tuple(artists)

        now = time.monotonic() - t0
        s = last_sample

        # Append to plot buffers (at UI rate)
        t.append(now)
        acc_x.append(s["ax_g"]);   acc_y.append(s["ay_g"]);   acc_z.append(s["az_g"])
        gyr_x.append(s["gx_dps"]); gyr_y.append(s["gy_dps"]); gyr_z.append(s["gz_dps"])
        mag_x.append(s["mx_uT"]);  mag_y.append(s["my_uT"]);  mag_z.append(s["mz_uT"])

        # Add a short “tracking status” line to the dashboard
        if tracked_pts and tracked_dists:
            # show first point distance as quick indicator
            d0 = tracked_dists[0]
            d0s = ("--" if d0 is None else f"{d0:.2f}m")
            track_line = f"TRACK: {len(tracked_pts)} pts | P1={d0s} | (click depth to add / right-click clear)"
        else:
            track_line = "TRACK: 0 pts | (click depth to add / right-click clear)"

        dash = (
            f"ACC [g]   ax={s['ax_g']:+8.3f}  ay={s['ay_g']:+8.3f}  az={s['az_g']:+8.3f}\n"
            f"GYR [dps] gx={s['gx_dps']:+8.3f}  gy={s['gy_dps']:+8.3f}  gz={s['gz_dps']:+8.3f}\n"
            f"MAG [uT]  mx={s['mx_uT']:+9.3f}  my={s['my_uT']:+9.3f}  mz={s['mz_uT']:+9.3f}\n"
            f"P/T/Alt   P={s['p_hpa']:9.2f} hPa   T={s['t_C']:6.2f} °C   Alt={s['alt_m']:8.2f} m\n"
            f"{track_line}\n"
            f"UI: {PLOT_HZ:.1f}Hz | CAM: {CAM_FPS:.1f}fps | Window: {WINDOW_SEC:.1f}s | Log: "
            f"{('ALL' if STORE_HZ <= 0 else f'{STORE_HZ:.1f}Hz')} | Samples saved: {len(records)}"
        )
        text_artist.set_text(dash)

        # Update line data
        acc_l1.set_data(t, acc_x); acc_l2.set_data(t, acc_y); acc_l3.set_data(t, acc_z)
        gyr_l1.set_data(t, gyr_x); gyr_l2.set_data(t, gyr_y); gyr_l3.set_data(t, gyr_z)
        mag_l1.set_data(t, mag_x); mag_l2.set_data(t, mag_y); mag_l3.set_data(t, mag_z)

        # Sliding X window
        if len(t) > 2:
            ax_acc.set_xlim(max(0.0, t[-1] - WINDOW_SEC), t[-1])

        # Auto-scale Y
        ax_acc.relim(); ax_acc.autoscale_view(scalex=False, scaley=True)
        ax_gyr.relim(); ax_gyr.autoscale_view(scalex=False, scaley=True)
        ax_mag.relim(); ax_mag.autoscale_view(scalex=False, scaley=True)

        artists = [text_artist, color_artist, acc_l1, acc_l2, acc_l3, gyr_l1, gyr_l2, gyr_l3, mag_l1, mag_l2, mag_l3]
        if SHOW_DEPTH and depth_artist is not None:
            artists.append(depth_artist)
        if depth_info_text is not None:
            artists.append(depth_info_text)
        return tuple(artists)

    interval_ms = int(1000.0 / PLOT_HZ)
    _ani = FuncAnimation(fig, update, interval=interval_ms, blit=False)

    # Save CSV on exit
    try:
        plt.show()
    finally:
        try:
            mqttc.loop_stop()
            mqttc.disconnect()
        except Exception:
            pass

        if records:
            df = pd.DataFrame.from_records(records)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_dir = os.path.expanduser("~/imu_logs")
            os.makedirs(out_dir, exist_ok=True)
            csv_path = os.path.join(out_dir, f"imu_log_{ts}.csv")
            df.to_csv(csv_path, index=False)
            print(f"💾 Saved CSV: {csv_path}  | rows: {len(df)}", flush=True)
        else:
            print("⚠️ No samples saved (records is empty).", flush=True)


if __name__ == "__main__":
    main()
