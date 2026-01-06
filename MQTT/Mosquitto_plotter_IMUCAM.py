#!/usr/bin/env python3
import time
import os
from collections import deque
from datetime import datetime
import threading
import queue

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

TOPIC_RAW   = "imu/jetson01/raw"         # IMU: CSV 12 fields
TOPIC_COLOR = "cam/jetson01/color_jpg"   # Color: JPEG bytes
TOPIC_DEPTH = "cam/jetson01/depth_png"   # Depth: RECOMMENDED uint16 PNG bytes (Z16 in mm). If you still use JPG, it will show but no metric distance.

SHOW_DEPTH = True

# ----------------------------
# UI / logging configuration
# ----------------------------
PLOT_HZ = 20.0
WINDOW_SEC = 10.0

# IMU logging rate to DataFrame
STORE_HZ = 50.0
STORE_PERIOD = (1.0 / STORE_HZ) if STORE_HZ > 0 else 0.0

# Camera decode/display rate (keep this <= what your laptop can decode smoothly)
CAM_FPS = 12.0
CAM_PERIOD = 1.0 / CAM_FPS

# Tracking configuration
MAX_TRACK_POINTS = 6
FEATURE_QUALITY = 0.01
FEATURE_MIN_DIST = 18

# Depth display scaling (only affects visualization)
DEPTH_VIS_MAX_M = 4.0   # meters for colormap scaling if depth is uint16 mm

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


def decode_jpg_to_rgb(jpg_bytes: bytes):
    """Decode JPEG bytes to RGB numpy array, or None."""
    if not jpg_bytes:
        return None
    arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def decode_depth_bytes(depth_bytes: bytes):
    """
    Decode depth payload.
    - If it's uint16 PNG (recommended): returns (depth_mm_uint16, depth_vis_rgb)
    - If it's JPEG/RGB: returns (None, rgb_vis)  (no metric depth available)
    """
    if not depth_bytes:
        return None, None

    arr = np.frombuffer(depth_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if img is None:
        return None, None

    # Case 1: uint16 depth (mm)
    if img.dtype == np.uint16 and img.ndim == 2:
        depth_mm = img

        # Make a pretty visualization (colormap) for display only
        depth_m = depth_mm.astype(np.float32) / 1000.0
        depth_m = np.clip(depth_m, 0.0, DEPTH_VIS_MAX_M)
        depth_u8 = (depth_m / DEPTH_VIS_MAX_M * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        depth_vis_rgb = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)
        return depth_mm, depth_vis_rgb

    # Case 2: already RGB/BGR-ish (e.g., jpg colormap)
    if img.ndim == 3 and img.shape[2] in (3, 4):
        bgr = img[:, :, :3]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return None, rgb

    return None, None


def make_mqtt_client(imu_queue: "queue.Queue[str]", frame_lock: threading.Lock, frames: dict):
    """
    MQTT client:
      - IMU CSV -> imu_queue (bounded)
      - Camera frames -> store ONLY latest bytes (overwrite previous)
      - IMPORTANT: callback does NOT decode images (fast)
    """
    client_id = f"pc-plot-{int(time.time())}"
    c = mqtt.Client(client_id=client_id, clean_session=True, protocol=mqtt.MQTTv311)

    # Reduce internal buffering (helps a bit)
    try:
        c.max_inflight_messages_set(20)
        c.max_queued_messages_set(0)  # 0 = unlimited in some versions; if unsupported, ignore
    except Exception:
        pass

    if MQTT_USER:
        c.username_pw_set(MQTT_USER, MQTT_PASS)

    c.reconnect_delay_set(min_delay=1, max_delay=30)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT connected", flush=True)
            client.subscribe(TOPIC_RAW, qos=0)
            client.subscribe(TOPIC_COLOR, qos=0)
            if SHOW_DEPTH:
                client.subscribe(TOPIC_DEPTH, qos=0)
            print(f"✅ Subscribed IMU  : {TOPIC_RAW}", flush=True)
            print(f"✅ Subscribed COLOR: {TOPIC_COLOR}", flush=True)
            if SHOW_DEPTH:
                print(f"✅ Subscribed DEPTH: {TOPIC_DEPTH}", flush=True)
        else:
            print(f"⚠️  MQTT connect failed rc={rc}", flush=True)

    def on_disconnect(client, userdata, rc):
        print(f"⚠️  MQTT disconnected rc={rc} (will retry)", flush=True)

    def on_message(client, userdata, msg):
        try:
            if msg.topic == TOPIC_RAW:
                payload = msg.payload.decode("utf-8", errors="ignore").strip()
                if payload:
                    # Keep queue bounded: prefer latest for real-time view
                    if imu_queue.qsize() > 200:
                        try:
                            while imu_queue.qsize() > 50:
                                imu_queue.get_nowait()
                        except queue.Empty:
                            pass
                    imu_queue.put_nowait(payload)
                return

            if msg.topic == TOPIC_COLOR:
                with frame_lock:
                    frames["color_bytes"] = msg.payload
                    frames["color_rx_ts"] = time.monotonic()
                return

            if SHOW_DEPTH and msg.topic == TOPIC_DEPTH:
                with frame_lock:
                    frames["depth_bytes"] = msg.payload
                    frames["depth_rx_ts"] = time.monotonic()
                return

        except Exception:
            pass

    c.on_connect = on_connect
    c.on_disconnect = on_disconnect
    c.on_message = on_message
    return c


def main():
    imu_queue: "queue.Queue[str]" = queue.Queue()

    frame_lock = threading.Lock()
    frames = {
        "color_bytes": None,
        "depth_bytes": None,
        "color_rx_ts": 0.0,
        "depth_rx_ts": 0.0,
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

    # Circular buffers for IMU plotting
    maxlen = int(WINDOW_SEC * PLOT_HZ) + 10
    t0 = time.monotonic()
    t = deque(maxlen=maxlen)

    acc_x = deque(maxlen=maxlen); acc_y = deque(maxlen=maxlen); acc_z = deque(maxlen=maxlen)
    gyr_x = deque(maxlen=maxlen); gyr_y = deque(maxlen=maxlen); gyr_z = deque(maxlen=maxlen)
    mag_x = deque(maxlen=maxlen); mag_y = deque(maxlen=maxlen); mag_z = deque(maxlen=maxlen)

    # IMU logging
    records = []
    last_store_t = time.monotonic()

    # ----------------------------
    # Figure layout (bigger camera)
    # ----------------------------
    plt.rcParams["figure.dpi"] = 110
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(
        4, 2,
        width_ratios=[3.2, 1.8],     # left = big camera, right = plots/text
        height_ratios=[1.0, 2.3, 2.3, 2.3],
        wspace=0.15, hspace=0.20
    )

    # Left column: big color and big depth (stacked)
    ax_color = fig.add_subplot(gs[0:2, 0])
    ax_depth = fig.add_subplot(gs[2:4, 0]) if SHOW_DEPTH else None

    for ax in [ax_color, ax_depth] if SHOW_DEPTH else [ax_color]:
        ax.axis("off")

    ax_color.set_title("Color (waiting...)")
    color_artist = ax_color.imshow(np.zeros((360, 640, 3), dtype=np.uint8))

    if SHOW_DEPTH and ax_depth is not None:
        ax_depth.set_title("Depth (waiting...)")
        depth_artist = ax_depth.imshow(np.zeros((360, 640, 3), dtype=np.uint8))
    else:
        depth_artist = None

    # Right column: dashboard + IMU plots
    ax_text = fig.add_subplot(gs[0, 1])
    ax_text.axis("off")
    text_artist = ax_text.text(
        0.01, 0.5, "Waiting for MQTT data...",
        va="center", ha="left", fontsize=11, family="monospace"
    )

    ax_acc = fig.add_subplot(gs[1, 1])
    ax_gyr = fig.add_subplot(gs[2, 1], sharex=ax_acc)
    ax_mag = fig.add_subplot(gs[3, 1], sharex=ax_acc)

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

    fig.suptitle("Jetson IMU + RealSense (Color/Depth) via MQTT — Live Dashboard", fontsize=16)

    # ----------------------------
    # Tracking state
    # ----------------------------
    tracked_pts = []  # list of (x,y) floats in image coordinates
    prev_gray = None
    last_color_rgb = None
    last_depth_mm = None
    last_depth_vis = None

    # Simple text overlay artist for points info (on color)
    overlay_text = ax_color.text(
        0.01, 0.99, "",
        transform=ax_color.transAxes,
        va="top", ha="left",
        fontsize=11, family="monospace",
        bbox=dict(facecolor="black", alpha=0.35, pad=6),
        color="white"
    )

    def on_click(event):
        """Mouse click handler: add a tracking point (prefer depth window clicks)."""
        nonlocal tracked_pts
        if event.inaxes not in (ax_color, ax_depth):
            return
        if event.xdata is None or event.ydata is None:
            return

        x = float(event.xdata)
        y = float(event.ydata)

        if len(tracked_pts) >= MAX_TRACK_POINTS:
            tracked_pts = tracked_pts[1:]  # drop oldest

        tracked_pts.append((x, y))

    fig.canvas.mpl_connect("button_press_event", on_click)

    # ----------------------------
    # Helpers
    # ----------------------------
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
        """Drain IMU queue and keep only the newest valid sample."""
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

    def track_points_optical_flow(curr_rgb):
        """
        Track points using Lucas-Kanade optical flow on the COLOR image.
        This keeps points stable while the camera moves.
        """
        nonlocal tracked_pts, prev_gray

        if curr_rgb is None:
            return

        curr_gray = cv2.cvtColor(curr_rgb, cv2.COLOR_RGB2GRAY)

        if prev_gray is None:
            prev_gray = curr_gray
            return

        if len(tracked_pts) == 0:
            prev_gray = curr_gray
            return

        p0 = np.array(tracked_pts, dtype=np.float32).reshape(-1, 1, 2)

        p1, st, _err = cv2.calcOpticalFlowPyrLK(
            prev_gray, curr_gray, p0, None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03)
        )

        if p1 is None or st is None:
            prev_gray = curr_gray
            return

        st = st.reshape(-1)
        p1 = p1.reshape(-1, 2)

        new_pts = []
        h, w = curr_gray.shape[:2]
        for ok, (x, y) in zip(st, p1):
            if not ok:
                continue
            if 0 <= x < w and 0 <= y < h:
                new_pts.append((float(x), float(y)))

        tracked_pts = new_pts[:MAX_TRACK_POINTS]
        prev_gray = curr_gray

    def sample_depth_at_points(depth_mm, pts):
        """
        Return list of distances in meters (or None if not available).
        depth_mm must be uint16 depth in millimeters.
        """
        if depth_mm is None:
            return [None for _ in pts]

        h, w = depth_mm.shape[:2]
        out = []
        for (x, y) in pts:
            ix = int(round(x))
            iy = int(round(y))
            if 0 <= ix < w and 0 <= iy < h:
                mm = int(depth_mm[iy, ix])
                if mm == 0:
                    out.append(None)
                else:
                    out.append(mm / 1000.0)
            else:
                out.append(None)
        return out

    # ----------------------------
    # Update loop
    # ----------------------------
    last_cam_draw = 0.0

    def update(_frame):
        nonlocal last_color_rgb, last_depth_mm, last_depth_vis
        nonlocal last_cam_draw

        # --- IMU ---
        poll_mqtt_imu()

        # --- Camera decode (latest only; no accumulation) ---
        now_m = time.monotonic()
        cam_age_color = None
        cam_age_depth = None

        if (now_m - last_cam_draw) >= CAM_PERIOD:
            last_cam_draw = now_m

            with frame_lock:
                color_bytes = frames["color_bytes"]
                depth_bytes = frames["depth_bytes"] if SHOW_DEPTH else None
                color_rx_ts = frames["color_rx_ts"]
                depth_rx_ts = frames["depth_rx_ts"]

                # Clear so we don't keep old frames around (prevents local accumulation)
                frames["color_bytes"] = None
                frames["depth_bytes"] = None

            if color_bytes is not None:
                last_color_rgb = decode_jpg_to_rgb(color_bytes)
                cam_age_color = (time.monotonic() - color_rx_ts) if color_rx_ts > 0 else 0.0

            if SHOW_DEPTH and depth_bytes is not None:
                depth_mm, depth_vis = decode_depth_bytes(depth_bytes)
                last_depth_mm = depth_mm
                last_depth_vis = depth_vis
                cam_age_depth = (time.monotonic() - depth_rx_ts) if depth_rx_ts > 0 else 0.0

        # Track points on color
        if last_color_rgb is not None:
            track_points_optical_flow(last_color_rgb)

        # Compute distances at points
        dists = sample_depth_at_points(last_depth_mm, tracked_pts)

        # Draw overlays on color image (copy to keep base frame clean)
        if last_color_rgb is not None:
            vis = last_color_rgb.copy()
            for i, ((x, y), dist) in enumerate(zip(tracked_pts, dists), start=1):
                ix = int(round(x)); iy = int(round(y))
                cv2.circle(vis, (ix, iy), 6, (0, 255, 0), 2)
                label = f"P{i}"
                if dist is not None:
                    label += f" {dist:.2f}m"
                else:
                    label += " N/A"
                cv2.putText(vis, label, (ix + 8, iy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            color_artist.set_data(vis)
            title = "Color"
            if cam_age_color is not None:
                title += f" (rx age {cam_age_color:.2f}s)"
            ax_color.set_title(title)

            # Overlay block with distances
            lines = ["Depth @ points (click to add):"]
            for i, dist in enumerate(dists, start=1):
                if dist is None:
                    lines.append(f"  P{i}: N/A")
                else:
                    lines.append(f"  P{i}: {dist:.3f} m")
            overlay_text.set_text("\n".join(lines))
        else:
            ax_color.set_title("Color (waiting...)")
            overlay_text.set_text("Click to add points.\nWaiting for color...")

        # Depth display
        if SHOW_DEPTH and depth_artist is not None:
            if last_depth_vis is not None:
                depth_artist.set_data(last_depth_vis)
                title = "Depth"
                if last_depth_mm is not None:
                    title += " (uint16 mm ✅)"
                else:
                    title += " (visual only ❌ metric)"
                if cam_age_depth is not None:
                    title += f" (rx age {cam_age_depth:.2f}s)"
                ax_depth.set_title(title)
            else:
                ax_depth.set_title("Depth (waiting...)")

        # --- IMU plots + dashboard ---
        if last_sample is None:
            text_artist.set_text(
                f"Waiting IMU...\n"
                f"Broker: {BROKER_HOST}:{BROKER_PORT}\n"
                f"IMU: {TOPIC_RAW}\n"
                f"COLOR: {TOPIC_COLOR}\n"
                f"DEPTH: {TOPIC_DEPTH}\n"
                f"Tip: publish DEPTH as uint16 PNG to get meters."
            )
        else:
            now = time.monotonic() - t0
            s = last_sample

            t.append(now)
            acc_x.append(s["ax_g"]);   acc_y.append(s["ay_g"]);   acc_z.append(s["az_g"])
            gyr_x.append(s["gx_dps"]); gyr_y.append(s["gy_dps"]); gyr_z.append(s["gz_dps"])
            mag_x.append(s["mx_uT"]);  mag_y.append(s["my_uT"]);  mag_z.append(s["mz_uT"])

            # Dashboard text
            dp = "metric ✅" if last_depth_mm is not None else "visual-only ❌"
            dash = (
                f"ACC [g]   ax={s['ax_g']:+8.3f}  ay={s['ay_g']:+8.3f}  az={s['az_g']:+8.3f}\n"
                f"GYR [dps] gx={s['gx_dps']:+8.3f}  gy={s['gy_dps']:+8.3f}  gz={s['gz_dps']:+8.3f}\n"
                f"MAG [uT]  mx={s['mx_uT']:+9.3f}  my={s['my_uT']:+9.3f}  mz={s['mz_uT']:+9.3f}\n"
                f"P/T/Alt   P={s['p_hpa']:9.2f} hPa   T={s['t_C']:6.2f} °C   Alt={s['alt_m']:8.2f} m\n"
                f"CAM_FPS={CAM_FPS:.1f}  Depth={dp}  Points={len(tracked_pts)}\n"
                f"Log: {('ALL' if STORE_HZ <= 0 else f'{STORE_HZ:.1f} Hz')}  Samples={len(records)}"
            )
            text_artist.set_text(dash)

            # Update plot lines
            acc_l1.set_data(t, acc_x); acc_l2.set_data(t, acc_y); acc_l3.set_data(t, acc_z)
            gyr_l1.set_data(t, gyr_x); gyr_l2.set_data(t, gyr_y); gyr_l3.set_data(t, gyr_z)
            mag_l1.set_data(t, mag_x); mag_l2.set_data(t, mag_y); mag_l3.set_data(t, mag_z)

            # Sliding time window
            if len(t) > 2:
                ax_acc.set_xlim(max(0.0, t[-1] - WINDOW_SEC), t[-1])

            ax_acc.relim(); ax_acc.autoscale_view(scalex=False, scaley=True)
            ax_gyr.relim(); ax_gyr.autoscale_view(scalex=False, scaley=True)
            ax_mag.relim(); ax_mag.autoscale_view(scalex=False, scaley=True)

        artists = [
            text_artist, color_artist, overlay_text,
            acc_l1, acc_l2, acc_l3, gyr_l1, gyr_l2, gyr_l3, mag_l1, mag_l2, mag_l3
        ]
        if SHOW_DEPTH and depth_artist is not None:
            artists.append(depth_artist)
        return tuple(artists)

    interval_ms = int(1000.0 / PLOT_HZ)
    _ani = FuncAnimation(fig, update, interval=interval_ms, blit=False)

    # Save IMU CSV on exit
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
            print("⚠️ No IMU samples saved.", flush=True)


if __name__ == "__main__":
    main()
