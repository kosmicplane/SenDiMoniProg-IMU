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
TOPIC_COLOR = "cam/jetson01/color_jpg"   # Camera: JPEG bytes
TOPIC_DEPTH = "cam/jetson01/depth_jpg"   # Depth preview: JPEG bytes (optional)

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
CAM_FPS = 5                     # decode/display at most this rate
CAM_PERIOD = 1.0 / CAM_FPS

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
      - stores ONLY latest JPEG bytes for color/depth in frames (no decode here)
        to avoid backlog/lag.
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
            if SHOW_DEPTH:
                client.subscribe(TOPIC_DEPTH, qos=0)
            print(f"✅ Subscribed IMU: {TOPIC_RAW}", flush=True)
            print(f"✅ Subscribed CAM: {TOPIC_COLOR}" + (f" + {TOPIC_DEPTH}" if SHOW_DEPTH else ""), flush=True)
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

            # Camera: store latest bytes ONLY (decode in UI loop)
            if msg.topic == TOPIC_COLOR or (SHOW_DEPTH and msg.topic == TOPIC_DEPTH):
                with frame_lock:
                    if msg.topic == TOPIC_COLOR:
                        frames["color_jpg"] = msg.payload
                        frames["color_ts"] = time.monotonic()
                    else:
                        frames["depth_jpg"] = msg.payload
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


def main():
    # Thread-safe queue for IMU MQTT -> UI
    imu_queue: "queue.Queue[str]" = queue.Queue()

    # Latest JPEG frames shared between MQTT callback and Matplotlib update()
    frame_lock = threading.Lock()
    frames = {
        "color_jpg": None,
        "depth_jpg": None,
        "color_ts": 0.0,
        "depth_ts": 0.0,
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

    # Data logging storage (later converted to a DataFrame)
    records = []
    last_store_t = time.monotonic()

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
    else:
        ax_img_color = fig.add_subplot(gs[1])
        ax_img_depth = None
        depth_artist = None

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

    fig.suptitle("Jetson IMU + Camera via MQTT — Live Plot + Dashboard + CSV logger", fontsize=14)

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
    cached_depth_rgb = None
    cached_color_ts = 0.0
    cached_depth_ts = 0.0

    def update(_frame):
        """Matplotlib animation callback: update camera + IMU plots."""
        nonlocal last_sample, last_cam_draw
        nonlocal cached_color_rgb, cached_depth_rgb, cached_color_ts, cached_depth_ts

        # --- IMU ---
        poll_mqtt_imu()

        # --- Camera (decode only latest, at CAM_FPS) ---
        now_m = time.monotonic()
        if (now_m - last_cam_draw) >= CAM_PERIOD:
            last_cam_draw = now_m
            with frame_lock:
                color_jpg = frames["color_jpg"]
                depth_jpg = frames["depth_jpg"] if SHOW_DEPTH else None
                color_ts  = frames["color_ts"]
                depth_ts  = frames["depth_ts"]

            # Decode only if there is something new (or first time)
            if color_jpg is not None and color_ts != cached_color_ts:
                rgb = decode_jpg_to_rgb(color_jpg)
                if rgb is not None:
                    cached_color_rgb = rgb
                    cached_color_ts = color_ts

            if SHOW_DEPTH and depth_jpg is not None and depth_ts != cached_depth_ts:
                rgb = decode_jpg_to_rgb(depth_jpg)
                if rgb is not None:
                    cached_depth_rgb = rgb
                    cached_depth_ts = depth_ts

        # Update image artists using cached frames (fast)
        if cached_color_rgb is not None:
            color_artist.set_data(cached_color_rgb)
            age = (time.monotonic() - cached_color_ts) if cached_color_ts > 0 else 0.0
            ax_img_color.set_title(f"Color (age {age:.2f}s, cam_fps {CAM_FPS:.1f})")
        else:
            ax_img_color.set_title("Color (waiting...)")

        if SHOW_DEPTH and depth_artist is not None:
            if cached_depth_rgb is not None:
                depth_artist.set_data(cached_depth_rgb)
                age = (time.monotonic() - cached_depth_ts) if cached_depth_ts > 0 else 0.0
                ax_img_depth.set_title(f"Depth (age {age:.2f}s, cam_fps {CAM_FPS:.1f})")
            else:
                ax_img_depth.set_title("Depth (waiting...)")

        # --- Dashboard + IMU plots ---
        if last_sample is None:
            text_artist.set_text(
                f"Waiting for MQTT IMU data...\n"
                f"Broker: {BROKER_HOST}:{BROKER_PORT}\n"
                f"IMU: {TOPIC_RAW}\n"
                f"CAM: {TOPIC_COLOR}" + (f"\nDEPTH: {TOPIC_DEPTH}" if SHOW_DEPTH else "")
            )
            artists = [text_artist, color_artist, acc_l1, acc_l2, acc_l3, gyr_l1, gyr_l2, gyr_l3, mag_l1, mag_l2, mag_l3]
            if SHOW_DEPTH and depth_artist is not None:
                artists.append(depth_artist)
            return tuple(artists)

        now = time.monotonic() - t0
        s = last_sample

        # Append to plot buffers (at UI rate)
        t.append(now)

        acc_x.append(s["ax_g"]);   acc_y.append(s["ay_g"]);   acc_z.append(s["az_g"])
        gyr_x.append(s["gx_dps"]); gyr_y.append(s["gy_dps"]); gyr_z.append(s["gz_dps"])
        mag_x.append(s["mx_uT"]);  mag_y.append(s["my_uT"]);  mag_z.append(s["mz_uT"])

        dash = (
            f"ACC [g]   ax={s['ax_g']:+8.3f}  ay={s['ay_g']:+8.3f}  az={s['az_g']:+8.3f}\n"
            f"GYR [dps] gx={s['gx_dps']:+8.3f}  gy={s['gy_dps']:+8.3f}  gz={s['gz_dps']:+8.3f}\n"
            f"MAG [uT]  mx={s['mx_uT']:+9.3f}  my={s['my_uT']:+9.3f}  mz={s['mz_uT']:+9.3f}\n"
            f"P/T/Alt   P={s['p_hpa']:9.2f} hPa   T={s['t_C']:6.2f} °C   Alt={s['alt_m']:8.2f} m\n"
            f"UI: {PLOT_HZ:.1f} Hz   Window: {WINDOW_SEC:.1f} s   Log: "
            f"{('ALL' if STORE_HZ <= 0 else f'{STORE_HZ:.1f} Hz')}   Samples saved: {len(records)}"
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
        return tuple(artists)

    interval_ms = int(1000.0 / PLOT_HZ)
    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False)

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
