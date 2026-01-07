#!/usr/bin/env python3
import json
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import pandas as pd
import paho.mqtt.client as mqtt

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider

# ----------------------------
# MQTT configuration
# ----------------------------
BROKER_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")

TOPIC_RAW = "imu/jetson01/raw"           # IMU: CSV 12 fields
TOPIC_COLOR = "cam/jetson01/color_jpg"   # Camera: JPEG bytes
TOPIC_DEPTH = "cam/jetson01/depth_jpg"   # Depth preview: JPEG bytes (optional)
TOPIC_DEPTH_RAW = "cam/jetson01/depth_z16_png"
TOPIC_META = "cam/jetson01/meta"
TOPIC_CONTROL = "cam/jetson01/control"
TOPIC_STATUS = "cam/jetson01/status"
TOPIC_CALIB = "cam/jetson01/calib"

SHOW_DEPTH = True  # set False if you don't publish depth preview

# ----------------------------
# UI / logging configuration
# ----------------------------
PLOT_HZ = 20.0
WINDOW_SEC = 10.0
STORE_HZ = 50.0
STORE_PERIOD = (1.0 / STORE_HZ) if STORE_HZ > 0 else 0.0

CAM_FPS = 30
CAM_PERIOD = 1.0 / CAM_FPS

DEMO_MODE = os.getenv("DEMO_MODE", "0") == "1"

# Expected CSV layout: 12 values per line
FIELDS = [
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "mx_uT", "my_uT", "mz_uT",
    "p_hpa", "t_C", "alt_m",
]


class SharedFrames:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.data: Dict[str, Any] = {
            "color_rgb": None,
            "depth_rgb": None,
            "depth_raw": None,
            "color_ts": 0.0,
            "depth_ts": 0.0,
            "depth_raw_ts": 0.0,
            "color_recv_fps": 0.0,
            "depth_recv_fps": 0.0,
        }

    def update(self, key: str, value: Any, ts: float, ts_key: str, fps_key: Optional[str] = None) -> None:
        with self.lock:
            self.data[key] = value
            self.data[ts_key] = ts
            if fps_key:
                prev_ts = self.data.get(f"{fps_key}_prev_ts", 0.0)
                if prev_ts > 0:
                    fps = 1.0 / max(1e-6, ts - prev_ts)
                    self.data[fps_key] = fps
                self.data[f"{fps_key}_prev_ts"] = ts

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return dict(self.data)


def parse_csv12(line: str) -> Optional[Dict[str, float]]:
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 12:
        return None
    try:
        vals = list(map(float, parts))
        return dict(zip(FIELDS, vals))
    except ValueError:
        return None


def decode_jpg_to_rgb(jpg_bytes: bytes) -> Optional[np.ndarray]:
    if not jpg_bytes:
        return None
    arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def decode_png16(depth_bytes: bytes) -> Optional[np.ndarray]:
    if not depth_bytes:
        return None
    arr = np.frombuffer(depth_bytes, dtype=np.uint8)
    depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if depth is None or depth.dtype != np.uint16:
        return None
    return depth


def make_mqtt_client(
    imu_queue: "queue.Queue[str]",
    frame_queue: "queue.Queue[Tuple[str, bytes, float]]",
    meta_state: Dict[str, Any],
    meta_lock: threading.Lock,
):
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
            client.subscribe(TOPIC_STATUS, qos=0)
            client.subscribe(TOPIC_CALIB, qos=0)
            if SHOW_DEPTH:
                client.subscribe(TOPIC_DEPTH, qos=0)
            client.subscribe(TOPIC_DEPTH_RAW, qos=0)
            print(f"✅ Subscribed IMU: {TOPIC_RAW}", flush=True)
            print(f"✅ Subscribed CAM: {TOPIC_COLOR}", flush=True)
        else:
            print(f"⚠️  MQTT connect failed rc={rc}", flush=True)

    def on_disconnect(client, userdata, rc):
        print(f"⚠️  MQTT disconnected rc={rc} (will retry)", flush=True)

    def on_message(client, userdata, msg):
        try:
            if msg.topic == TOPIC_RAW:
                payload = msg.payload.decode("utf-8", errors="ignore").strip()
                if payload:
                    if imu_queue.qsize() > 200:
                        try:
                            while imu_queue.qsize() > 50:
                                imu_queue.get_nowait()
                        except queue.Empty:
                            pass
                    imu_queue.put_nowait(payload)
                return

            if msg.topic in {TOPIC_META, TOPIC_STATUS, TOPIC_CALIB}:
                payload = msg.payload.decode("utf-8", errors="ignore")
                meta = json.loads(payload) if payload else {}
                if isinstance(meta, dict):
                    with meta_lock:
                        meta_state.update(meta)
                        meta_state["_meta_ts"] = time.time()
                return

            if msg.topic in {TOPIC_COLOR, TOPIC_DEPTH, TOPIC_DEPTH_RAW}:
                try:
                    frame_queue.put_nowait((msg.topic, msg.payload, time.monotonic()))
                except queue.Full:
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        frame_queue.put_nowait((msg.topic, msg.payload, time.monotonic()))
                    except queue.Full:
                        pass
        except Exception:
            pass

    c.on_connect = on_connect
    c.on_disconnect = on_disconnect
    c.on_message = on_message

    return c


def compute_3d_distance(
    p1: Tuple[int, int],
    p2: Tuple[int, int],
    depth: np.ndarray,
    intrinsics: Dict[str, float],
    depth_scale: float,
) -> Tuple[Optional[float], str]:
    if depth is None or intrinsics is None or depth_scale is None:
        return None, "No depth/intrinsics"

    h, w = depth.shape[:2]
    u1, v1 = p1
    u2, v2 = p2
    if not (0 <= u1 < w and 0 <= v1 < h and 0 <= u2 < w and 0 <= v2 < h):
        return None, "Point out of range"

    z1 = depth[v1, u1] * depth_scale
    z2 = depth[v2, u2] * depth_scale
    if z1 <= 0 or z2 <= 0:
        return None, "No valid depth"

    fx = intrinsics["fx"]
    fy = intrinsics["fy"]
    ppx = intrinsics["ppx"]
    ppy = intrinsics["ppy"]

    x1 = (u1 - ppx) / fx * z1
    y1 = (v1 - ppy) / fy * z1
    x2 = (u2 - ppx) / fx * z2
    y2 = (v2 - ppy) / fy * z2

    dist = float(np.linalg.norm([x2 - x1, y2 - y1, z2 - z1]))
    return dist, ""


def generate_demo_frames(t: float, width: int = 640, height: int = 480) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    x = np.linspace(0, 1, width)
    y = np.linspace(0, 1, height)
    xv, yv = np.meshgrid(x, y)
    color = np.stack([
        (xv * 255).astype(np.uint8),
        (yv * 255).astype(np.uint8),
        ((0.5 + 0.5 * np.sin(t)) * 255 * np.ones_like(xv)).astype(np.uint8),
    ], axis=-1)
    depth_preview = cv2.applyColorMap(((xv + yv) * 127).astype(np.uint8), cv2.COLORMAP_TURBO)
    depth_raw = ((xv + yv) * 1000 + 500).astype(np.uint16)
    return color, depth_preview, depth_raw


def main():
    imu_queue: "queue.Queue[str]" = queue.Queue()
    frame_queue: "queue.Queue[Tuple[str, bytes, float]]" = queue.Queue(maxsize=4)

    shared_frames = SharedFrames()
    meta_state: Dict[str, Any] = {}
    meta_lock = threading.Lock()

    mqttc = make_mqtt_client(imu_queue, frame_queue, meta_state, meta_lock)

    while True:
        try:
            mqttc.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
            break
        except Exception as e:
            print(f"⚠️  MQTT retry: {e}", flush=True)
            time.sleep(2)

    mqttc.loop_start()

    stop_event = threading.Event()

    def decode_worker():
        while not stop_event.is_set():
            try:
                topic, payload, ts = frame_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if topic == TOPIC_COLOR:
                rgb = decode_jpg_to_rgb(payload)
                if rgb is not None:
                    shared_frames.update("color_rgb", rgb, ts, "color_ts", "color_recv_fps")
            elif topic == TOPIC_DEPTH:
                rgb = decode_jpg_to_rgb(payload)
                if rgb is not None:
                    shared_frames.update("depth_rgb", rgb, ts, "depth_ts", "depth_recv_fps")
            elif topic == TOPIC_DEPTH_RAW:
                depth = decode_png16(payload)
                if depth is not None:
                    shared_frames.update("depth_raw", depth, ts, "depth_raw_ts")

    decoder_thread = threading.Thread(target=decode_worker, daemon=True)
    decoder_thread.start()

    last_sample = None
    maxlen = int(WINDOW_SEC * PLOT_HZ) + 10
    t0 = time.monotonic()
    t = deque(maxlen=maxlen)

    acc_x = deque(maxlen=maxlen)
    acc_y = deque(maxlen=maxlen)
    acc_z = deque(maxlen=maxlen)
    gyr_x = deque(maxlen=maxlen)
    gyr_y = deque(maxlen=maxlen)
    gyr_z = deque(maxlen=maxlen)
    mag_x = deque(maxlen=maxlen)
    mag_y = deque(maxlen=maxlen)
    mag_z = deque(maxlen=maxlen)

    records = []
    last_store_t = time.monotonic()

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(3, 2, height_ratios=[3.0, 1.4, 1.4], width_ratios=[3.2, 1.1])

    gs_img = gs[0, 0].subgridspec(1, 2, wspace=0.02)
    ax_img_color = fig.add_subplot(gs_img[0, 0])
    ax_img_color.axis("off")
    ax_img_color.set_title("Color")
    color_artist = ax_img_color.imshow(np.zeros((240, 320, 3), dtype=np.uint8), aspect="equal")

    ax_img_depth = fig.add_subplot(gs_img[0, 1])
    ax_img_depth.axis("off")
    ax_img_depth.set_title("Depth")
    depth_artist = ax_img_depth.imshow(np.zeros((240, 320, 3), dtype=np.uint8), aspect="equal")

    overlay_box = dict(facecolor="black", alpha=0.5, pad=4)
    color_overlay = ax_img_color.text(0.02, 0.98, "", transform=ax_img_color.transAxes,
                                      va="top", ha="left", color="white", fontsize=9, bbox=overlay_box)
    depth_overlay = ax_img_depth.text(0.02, 0.98, "", transform=ax_img_depth.transAxes,
                                      va="top", ha="left", color="white", fontsize=9, bbox=overlay_box)

    ax_ctrl = fig.add_subplot(gs[0, 1])
    ax_ctrl.axis("off")
    ctrl_text = ax_ctrl.text(0.02, 0.98, "Controls", va="top", ha="left", fontsize=11, family="monospace")

    gs_imu = gs[1:, :].subgridspec(3, 1, hspace=0.3)
    ax_acc = fig.add_subplot(gs_imu[0])
    ax_gyr = fig.add_subplot(gs_imu[1], sharex=ax_acc)
    ax_mag = fig.add_subplot(gs_imu[2], sharex=ax_acc)

    acc_l1, = ax_acc.plot([], [], label="ax_g")
    acc_l2, = ax_acc.plot([], [], label="ay_g")
    acc_l3, = ax_acc.plot([], [], label="az_g")
    ax_acc.set_ylabel("acc [g]")
    ax_acc.grid(True)
    ax_acc.legend(loc="upper right")

    gyr_l1, = ax_gyr.plot([], [], label="gx_dps")
    gyr_l2, = ax_gyr.plot([], [], label="gy_dps")
    gyr_l3, = ax_gyr.plot([], [], label="gz_dps")
    ax_gyr.set_ylabel("gyro [dps]")
    ax_gyr.grid(True)
    ax_gyr.legend(loc="upper right")

    mag_l1, = ax_mag.plot([], [], label="mx_uT")
    mag_l2, = ax_mag.plot([], [], label="my_uT")
    mag_l3, = ax_mag.plot([], [], label="mz_uT")
    ax_mag.set_ylabel("mag [uT]")
    ax_mag.set_xlabel("time [s]")
    ax_mag.grid(True)
    ax_mag.legend(loc="upper right")

    fig.suptitle("Jetson IMU + Camera via MQTT — Live View + Controls", fontsize=14)

    control_state = {
        "jpeg_quality": 20,
        "pub_hz": 20,
        "resolution": "424x240",
        "publish_depth_preview": True,
        "publish_depth_raw": False,
        "measurement_mode": False,
    }

    ax_quality = fig.add_axes([0.72, 0.82, 0.23, 0.03])
    slider_quality = Slider(ax_quality, "JPEG", 5, 90, valinit=control_state["jpeg_quality"], valstep=1)

    ax_pub = fig.add_axes([0.72, 0.69, 0.23, 0.12])
    pub_hz_options = ["5", "10", "15", "20", "30"]
    radio_pub = RadioButtons(ax_pub, pub_hz_options, active=pub_hz_options.index(str(control_state["pub_hz"])))
    ax_pub.set_title("PUB_HZ", fontsize=9)

    ax_res = fig.add_axes([0.72, 0.52, 0.23, 0.14])
    res_options = ["424x240", "640x480", "848x480", "1280x720"]
    radio_res = RadioButtons(ax_res, res_options, active=res_options.index(control_state["resolution"]))
    ax_res.set_title("Resolution", fontsize=9)

    ax_check = fig.add_axes([0.72, 0.41, 0.23, 0.08])
    check = CheckButtons(ax_check, ["Depth preview", "Depth raw", "Measure"],
                         [control_state["publish_depth_preview"],
                          control_state["publish_depth_raw"],
                          control_state["measurement_mode"]])

    ax_apply = fig.add_axes([0.72, 0.35, 0.23, 0.045])
    btn_apply = Button(ax_apply, "Apply")

    ax_full_color = fig.add_axes([0.72, 0.29, 0.23, 0.045])
    btn_full_color = Button(ax_full_color, "Fullscreen Color")

    ax_full_depth = fig.add_axes([0.72, 0.24, 0.23, 0.045])
    btn_full_depth = Button(ax_full_depth, "Fullscreen Depth")

    zoom_figs: Dict[str, Any] = {"color": None, "depth": None}
    zoom_artists: Dict[str, Any] = {"color": None, "depth": None}

    def open_zoom(which: str):
        if zoom_figs[which] is None:
            fig_zoom, ax_zoom = plt.subplots()
            ax_zoom.axis("off")
            ax_zoom.set_title(which.capitalize())
            artist = ax_zoom.imshow(np.zeros((240, 320, 3), dtype=np.uint8), aspect="equal")
            fig_zoom.canvas.manager.full_screen_toggle()
            zoom_figs[which] = fig_zoom
            zoom_artists[which] = artist
        else:
            zoom_figs[which].canvas.manager.full_screen_toggle()

    btn_full_color.on_clicked(lambda _evt: open_zoom("color"))
    btn_full_depth.on_clicked(lambda _evt: open_zoom("depth"))

    def update_control_state(_evt=None):
        control_state["jpeg_quality"] = int(slider_quality.val)
        control_state["pub_hz"] = int(radio_pub.value_selected)
        control_state["resolution"] = radio_res.value_selected
        control_state["publish_depth_preview"] = check.get_status()[0]
        control_state["publish_depth_raw"] = check.get_status()[1]
        control_state["measurement_mode"] = check.get_status()[2]

    def send_control(_evt=None):
        update_control_state()
        w, h = control_state["resolution"].split("x")
        payload = {
            "jpeg_quality": control_state["jpeg_quality"],
            "pub_hz": control_state["pub_hz"],
            "color_w": int(w),
            "color_h": int(h),
            "depth_w": int(w),
            "depth_h": int(h),
            "publish_depth_preview": control_state["publish_depth_preview"],
            "publish_depth_raw": control_state["publish_depth_raw"],
        }
        mqttc.publish(TOPIC_CONTROL, json.dumps(payload), qos=0, retain=False)

    btn_apply.on_clicked(send_control)
    check.on_clicked(lambda _evt: update_control_state())

    measurement_points: list[Tuple[int, int]] = []
    measurement_text = ax_img_color.text(0.02, 0.02, "", transform=ax_img_color.transAxes,
                                         va="bottom", ha="left", color="yellow", fontsize=10,
                                         bbox=dict(facecolor="black", alpha=0.4, pad=3))
    measurement_line, = ax_img_color.plot([], [], color="yellow", linewidth=2)

    def on_click(event):
        if event.inaxes != ax_img_color:
            return
        if not control_state["measurement_mode"]:
            return
        if event.xdata is None or event.ydata is None:
            return
        measurement_points.append((int(event.xdata), int(event.ydata)))
        if len(measurement_points) > 2:
            measurement_points.pop(0)

    fig.canvas.mpl_connect("button_press_event", on_click)

    def maybe_store(sample, now_mono):
        nonlocal last_store_t
        if STORE_HZ <= 0:
            records.append({
                "t_wall": datetime.now().isoformat(timespec="milliseconds"),
                "t_mono": now_mono - t0,
                **sample,
            })
            return
        if (now_mono - last_store_t) >= STORE_PERIOD:
            last_store_t = now_mono
            records.append({
                "t_wall": datetime.now().isoformat(timespec="milliseconds"),
                "t_mono": now_mono - t0,
                **sample,
            })

    def poll_mqtt_imu():
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

    last_cam_draw = 0.0
    last_color_ts = 0.0
    last_depth_ts = 0.0
    last_depth_raw_ts = 0.0

    def update(_frame):
        nonlocal last_cam_draw, last_color_ts, last_depth_ts, last_depth_raw_ts

        poll_mqtt_imu()

        now_m = time.monotonic()
        if (now_m - last_cam_draw) >= CAM_PERIOD:
            last_cam_draw = now_m

        frame_snapshot = shared_frames.snapshot()

        color_rgb = frame_snapshot.get("color_rgb")
        depth_rgb = frame_snapshot.get("depth_rgb")
        depth_raw = frame_snapshot.get("depth_raw")

        if DEMO_MODE and color_rgb is None:
            demo_color, demo_depth_preview, demo_depth_raw = generate_demo_frames(time.time())
            color_rgb = demo_color
            depth_rgb = demo_depth_preview
            depth_raw = demo_depth_raw

        if color_rgb is not None:
            color_artist.set_data(color_rgb)
        if depth_rgb is not None:
            depth_artist.set_data(depth_rgb)

        if frame_snapshot.get("color_ts", 0.0) != last_color_ts:
            last_color_ts = frame_snapshot.get("color_ts", 0.0)
        if frame_snapshot.get("depth_ts", 0.0) != last_depth_ts:
            last_depth_ts = frame_snapshot.get("depth_ts", 0.0)
        if frame_snapshot.get("depth_raw_ts", 0.0) != last_depth_raw_ts:
            last_depth_raw_ts = frame_snapshot.get("depth_raw_ts", 0.0)

        with meta_lock:
            meta_snapshot = dict(meta_state)
        t_wall = meta_snapshot.get("t_wall")
        latency_ms = (time.time() - t_wall) * 1000.0 if t_wall else None
        color_bytes = meta_snapshot.get("color_bytes", 0)
        depth_bytes = meta_snapshot.get("depth_bytes", 0)
        pub_fps = meta_snapshot.get("pub_fps")
        pub_hz = meta_snapshot.get("pub_hz")
        res_w = meta_snapshot.get("w")
        res_h = meta_snapshot.get("h")

        latency_text = f"Latency: {latency_ms:.1f} ms" if latency_ms is not None else "Latency: -"
        pub_fps_text = f"{pub_fps:.1f}" if isinstance(pub_fps, (float, int)) else "-"

        color_overlay.set_text(
            "\n".join([
                f"Recv FPS: {frame_snapshot.get('color_recv_fps', 0.0):.1f}",
                f"Pub FPS: {pub_fps_text}",
                latency_text,
                f"Bytes: {color_bytes}",
                f"Res: {res_w}x{res_h}",
            ])
        )
        depth_overlay.set_text(
            "\n".join([
                f"Recv FPS: {frame_snapshot.get('depth_recv_fps', 0.0):.1f}",
                f"Pub FPS: {pub_fps_text}",
                latency_text,
                f"Bytes: {depth_bytes}",
                f"Res: {res_w}x{res_h}",
            ])
        )

        status_text = "\n".join([
            f"Broker: {BROKER_HOST}:{BROKER_PORT}",
            f"PUB_HZ: {pub_hz}  JPEG: {meta_snapshot.get('jpeg_quality', '-')}",
            f"Depth preview: {meta_snapshot.get('publish_depth_preview', '-')}",
            f"Depth raw: {meta_snapshot.get('publish_depth_raw', '-')}",
            f"Resolution: {res_w}x{res_h}",
            latency_text,
        ])
        ctrl_text.set_text(status_text)

        if control_state["measurement_mode"] and len(measurement_points) == 2:
            p1, p2 = measurement_points
            measurement_line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
            intrinsics = meta_snapshot.get("intrinsics")
            depth_scale = meta_snapshot.get("depth_scale")
            dist, err = compute_3d_distance(p1, p2, depth_raw, intrinsics, depth_scale)
            px_dist = float(np.linalg.norm(np.array(p2) - np.array(p1)))
            if dist is None:
                measurement_text.set_text(f"2D: {px_dist:.1f}px\n3D: {err}")
            else:
                measurement_text.set_text(f"2D: {px_dist:.1f}px\n3D: {dist:.3f} m")
        else:
            measurement_line.set_data([], [])
            measurement_text.set_text("")

        if zoom_figs["color"] is not None and zoom_artists["color"] is not None and color_rgb is not None:
            zoom_artists["color"].set_data(color_rgb)
            zoom_figs["color"].canvas.draw_idle()
        if zoom_figs["depth"] is not None and zoom_artists["depth"] is not None and depth_rgb is not None:
            zoom_artists["depth"].set_data(depth_rgb)
            zoom_figs["depth"].canvas.draw_idle()

        if last_sample is not None:
            now = time.monotonic() - t0
            s = last_sample
            t.append(now)

            acc_x.append(s["ax_g"])
            acc_y.append(s["ay_g"])
            acc_z.append(s["az_g"])
            gyr_x.append(s["gx_dps"])
            gyr_y.append(s["gy_dps"])
            gyr_z.append(s["gz_dps"])
            mag_x.append(s["mx_uT"])
            mag_y.append(s["my_uT"])
            mag_z.append(s["mz_uT"])

            acc_l1.set_data(t, acc_x)
            acc_l2.set_data(t, acc_y)
            acc_l3.set_data(t, acc_z)
            gyr_l1.set_data(t, gyr_x)
            gyr_l2.set_data(t, gyr_y)
            gyr_l3.set_data(t, gyr_z)
            mag_l1.set_data(t, mag_x)
            mag_l2.set_data(t, mag_y)
            mag_l3.set_data(t, mag_z)

            if len(t) > 2:
                ax_acc.set_xlim(max(0.0, t[-1] - WINDOW_SEC), t[-1])

            ax_acc.relim(); ax_acc.autoscale_view(scalex=False, scaley=True)
            ax_gyr.relim(); ax_gyr.autoscale_view(scalex=False, scaley=True)
            ax_mag.relim(); ax_mag.autoscale_view(scalex=False, scaley=True)

        return (
            color_artist,
            depth_artist,
            color_overlay,
            depth_overlay,
            ctrl_text,
            measurement_line,
            measurement_text,
            acc_l1,
            acc_l2,
            acc_l3,
            gyr_l1,
            gyr_l2,
            gyr_l3,
            mag_l1,
            mag_l2,
            mag_l3,
        )

    interval_ms = int(1000.0 / PLOT_HZ)
    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False)

    try:
        plt.show()
    finally:
        stop_event.set()
        decoder_thread.join(timeout=1)
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
