#!/usr/bin/env python3
import json
import threading
import time
from typing import Any, Dict, Optional

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import pyrealsense2 as rs  # comes from librealsense install

# ---------- MQTT ----------
BROKER_HOST = "test.mosquitto.org"
BROKER_PORT = 1883
MQTT_USER = ""      # optional
MQTT_PASS = ""      # optional

TOPIC_COLOR = "cam/jetson01/color_jpg"
TOPIC_DEPTH = "cam/jetson01/depth_jpg"          # optional preview
TOPIC_DEPTH_RAW = "cam/jetson01/depth_z16_png"  # aligned 16-bit PNG
TOPIC_META = "cam/jetson01/meta"
TOPIC_CONTROL = "cam/jetson01/control"
TOPIC_STATUS = "cam/jetson01/status"
TOPIC_CALIB = "cam/jetson01/calib"

# ---------- Camera / publish tuning ----------
DEFAULT_CONFIG: Dict[str, Any] = {
    "color_w": 424,
    "color_h": 240,
    "color_fps": 30,
    "depth_w": 424,
    "depth_h": 240,
    "depth_fps": 30,
    "pub_hz": 20,
    "jpeg_quality": 20,
    "publish_depth_preview": True,
    "publish_depth_raw": False,
}

VALID_PUB_HZ = {5, 10, 15, 20, 30, 45, 60}
VALID_JPEG_RANGE = (5, 95)


def mqtt_connect() -> mqtt.Client:
    cid = f"jetson01-cam-{int(time.time())}"
    c = mqtt.Client(client_id=cid, clean_session=True, protocol=mqtt.MQTTv311)
    if MQTT_USER:
        c.username_pw_set(MQTT_USER, MQTT_PASS)
    c.reconnect_delay_set(min_delay=1, max_delay=30)
    c.max_inflight_messages_set(20)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT connected", flush=True)
            client.subscribe(TOPIC_CONTROL, qos=0)
            print(f"✅ Subscribed control: {TOPIC_CONTROL}", flush=True)
        else:
            print(f"⚠️ MQTT connect rc={rc}", flush=True)

    def on_disconnect(client, userdata, rc):
        print(f"⚠️ MQTT disconnected rc={rc} (will retry)", flush=True)

    c.on_connect = on_connect
    c.on_disconnect = on_disconnect

    while True:
        try:
            c.connect(BROKER_HOST, BROKER_PORT, keepalive=20)
            c.loop_start()
            return c
        except Exception as e:
            print(f"⚠️ MQTT retry: {e}", flush=True)
            time.sleep(2)


def encode_jpg(bgr: np.ndarray, quality: int) -> Optional[bytes]:
    ok, enc = cv2.imencode(
        ".jpg",
        bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )
    if not ok:
        return None
    return enc.tobytes()


def encode_png_16u(depth_u16: np.ndarray) -> Optional[bytes]:
    ok, enc = cv2.imencode(
        ".png",
        depth_u16,
        [int(cv2.IMWRITE_PNG_COMPRESSION), 1],
    )
    if not ok:
        return None
    return enc.tobytes()


def get_intrinsics(profile: rs.pipeline_profile) -> Dict[str, float]:
    stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = stream_profile.get_intrinsics()
    return {
        "fx": intr.fx,
        "fy": intr.fy,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
        "width": intr.width,
        "height": intr.height,
    }


def start_pipeline(config: Dict[str, Any]):
    pipeline = rs.pipeline()
    cfg = rs.config()

    cfg.enable_stream(
        rs.stream.color,
        int(config["color_w"]),
        int(config["color_h"]),
        rs.format.bgr8,
        int(config["color_fps"]),
    )

    depth_enabled = config["publish_depth_preview"] or config["publish_depth_raw"]
    if depth_enabled:
        cfg.enable_stream(
            rs.stream.depth,
            int(config["depth_w"]),
            int(config["depth_h"]),
            rs.format.z16,
            int(config["depth_fps"]),
        )

    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color) if depth_enabled else None
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale() if depth_enabled else None
    intrinsics = get_intrinsics(profile)

    return pipeline, align, depth_scale, intrinsics


def clamp_int(value: Any, default: int, min_value: int, max_value: int) -> int:
    try:
        value_int = int(value)
    except (TypeError, ValueError):
        return default
    return max(min_value, min(value_int, max_value))


def apply_control(update: Dict[str, Any], config: Dict[str, Any], lock: threading.Lock) -> bool:
    restart_needed = False
    with lock:
        if "jpeg_quality" in update:
            config["jpeg_quality"] = clamp_int(update["jpeg_quality"], config["jpeg_quality"], *VALID_JPEG_RANGE)
        if "pub_hz" in update:
            new_hz = clamp_int(update["pub_hz"], config["pub_hz"], 1, 60)
            if new_hz not in VALID_PUB_HZ:
                new_hz = config["pub_hz"]
            config["pub_hz"] = new_hz
        for key in ("color_w", "color_h", "color_fps", "depth_w", "depth_h", "depth_fps"):
            if key in update:
                new_val = clamp_int(update[key], config[key], 160, 1920)
                if new_val != config[key]:
                    config[key] = new_val
                    restart_needed = True
        if "publish_depth_preview" in update:
            new_val = bool(update["publish_depth_preview"])
            if new_val != config["publish_depth_preview"]:
                config["publish_depth_preview"] = new_val
                restart_needed = True
        if "publish_depth_raw" in update:
            new_val = bool(update["publish_depth_raw"])
            if new_val != config["publish_depth_raw"]:
                config["publish_depth_raw"] = new_val
                restart_needed = True
    return restart_needed


def main():
    config = DEFAULT_CONFIG.copy()
    config_lock = threading.Lock()
    restart_event = threading.Event()
    status_event = threading.Event()

    client = mqtt_connect()

    def on_message(client, userdata, msg):
        if msg.topic != TOPIC_CONTROL:
            return
        try:
            payload = msg.payload.decode("utf-8")
            update = json.loads(payload)
            if not isinstance(update, dict):
                return
            needs_restart = apply_control(update, config, config_lock)
            status_event.set()
            if needs_restart:
                restart_event.set()
        except Exception:
            return

    client.on_message = on_message

    pipeline = None
    align = None
    depth_scale = None
    intrinsics = None
    colorizer = rs.colorizer()
    pub_fps = 0.0
    pub_count = 0
    fps_start = time.monotonic()

    seq = 0
    last_pub = time.monotonic()

    try:
        while True:
            if pipeline is None or restart_event.is_set():
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
                restart_event.clear()
                with config_lock:
                    pipeline, align, depth_scale, intrinsics = start_pipeline(config)
                status_event.set()
                if intrinsics is not None:
                    client.publish(TOPIC_CALIB, json.dumps({
                        "intrinsics": intrinsics,
                        "depth_scale": depth_scale,
                        "aligned_to_color": True,
                    }), qos=0, retain=False)
                print("✅ RealSense streaming started", flush=True)

            if status_event.is_set():
                status_event.clear()
                with config_lock:
                    status_payload = {
                        "config": config.copy(),
                        "intrinsics": intrinsics,
                        "depth_scale": depth_scale,
                        "t_wall": time.time(),
                    }
                client.publish(TOPIC_STATUS, json.dumps(status_payload), qos=0, retain=False)

            frames = pipeline.wait_for_frames()
            with config_lock:
                cfg_snapshot = config.copy()

            depth_enabled = cfg_snapshot["publish_depth_preview"] or cfg_snapshot["publish_depth_raw"]
            if depth_enabled and align is not None:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color = np.asanyarray(color_frame.get_data())  # BGR8

            depth_frame = frames.get_depth_frame() if depth_enabled else None
            depth_jpg = None
            depth_raw_png = None
            if depth_frame:
                if cfg_snapshot["publish_depth_preview"]:
                    depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                    depth_color = cv2.cvtColor(depth_color, cv2.COLOR_RGB2BGR)
                    depth_jpg = encode_jpg(depth_color, cfg_snapshot["jpeg_quality"])
                if cfg_snapshot["publish_depth_raw"]:
                    depth_raw = np.asanyarray(depth_frame.get_data())
                    depth_raw_png = encode_png_16u(depth_raw)

            now = time.monotonic()
            pub_period = 1.0 / max(1, cfg_snapshot["pub_hz"])
            if (now - last_pub) >= pub_period:
                last_pub = now
                color_jpg = encode_jpg(color, cfg_snapshot["jpeg_quality"])
                if color_jpg is None:
                    continue

                client.publish(TOPIC_COLOR, payload=color_jpg, qos=0, retain=False)

                if cfg_snapshot["publish_depth_preview"] and depth_jpg is not None:
                    client.publish(TOPIC_DEPTH, payload=depth_jpg, qos=0, retain=False)

                if cfg_snapshot["publish_depth_raw"] and depth_raw_png is not None:
                    client.publish(TOPIC_DEPTH_RAW, payload=depth_raw_png, qos=0, retain=False)

                pub_count += 1
                if (now - fps_start) >= 1.0:
                    pub_fps = pub_count / (now - fps_start)
                    pub_count = 0
                    fps_start = now

                meta = {
                    "seq": seq,
                    "t_wall": time.time(),
                    "color_bytes": len(color_jpg),
                    "depth_bytes": (len(depth_jpg) if depth_jpg else 0),
                    "depth_raw_bytes": (len(depth_raw_png) if depth_raw_png else 0),
                    "w": cfg_snapshot["color_w"],
                    "h": cfg_snapshot["color_h"],
                    "pub_hz": cfg_snapshot["pub_hz"],
                    "pub_fps": pub_fps,
                    "jpeg_quality": cfg_snapshot["jpeg_quality"],
                    "publish_depth_preview": cfg_snapshot["publish_depth_preview"],
                    "publish_depth_raw": cfg_snapshot["publish_depth_raw"],
                    "intrinsics": intrinsics,
                    "depth_scale": depth_scale,
                }
                client.publish(TOPIC_META, json.dumps(meta), qos=0, retain=False)

                seq += 1

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try:
            if pipeline is not None:
                pipeline.stop()
        except Exception:
            pass
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
