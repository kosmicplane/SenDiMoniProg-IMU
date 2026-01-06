#!/usr/bin/env python3
import time, json
import paho.mqtt.client as mqtt
import numpy as np
import cv2

import pyrealsense2 as rs  # comes from librealsense install

# ---------- MQTT ----------
BROKER_HOST = "test.mosquitto.org"
BROKER_PORT = 1883
MQTT_USER = ""      # optional
MQTT_PASS = ""      # optional

TOPIC_COLOR = "cam/jetson01/color_jpg"
TOPIC_DEPTH = "cam/jetson01/depth_png16"   # ✅ metric depth: uint16 PNG (Z16 in mm)
TOPIC_META  = "cam/jetson01/meta"

# ---------- Camera / publish tuning ----------
COLOR_W, COLOR_H, COLOR_FPS = 424, 240, 15
DEPTH_W, DEPTH_H, DEPTH_FPS = 424, 240, 15

PUB_HZ = 60
PUB_PERIOD = 1.0 / PUB_HZ
JPEG_QUALITY = 20 
PUBLISH_DEPTH = True          # ✅ publish metric depth
ALIGN_DEPTH_TO_COLOR = True   # keep aligned pixels for click->distance
# Optional: also publish a pretty depth preview (costs bandwidth)
PUBLISH_DEPTH_PREVIEW_JPG = False
TOPIC_DEPTH_PREVIEW = "cam/jetson01/depth_preview_jpg"

def mqtt_connect():
    cid = f"jetson01-cam-{int(time.time())}"
    c = mqtt.Client(client_id=cid, clean_session=True, protocol=mqtt.MQTTv311)
    if MQTT_USER:
        c.username_pw_set(MQTT_USER, MQTT_PASS)
    c.reconnect_delay_set(min_delay=1, max_delay=30)

    def on_connect(client, userdata, flags, rc):
        print("✅ MQTT connected" if rc == 0 else f"⚠️ MQTT connect rc={rc}", flush=True)

    def on_disconnect(client, userdata, rc):
        print(f"⚠️ MQTT disconnected rc={rc} (will retry)", flush=True)

    c.on_connect = on_connect
    c.on_disconnect = on_disconnect

    while True:
        try:
            c.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
            c.loop_start()
            return c
        except Exception as e:
            print(f"⚠️ MQTT retry: {e}", flush=True)
            time.sleep(2)


def encode_jpg(bgr: np.ndarray, quality: int):
    ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    return enc.tobytes()


def encode_png16(depth_u16: np.ndarray):
    """
    Encode uint16 depth image to PNG bytes (lossless, preserves depth).
    depth_u16 must be HxW uint16.
    """
    if depth_u16 is None or depth_u16.dtype != np.uint16:
        return None
    ok, enc = cv2.imencode(".png", depth_u16)
    if not ok:
        return None
    return enc.tobytes()


def main():
    client = mqtt_connect()

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
    if PUBLISH_DEPTH:
        config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, DEPTH_FPS)

    profile = pipeline.start(config)

    align = rs.align(rs.stream.color) if (PUBLISH_DEPTH and ALIGN_DEPTH_TO_COLOR) else None
    colorizer = rs.colorizer()

    # Grab depth scale + intrinsics for meta (nice for converting + 3D)
    depth_scale = None
    intr = None
    try:
        if PUBLISH_DEPTH:
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = float(depth_sensor.get_depth_scale())  # meters per unit
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
    except Exception:
        pass

    seq = 0
    last_pub = time.monotonic()

    print("✅ RealSense streaming started", flush=True)

    try:
        while True:
            frames = pipeline.wait_for_frames()

            if align is not None:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())  # BGR8 already

            depth_png16 = None
            depth_preview_jpg = None

            if PUBLISH_DEPTH:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    # ✅ Metric depth: uint16 mm (Z16). Values are in "depth units".
                    depth_u16 = np.asanyarray(depth_frame.get_data()).astype(np.uint16)

                    # Many pipelines use mm-like units, but RealSense provides depth_scale.
                    # We'll keep raw units as uint16 and share depth_scale in meta.
                    depth_png16 = encode_png16(depth_u16)

                    if PUBLISH_DEPTH_PREVIEW_JPG:
                        depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())  # RGB
                        depth_bgr = cv2.cvtColor(depth_color, cv2.COLOR_RGB2BGR)
                        depth_preview_jpg = encode_jpg(depth_bgr, JPEG_QUALITY)

            now = time.monotonic()
            if (now - last_pub) >= PUB_PERIOD:
                last_pub = now

                color_jpg = encode_jpg(color, JPEG_QUALITY)
                if color_jpg is None:
                    continue

                # Publish color
                client.publish(TOPIC_COLOR, payload=color_jpg, qos=0, retain=False)

                # Publish metric depth (PNG16)
                if PUBLISH_DEPTH and depth_png16 is not None:
                    client.publish(TOPIC_DEPTH, payload=depth_png16, qos=0, retain=False)

                # Optional preview
                if PUBLISH_DEPTH_PREVIEW_JPG and depth_preview_jpg is not None:
                    client.publish(TOPIC_DEPTH_PREVIEW, payload=depth_preview_jpg, qos=0, retain=False)

                meta = {
                    "seq": seq,
                    "t_wall": time.time(),
                    "w": COLOR_W, "h": COLOR_H,
                    "pub_hz": PUB_HZ,

                    "color_bytes": len(color_jpg),
                    "depth_bytes": (len(depth_png16) if depth_png16 else 0),

                    "depth_encoding": "png16_z16_units",
                    "depth_scale_m_per_unit": depth_scale,

                    # Intrinsics for true XYZ later (optional but useful)
                    "fx": (intr.fx if intr else None),
                    "fy": (intr.fy if intr else None),
                    "ppx": (intr.ppx if intr else None),
                    "ppy": (intr.ppy if intr else None),
                }
                client.publish(TOPIC_META, json.dumps(meta), qos=0, retain=False)

                seq += 1

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try:
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
