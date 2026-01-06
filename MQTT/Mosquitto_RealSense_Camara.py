#!/usr/bin/env python3
import time, json
import paho.mqtt.client as mqtt
import numpy as np
import cv2
import pyrealsense2 as rs  # from librealsense

# ---------- MQTT ----------
BROKER_HOST = "test.mosquitto.org"
BROKER_PORT = 1883
MQTT_USER = ""      # optional
MQTT_PASS = ""      # optional

TOPIC_COLOR = "cam/jetson01/color_jpg"
TOPIC_DEPTH16 = "cam/jetson01/depth_png16"  # ✅ raw depth for metric distance
TOPIC_META  = "cam/jetson01/meta"

# ---------- Camera / publish tuning ----------
COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 15
DEPTH_W, DEPTH_H, DEPTH_FPS = 640, 480, 15

PUB_HZ = 20
PUB_PERIOD = 1.0 / PUB_HZ
JPEG_QUALITY = 20  # lower => smaller packets

PUBLISH_DEPTH = True  # ✅ must be True for distances

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

def encode_png16(u16: np.ndarray):
    """
    Encode uint16 depth image (HxW) to PNG preserving 16-bit values.
    IMPORTANT: Do NOT colorize. This is for metric distance.
    """
    if u16 is None or u16.dtype != np.uint16:
        return None
    ok, enc = cv2.imencode(".png", u16)
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

    # Align depth to color so (x,y) matches between color and depth
    align = rs.align(rs.stream.color)

    # Depth scale (meters per unit)
    depth_scale = None
    if PUBLISH_DEPTH:
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        print(f"✅ depth_scale_m_per_unit = {depth_scale}", flush=True)

    # Get intrinsics from color stream (used by PC if needed later)
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()

    seq = 0
    last_pub = time.monotonic()

    print("✅ RealSense streaming started", flush=True)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            if PUBLISH_DEPTH:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())  # BGR8
            color_jpg = encode_jpg(color, JPEG_QUALITY)
            if color_jpg is None:
                continue

            depth_png16 = None
            depth_frame = None
            if PUBLISH_DEPTH:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_u16 = np.asanyarray(depth_frame.get_data())  # uint16 Z16
                    depth_png16 = encode_png16(depth_u16)

            now = time.monotonic()
            if (now - last_pub) >= PUB_PERIOD:
                last_pub = now

                # Publish COLOR
                client.publish(TOPIC_COLOR, payload=color_jpg, qos=0, retain=False)

                # Publish DEPTH (raw 16-bit)
                if PUBLISH_DEPTH and depth_png16 is not None:
                    client.publish(TOPIC_DEPTH16, payload=depth_png16, qos=0, retain=False)

                # Publish META (scale + intrinsics)
                meta = {
                    "seq": seq,
                    "t_wall": time.time(),
                    "pub_hz": PUB_HZ,
                    "w": COLOR_W,
                    "h": COLOR_H,

                    # ✅ This is what makes "distance in meters" possible on the PC
                    "depth_scale_m_per_unit": depth_scale,

                    # Intrinsics (optional but good to have)
                    "fx": intr.fx,
                    "fy": intr.fy,
                    "ppx": intr.ppx,
                    "ppy": intr.ppy,

                    # Sizes (helpful for debugging)
                    "color_bytes": len(color_jpg),
                    "depth_png16_bytes": (len(depth_png16) if depth_png16 else 0),
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

