#!/usr/bin/env python3
import time, json
import paho.mqtt.client as mqtt
import numpy as np
import cv2

import pyrealsense2 as rs  # comes from librealsense install

# ---------- MQTT ----------
BROKER_HOST = "test.mosquitto.org"
#BROKER_HOST = "test.mosquitto.org"
BROKER_PORT = 1883
MQTT_USER = ""      # optional
MQTT_PASS = ""      # optional

TOPIC_COLOR = "cam/jetson01/color_jpg"
TOPIC_DEPTH = "cam/jetson01/depth_jpg"   # optional
TOPIC_META  = "cam/jetson01/meta"

# ---------- Camera / publish tuning ----------
COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 15
DEPTH_W, DEPTH_H, DEPTH_FPS = 640, 480, 15

PUB_HZ = 10.0                  # publish rate over LTE
PUB_PERIOD = 1.0 / PUB_HZ
JPEG_QUALITY = 70              # lower => smaller packets

PUBLISH_DEPTH_PREVIEW = True   # set False if you only need color

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

def main():
    client = mqtt_connect()

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
    if PUBLISH_DEPTH_PREVIEW:
        config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, DEPTH_FPS)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)  # align depth to color if depth enabled
    colorizer = rs.colorizer()

    seq = 0
    last_pub = time.monotonic()

    print("✅ RealSense streaming started", flush=True)

    try:
        while True:
            frames = pipeline.wait_for_frames()

            if PUBLISH_DEPTH_PREVIEW:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())  # BGR8 already

            depth_jpg = None
            if PUBLISH_DEPTH_PREVIEW:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())  # RGB
                    depth_color = cv2.cvtColor(depth_color, cv2.COLOR_RGB2BGR)
                    depth_jpg = encode_jpg(depth_color, JPEG_QUALITY)

            now = time.monotonic()
            if (now - last_pub) >= PUB_PERIOD:
                last_pub = now

                color_jpg = encode_jpg(color, JPEG_QUALITY)
                if color_jpg is None:
                    continue

                # Publish binary JPEG payloads
                client.publish(TOPIC_COLOR, payload=color_jpg, qos=0, retain=False)

                if PUBLISH_DEPTH_PREVIEW and depth_jpg is not None:
                    client.publish(TOPIC_DEPTH, payload=depth_jpg, qos=0, retain=False)

                meta = {
                    "seq": seq,
                    "t_wall": time.time(),
                    "color_bytes": len(color_jpg),
                    "depth_bytes": (len(depth_jpg) if depth_jpg else 0),
                    "w": COLOR_W, "h": COLOR_H,
                    "pub_hz": PUB_HZ,
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
