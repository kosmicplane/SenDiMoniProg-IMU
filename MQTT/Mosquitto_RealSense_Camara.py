#!/usr/bin/env python3
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
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
TOPIC_IMU = "cam/jetson01/imu"

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
    "publish_depth_z16": False,
    "imu_enabled": True,
    "imu_hz": 200,
    "record_enabled": False,
    "streaming_enabled": True,
}

VALID_PUB_HZ = {5, 10, 15, 20, 30, 45, 60}
VALID_JPEG_RANGE = (5, 95)
VALID_IMU_HZ = (50, 400)


@dataclass
class Recorder:
    output_dir: Optional[Path] = None
    color_video: Optional[cv2.VideoWriter] = None
    imu_log: Optional[Path] = None
    meta_log: Optional[Path] = None

    def start(self, color_w: int, color_h: int, color_fps: int) -> None:
        self.output_dir = Path(__file__).parent / "recordings" / time.strftime("%Y%m%d_%H%M%S")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        color_path = self.output_dir / "color.mp4"
        self.color_video = cv2.VideoWriter(
            str(color_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(color_fps),
            (int(color_w), int(color_h)),
        )
        self.imu_log = self.output_dir / "imu.jsonl"
        self.meta_log = self.output_dir / "meta.jsonl"

    def stop(self) -> None:
        if self.color_video is not None:
            self.color_video.release()
        self.color_video = None
        self.output_dir = None

    def write_color(self, frame: np.ndarray) -> None:
        if self.color_video is None:
            return
        self.color_video.write(frame)

    def write_depth(self, depth_png: Optional[bytes], ts: float) -> None:
        if self.output_dir is None or depth_png is None:
            return
        depth_path = self.output_dir / f"depth_{ts:.6f}.png"
        depth_path.write_bytes(depth_png)

    def write_imu(self, payload: Dict[str, Any]) -> None:
        if self.imu_log is None:
            return
        with self.imu_log.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload) + "\n")

    def write_meta(self, payload: Dict[str, Any]) -> None:
        if self.meta_log is None:
            return
        with self.meta_log.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload) + "\n")


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
        "w": intr.width,
        "h": intr.height,
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

    depth_enabled = config["publish_depth_preview"] or config["publish_depth_z16"]
    if depth_enabled:
        cfg.enable_stream(
            rs.stream.depth,
            int(config["depth_w"]),
            int(config["depth_h"]),
            rs.format.z16,
            int(config["depth_fps"]),
        )

    if config["imu_enabled"]:
        cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, int(config["imu_hz"]))
        cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, int(config["imu_hz"]))

    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color) if depth_enabled else None
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale() if depth_enabled else None
    intrinsics = get_intrinsics(profile)
    model = profile.get_device().get_info(rs.camera_info.name)

    return pipeline, align, depth_scale, intrinsics, model


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
        if "publish_depth_z16" in update or "publish_depth_raw" in update:
            new_val = bool(update.get("publish_depth_z16", update.get("publish_depth_raw", False)))
            if new_val != config["publish_depth_z16"]:
                config["publish_depth_z16"] = new_val
                restart_needed = True
        if "imu_enabled" in update:
            new_val = bool(update["imu_enabled"])
            if new_val != config["imu_enabled"]:
                config["imu_enabled"] = new_val
                restart_needed = True
        if "imu_hz" in update:
            new_val = clamp_int(update["imu_hz"], config["imu_hz"], *VALID_IMU_HZ)
            if new_val != config["imu_hz"]:
                config["imu_hz"] = new_val
                restart_needed = True
        if "record_enabled" in update:
            config["record_enabled"] = bool(update["record_enabled"])
        if "streaming_enabled" in update:
            new_val = bool(update["streaming_enabled"])
            if new_val != config["streaming_enabled"]:
                config["streaming_enabled"] = new_val
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
    device_model = None
    colorizer = rs.colorizer()
    pub_fps = 0.0
    pub_count = 0
    fps_start = time.monotonic()

    seq = 0
    imu_seq = 0
    last_pub = time.monotonic()
    last_imu_pub = time.monotonic()
    last_accel = None
    last_gyro = None
    recorder = Recorder()

    try:
        while True:
            if pipeline is None or restart_event.is_set():
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
                if recorder.output_dir is not None:
                    recorder.stop()
                restart_event.clear()
                with config_lock:
                    pipeline, align, depth_scale, intrinsics, device_model = start_pipeline(config)
                status_event.set()
                if intrinsics is not None:
                    client.publish(TOPIC_CALIB, json.dumps({
                        "fx": intrinsics["fx"],
                        "fy": intrinsics["fy"],
                        "ppx": intrinsics["ppx"],
                        "ppy": intrinsics["ppy"],
                        "depth_scale": depth_scale,
                        "aligned_to_color": True,
                        "w": intrinsics["w"],
                        "h": intrinsics["h"],
                        "model": device_model,
                        "intrinsics": intrinsics,
                        "depth_scale": depth_scale,
                    }), qos=0, retain=False)
                print("✅ RealSense streaming started", flush=True)

            if status_event.is_set():
                status_event.clear()
                with config_lock:
                    status_payload = {
                        "ok": True,
                        "applied": config.copy(),
                        "errors": [],
                        "config": config.copy(),
                        "intrinsics": intrinsics,
                        "depth_scale": depth_scale,
                        "t_wall": time.time(),
                    }
                client.publish(TOPIC_STATUS, json.dumps(status_payload), qos=0, retain=False)

            with config_lock:
                cfg_snapshot = config.copy()

            if not cfg_snapshot["streaming_enabled"]:
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
                    pipeline = None
                    print("⏸ Streaming paused", flush=True)
                if recorder.output_dir is not None:
                    recorder.stop()
                time.sleep(0.1)
                continue

            if pipeline is None:
                continue

            frames = pipeline.wait_for_frames()
            with config_lock:
                cfg_snapshot = config.copy()

            depth_enabled = cfg_snapshot["publish_depth_preview"] or cfg_snapshot["publish_depth_z16"]
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
                if cfg_snapshot["publish_depth_z16"]:
                    depth_raw = np.asanyarray(depth_frame.get_data())
                    depth_raw_png = encode_png_16u(depth_raw)

            if cfg_snapshot["imu_enabled"]:
                accel_frame = frames.first_or_default(rs.stream.accel)
                gyro_frame = frames.first_or_default(rs.stream.gyro)
                if accel_frame:
                    last_accel = accel_frame.as_motion_frame().get_motion_data()
                if gyro_frame:
                    last_gyro = gyro_frame.as_motion_frame().get_motion_data()

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

                if cfg_snapshot["publish_depth_z16"] and depth_raw_png is not None:
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
                    "publish_depth_z16": cfg_snapshot["publish_depth_z16"],
                    "publish_depth_raw": cfg_snapshot["publish_depth_z16"],
                    "imu_enabled": cfg_snapshot["imu_enabled"],
                    "imu_hz": cfg_snapshot["imu_hz"],
                    "record_enabled": cfg_snapshot["record_enabled"],
                    "streaming_enabled": cfg_snapshot["streaming_enabled"],
                    "intrinsics": intrinsics,
                    "depth_scale": depth_scale,
                }
                client.publish(TOPIC_META, json.dumps(meta), qos=0, retain=False)
                if cfg_snapshot["record_enabled"]:
                    if recorder.output_dir is None:
                        recorder.start(cfg_snapshot["color_w"], cfg_snapshot["color_h"], cfg_snapshot["color_fps"])
                    recorder.write_color(color)
                    recorder.write_depth(depth_raw_png, time.time())
                    recorder.write_meta(meta)
                elif recorder.output_dir is not None:
                    recorder.stop()

                seq += 1

            imu_period = 1.0 / max(1, cfg_snapshot["imu_hz"])
            if cfg_snapshot["imu_enabled"] and last_accel and last_gyro and (now - last_imu_pub) >= imu_period:
                last_imu_pub = now
                imu_payload = {
                    "t_wall": time.time(),
                    "seq": imu_seq,
                    "accel": {"x": last_accel.x, "y": last_accel.y, "z": last_accel.z},
                    "gyro": {"x": last_gyro.x, "y": last_gyro.y, "z": last_gyro.z},
                    "imu_hz": cfg_snapshot["imu_hz"],
                }
                client.publish(TOPIC_IMU, json.dumps(imu_payload), qos=0, retain=False)
                if cfg_snapshot["record_enabled"]:
                    recorder.write_imu(imu_payload)
                imu_seq += 1

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try:
            if pipeline is not None:
                pipeline.stop()
        except Exception:
            pass
        try:
            recorder.stop()
        except Exception:
            pass
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
