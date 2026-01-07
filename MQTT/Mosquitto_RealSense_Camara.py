#!/usr/bin/env python3
import json
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import pyrealsense2 as rs  # comes from librealsense install

from frame_codec import KIND_COLOR, KIND_DEPTH, pack_frame

# ---------- MQTT ----------
BROKER_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")

TOPIC_COLOR = "cam/jetson01/color_mat"
TOPIC_DEPTH = "cam/jetson01/depth_mat"
TOPIC_META = "cam/jetson01/meta"
TOPIC_CONTROL = "cam/jetson01/control"
TOPIC_STATUS = "cam/jetson01/status"
TOPIC_CALIB = "cam/jetson01/calib"

USE_LZ4 = os.getenv("RSF_LZ4", "0") == "1"

# ---------- Camera / publish tuning ----------
DEFAULT_CONFIG: Dict[str, Any] = {
    "color_w": 424,
    "color_h": 240,
    "color_fps": 30,
    "depth_w": 424,
    "depth_h": 240,
    "depth_fps": 30,
    "pub_hz": 20,
    "record_enabled": False,
    "streaming_enabled": True,
}


@dataclass
class Recorder:
    output_dir: Optional[Path] = None
    color_video: Optional[cv2.VideoWriter] = None
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

    def write_depth(self, depth_raw: Optional[np.ndarray], ts: float) -> None:
        if self.output_dir is None or depth_raw is None:
            return
        depth_path = self.output_dir / f"depth_{ts:.6f}.npy"
        np.save(depth_path, depth_raw)

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

    cfg.enable_stream(
        rs.stream.depth,
        int(config["depth_w"]),
        int(config["depth_h"]),
        rs.format.z16,
        int(config["depth_fps"]),
    )

    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intrinsics = get_intrinsics(profile)
    model = profile.get_device().get_info(rs.camera_info.name)

    try:
        sensors = profile.get_device().query_sensors()
        for sensor in sensors:
            if sensor.supports(rs.option.frames_queue_size):
                sensor.set_option(rs.option.frames_queue_size, 1)
    except Exception:
        pass

    return pipeline, align, depth_scale, intrinsics, model


def clamp_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return default


def apply_control(update: Dict[str, Any], config: Dict[str, Any], lock: threading.Lock) -> bool:
    restart_needed = False
    with lock:
        if "record_enabled" in update:
            config["record_enabled"] = clamp_bool(update["record_enabled"], config["record_enabled"])
        if "streaming_enabled" in update:
            new_val = clamp_bool(update["streaming_enabled"], config["streaming_enabled"])
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
    pub_fps = 0.0
    pub_count = 0
    fps_start = time.monotonic()

    seq = 0
    last_pub = time.monotonic()
    recorder = Recorder()
    capture_thread = None
    capture_state = {"color": None, "depth": None, "t_cap_ns": 0}
    capture_lock = threading.Lock()
    capture_stop = threading.Event()

    def capture_loop() -> None:
        while not capture_stop.is_set() and pipeline is not None:
            try:
                frames = pipeline.wait_for_frames()
                if align is not None:
                    frames = align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                color = np.asanyarray(color_frame.get_data())
                depth_raw = np.asanyarray(depth_frame.get_data())
                t_cap_ns = time.time_ns()
                with capture_lock:
                    capture_state["color"] = color
                    capture_state["depth"] = depth_raw
                    capture_state["t_cap_ns"] = t_cap_ns
            except Exception:
                time.sleep(0.01)

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
                capture_stop.set()
                if capture_thread is not None:
                    capture_thread.join(timeout=1.0)
                restart_event.clear()
                with config_lock:
                    pipeline, align, depth_scale, intrinsics, device_model = start_pipeline(config)
                status_event.set()
                capture_stop.clear()
                capture_thread = threading.Thread(target=capture_loop, daemon=True)
                capture_thread.start()
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
                capture_stop.set()
                if capture_thread is not None:
                    capture_thread.join(timeout=1.0)
                    capture_thread = None
                print("⏸ Streaming paused", flush=True)
                if recorder.output_dir is not None:
                    recorder.stop()
                time.sleep(0.1)
                continue

            if pipeline is None:
                continue

            with capture_lock:
                color = capture_state["color"]
                depth_raw = capture_state["depth"]
                t_cap_ns = capture_state["t_cap_ns"]
            if color is None or depth_raw is None:
                time.sleep(0.005)
                continue

            now = time.monotonic()
            pub_period = 1.0 / max(1, cfg_snapshot["pub_hz"])
            if (now - last_pub) >= pub_period:
                last_pub = now

                color_payload = pack_frame(KIND_COLOR, seq, t_cap_ns, color, compress=USE_LZ4)
                depth_payload = pack_frame(KIND_DEPTH, seq, t_cap_ns, depth_raw, compress=USE_LZ4)

                client.publish(TOPIC_COLOR, payload=color_payload, qos=0, retain=False)
                client.publish(TOPIC_DEPTH, payload=depth_payload, qos=0, retain=False)

                pub_count += 1
                if (now - fps_start) >= 1.0:
                    pub_fps = pub_count / (now - fps_start)
                    pub_count = 0
                    fps_start = now

                meta = {
                    "seq": seq,
                    "t_wall": time.time(),
                    "t_cap_ns": t_cap_ns,
                    "color_bytes": len(color_payload),
                    "depth_bytes": len(depth_payload),
                    "color_w": cfg_snapshot["color_w"],
                    "color_h": cfg_snapshot["color_h"],
                    "depth_w": cfg_snapshot["depth_w"],
                    "depth_h": cfg_snapshot["depth_h"],
                    "pub_hz": cfg_snapshot["pub_hz"],
                    "pub_fps": pub_fps,
                    "model": device_model,
                    "record_enabled": cfg_snapshot["record_enabled"],
                    "streaming_enabled": cfg_snapshot["streaming_enabled"],
                    "intrinsics": intrinsics,
                    "depth_scale": depth_scale,
                    "lz4": USE_LZ4,
                }
                client.publish(TOPIC_META, json.dumps(meta), qos=0, retain=False)

                if cfg_snapshot["record_enabled"]:
                    if recorder.output_dir is None:
                        recorder.start(cfg_snapshot["color_w"], cfg_snapshot["color_h"], cfg_snapshot["color_fps"])
                    recorder.write_color(color)
                    recorder.write_depth(depth_raw, time.time())
                    recorder.write_meta(meta)
                elif recorder.output_dir is not None:
                    recorder.stop()

                seq += 1

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        capture_stop.set()
        if capture_thread is not None:
            capture_thread.join(timeout=1.0)
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
