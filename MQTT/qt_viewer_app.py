#!/usr/bin/env python3
"""High-performance Qt dashboard for Jetson + RealSense MQTT streams (PyQt6)."""
from __future__ import annotations

from collections import deque
import csv
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from PyQt6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
import cv2
import numpy as np
import paho.mqtt.client as mqtt

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt, pyqtSignal as Signal, pyqtSlot as Slot

# ---- Qt6 enum helpers ----
ALIGN_CENTER = Qt.AlignmentFlag.AlignCenter
ALIGN_RIGHT = Qt.AlignmentFlag.AlignRight
ALIGN_VCENTER = Qt.AlignmentFlag.AlignVCenter
ALIGN_LEFT = Qt.AlignmentFlag.AlignLeft
ALIGN_TOP = Qt.AlignmentFlag.AlignTop

KEEP_ASPECT = Qt.AspectRatioMode.KeepAspectRatio
SMOOTH_TRANSFORM = Qt.TransformationMode.SmoothTransformation
LEFT_BUTTON = Qt.MouseButton.LeftButton

AA_HINT = QtGui.QPainter.RenderHint.Antialiasing
FONT_BOLD = QtGui.QFont.Weight.Bold


BROKER_HOST = os.getenv("MQTT_HOST", "test.mosquitto.org")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")
CLIENT_ID_PREFIX = "qt-viewer"

TOPIC_COLOR = "cam/jetson01/color_raw"
TOPIC_DEPTH = "cam/jetson01/depth_raw"
TOPIC_META = "cam/jetson01/meta"
TOPIC_STATUS = "cam/jetson01/status"
TOPIC_CALIB = "cam/jetson01/calib"
TOPIC_IMU = "imu/jetson01/raw"
TOPIC_CONTROL = "cam/jetson01/control"

DEMO_MODE = os.getenv("DEMO_MODE", "0") == "1"

IMU_FIELDS = [
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "mx_uT", "my_uT", "mz_uT",
    "p_hpa", "t_C", "alt_m",
]

LOG_MAX_LINES = 400


def timestamp_now() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_imu_csv(line: str) -> Optional[Dict[str, Any]]:
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 12:
        return None
    try:
        values = list(map(float, parts))
    except ValueError:
        return None
    data = dict(zip(IMU_FIELDS, values))
    return {
        "accel": {"x": data["ax_g"], "y": data["ay_g"], "z": data["az_g"]},
        "gyro": {"x": data["gx_dps"], "y": data["gy_dps"], "z": data["gz_dps"]},
        "mag": {"x": data["mx_uT"], "y": data["my_uT"], "z": data["mz_uT"]},
        "pressure_hpa": data["p_hpa"],
        "temp_c": data["t_C"],
        "alt_m": data["alt_m"],
    }


@dataclass
class LatestFrames:
    color_image: Optional[QtGui.QImage] = None
    depth_image: Optional[QtGui.QImage] = None
    depth_raw: Optional[np.ndarray] = None
    color_bytes: int = 0
    depth_bytes: int = 0
    color_rx_fps: float = 0.0
    depth_rx_fps: float = 0.0
    last_color_ts: float = 0.0
    last_depth_ts: float = 0.0
    dropped_color: int = 0
    dropped_depth: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_color(
        self,
        image: QtGui.QImage,
        ts: float,
        payload_bytes: int,
        rx_fps: Optional[float] = None,
    ) -> None:
        with self.lock:
            if rx_fps is None and self.last_color_ts > 0:
                self.color_rx_fps = 1.0 / max(1e-6, ts - self.last_color_ts)
            elif rx_fps is not None:
                self.color_rx_fps = rx_fps
            self.last_color_ts = ts
            self.color_image = image
            self.color_bytes = payload_bytes

    def update_depth(
        self,
        depth_raw: np.ndarray,
        image: QtGui.QImage,
        ts: float,
        payload_bytes: int,
        rx_fps: Optional[float] = None,
    ) -> None:
        with self.lock:
            if rx_fps is None and self.last_depth_ts > 0:
                self.depth_rx_fps = 1.0 / max(1e-6, ts - self.last_depth_ts)
            elif rx_fps is not None:
                self.depth_rx_fps = rx_fps
            self.last_depth_ts = ts
            self.depth_raw = depth_raw
            self.depth_image = image
            self.depth_bytes = payload_bytes

    def snapshot(self) -> "LatestFrames":
        with self.lock:
            return LatestFrames(
                color_image=self.color_image,
                depth_image=self.depth_image,
                depth_raw=self.depth_raw.copy() if self.depth_raw is not None else None,
                color_bytes=self.color_bytes,
                depth_bytes=self.depth_bytes,
                color_rx_fps=self.color_rx_fps,
                depth_rx_fps=self.depth_rx_fps,
                last_color_ts=self.last_color_ts,
                last_depth_ts=self.last_depth_ts,
                dropped_color=self.dropped_color,
                dropped_depth=self.dropped_depth,
            )


@dataclass
class SharedState:
    meta: Dict[str, Any] = field(default_factory=dict)
    calib: Dict[str, Any] = field(default_factory=dict)
    status: Dict[str, Any] = field(default_factory=dict)
    imu: Dict[str, Any] = field(default_factory=dict)
    connected: bool = False
    last_meta_ts: float = 0.0
    last_status_ts: float = 0.0
    last_calib_ts: float = 0.0
    last_imu_ts: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, key: str, payload: Dict[str, Any]) -> None:
        now = time.time()
        with self.lock:
            if key == "meta":
                self.meta.update(payload)
                self.last_meta_ts = now
            elif key == "calib":
                self.calib.update(payload)
                self.last_calib_ts = now
            elif key == "status":
                self.status.update(payload)
                self.last_status_ts = now
            elif key == "imu":
                self.imu.update(payload)
                self.last_imu_ts = now

    def set_connected(self, connected: bool) -> None:
        with self.lock:
            self.connected = connected

    def snapshot(self) -> "SharedState":
        with self.lock:
            return SharedState(
                meta=dict(self.meta),
                calib=dict(self.calib),
                status=dict(self.status),
                imu=dict(self.imu),
                connected=self.connected,
                last_meta_ts=self.last_meta_ts,
                last_status_ts=self.last_status_ts,
                last_calib_ts=self.last_calib_ts,
                last_imu_ts=self.last_imu_ts,
            )


def make_qimage_from_rgb(rgb: np.ndarray) -> Optional[QtGui.QImage]:
    if rgb is None:
        return None
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        return None

    rgb = np.ascontiguousarray(rgb)
    h, w, _ = rgb.shape
    bytes_per_line = 3 * w

    img = QtGui.QImage(
        rgb.data, w, h, bytes_per_line,
        QtGui.QImage.Format.Format_RGB888
    )
    return img.copy()


def decode_color_raw(payload: bytes, width: int, height: int) -> Optional[QtGui.QImage]:
    if not payload or width <= 0 or height <= 0:
        return None
    expected = width * height * 3
    if len(payload) != expected:
        return None
    arr = np.frombuffer(payload, dtype=np.uint8)
    try:
        bgr = arr.reshape((height, width, 3))
    except ValueError:
        return None
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return make_qimage_from_rgb(rgb)


def decode_depth_raw(payload: bytes, width: int, height: int) -> Optional[np.ndarray]:
    if not payload or width <= 0 or height <= 0:
        return None
    expected = width * height
    arr = np.frombuffer(payload, dtype=np.uint16)
    if arr.size != expected:
        return None
    try:
        depth = arr.reshape((height, width))
    except ValueError:
        return None
    return depth


def depth_to_colormap(depth: np.ndarray) -> Optional[QtGui.QImage]:
    if depth is None:
        return None
    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
    depth_u8 = depth_norm.astype(np.uint8)
    colored = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
    rgb = cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)
    return make_qimage_from_rgb(rgb)


def demo_frames(t: float, width: int = 640, height: int = 480) -> Tuple[QtGui.QImage, QtGui.QImage, np.ndarray]:
    x = np.linspace(0, 1, width)
    y = np.linspace(0, 1, height)
    xv, yv = np.meshgrid(x, y)
    color = np.stack(
        [
            (xv * 255).astype(np.uint8),
            (yv * 255).astype(np.uint8),
            ((0.5 + 0.5 * np.sin(t)) * 255 * np.ones_like(xv)).astype(np.uint8),
        ],
        axis=-1,
    )
    depth_raw = ((xv + yv) * 1000 + 500).astype(np.uint16)
    depth_preview = depth_to_colormap(depth_raw)
    return make_qimage_from_rgb(color), depth_preview, depth_raw


def project_pixel_to_3d(u: int, v: int, depth_m: float, intr: Dict[str, float]) -> Tuple[float, float, float]:
    fx = float(intr.get("fx", 1.0))
    fy = float(intr.get("fy", 1.0))
    ppx = float(intr.get("ppx", 0.0))
    ppy = float(intr.get("ppy", 0.0))
    x = (u - ppx) / fx * depth_m
    y = (v - ppy) / fy * depth_m
    return x, y, depth_m


def compute_distance(
    p1: Tuple[int, int],
    p2: Tuple[int, int],
    depth: Optional[np.ndarray],
    intr: Dict[str, float],
    depth_scale: Optional[float],
) -> Tuple[Optional[float], str]:
    if depth is None or depth_scale is None or not intr:
        return None, "No depth/intrinsics"
    h, w = depth.shape[:2]
    (u1, v1), (u2, v2) = p1, p2
    if not (0 <= u1 < w and 0 <= v1 < h and 0 <= u2 < w and 0 <= v2 < h):
        return None, "Point out of range"
    z1 = float(depth[v1, u1]) * float(depth_scale)
    z2 = float(depth[v2, u2]) * float(depth_scale)
    if z1 <= 0 or z2 <= 0:
        return None, "No valid depth"
    x1, y1, z1 = project_pixel_to_3d(u1, v1, z1, intr)
    x2, y2, z2 = project_pixel_to_3d(u2, v2, z2, intr)
    dist = float(np.linalg.norm([x2 - x1, y2 - y1, z2 - z1]))
    return dist, ""


@dataclass
class LatestPayload:
    payload: Optional[bytes] = None
    ts: float = 0.0
    rx_fps: float = 0.0
    bytes_len: int = 0
    seq: int = 0
    unread: bool = False


class LatestPayloadBuffer:
    def __init__(self, topics: Tuple[str, ...]) -> None:
        self._payloads = {topic: LatestPayload() for topic in topics}
        self._overwritten = {topic: 0 for topic in topics}
        self._lock = threading.Lock()

    def update(self, topic: str, payload: bytes) -> None:
        now = time.monotonic()
        with self._lock:
            entry = self._payloads.setdefault(topic, LatestPayload())
            if entry.unread:
                self._overwritten[topic] = self._overwritten.get(topic, 0) + 1
            if entry.ts > 0:
                entry.rx_fps = 1.0 / max(1e-6, now - entry.ts)
            entry.ts = now
            entry.payload = payload
            entry.bytes_len = len(payload)
            entry.seq += 1
            entry.unread = True

    def take_if_new(self, topic: str, last_seq: int) -> Optional[Tuple[bytes, float, float, int, int]]:
        with self._lock:
            entry = self._payloads.get(topic)
            if entry is None or entry.payload is None or entry.seq <= last_seq:
                return None
            entry.unread = False
            return entry.payload, entry.ts, entry.rx_fps, entry.bytes_len, entry.seq

    def overwritten_counts(self) -> Dict[str, int]:
        with self._lock:
            return dict(self._overwritten)


class MqttClient(threading.Thread):
    def __init__(self, latest_buffer: LatestPayloadBuffer, state: SharedState, log_cb) -> None:
        super().__init__(daemon=True)
        self.latest_buffer = latest_buffer
        self.state = state
        self.log_cb = log_cb
        self._last_imu_ts = 0.0

        self.client = mqtt.Client(client_id=f"{CLIENT_ID_PREFIX}-{int(time.time())}", clean_session=True)
        if MQTT_USER:
            self.client.username_pw_set(MQTT_USER, MQTT_PASS)
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        self._stop_event = threading.Event()

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.log_cb("✅ MQTT connected")
            client.subscribe(
                [
                    (TOPIC_COLOR, 0),
                    (TOPIC_DEPTH, 0),
                    (TOPIC_META, 0),
                    (TOPIC_STATUS, 0),
                    (TOPIC_CALIB, 0),
                    (TOPIC_IMU, 0),
                ]
            )
            self.state.set_connected(True)
        else:
            self.log_cb(f"⚠️ MQTT connect failed rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        self.state.set_connected(False)
        self.log_cb(f"⚠️ MQTT disconnected rc={rc} (will retry)")

    def _on_message(self, client, userdata, msg):
        if msg.topic in {TOPIC_COLOR, TOPIC_DEPTH}:
            self.latest_buffer.update(msg.topic, msg.payload)
            return

        if msg.topic in {TOPIC_META, TOPIC_STATUS, TOPIC_CALIB}:
            try:
                payload = msg.payload.decode("utf-8", errors="ignore")
                data = json.loads(payload) if payload else {}
            except json.JSONDecodeError:
                return
            if not isinstance(data, dict):
                return
            if msg.topic == TOPIC_META:
                self.state.update("meta", data)
            elif msg.topic == TOPIC_STATUS:
                self.state.update("status", data)
            elif msg.topic == TOPIC_CALIB:
                self.state.update("calib", data)
            return

        if msg.topic == TOPIC_IMU:
            try:
                payload = msg.payload.decode("utf-8", errors="ignore").strip()
            except Exception:
                return
            if not payload:
                return
            parsed = parse_imu_csv(payload)
            if parsed is None:
                return
            now = time.time()
            imu_rate = 0.0
            if self._last_imu_ts > 0:
                imu_rate = 1.0 / max(1e-6, now - self._last_imu_ts)
            self._last_imu_ts = now
            parsed["t_wall"] = now
            parsed["imu_hz"] = imu_rate
            self.state.update("imu", parsed)

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.client.connect(BROKER_HOST, BROKER_PORT, keepalive=20)
                break
            except Exception as exc:
                self.log_cb(f"⚠️ MQTT retry: {exc}")
                time.sleep(2)

        self.client.loop_start()
        while not self._stop_event.is_set():
            time.sleep(0.2)
        self.client.loop_stop()
        try:
            self.client.disconnect()
        except Exception:
            pass

    def publish_control(self, payload: Dict[str, Any]) -> None:
        try:
            self.client.publish(TOPIC_CONTROL, json.dumps(payload), qos=0, retain=False)
        except Exception as exc:
            self.log_cb(f"⚠️ Publish failed: {exc}")

    def stop(self) -> None:
        self._stop_event.set()

    def reconnect(self) -> None:
        try:
            self.client.reconnect()
        except Exception as exc:
            self.log_cb(f"⚠️ Reconnect failed: {exc}")


class DecoderWorker(threading.Thread):
    def __init__(self, latest_buffer: LatestPayloadBuffer, frames: LatestFrames, state: SharedState, log_cb=None) -> None:
        super().__init__(daemon=True)
        self.latest_buffer = latest_buffer
        self.frames = frames
        self.state = state
        self.log_cb = log_cb
        self._stop_event = threading.Event()
        self._last_log = 0.0
        self._last_color_shape: Tuple[int, int] = (0, 0)
        self._last_depth_shape: Tuple[int, int] = (0, 0)
        self._last_color_seq = 0
        self._last_depth_seq = 0

    def _resolve_shapes(self) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        meta = self.state.snapshot().meta
        color_w = int(meta.get("color_w", meta.get("w", 0)) or 0)
        color_h = int(meta.get("color_h", meta.get("h", 0)) or 0)
        depth_w = int(meta.get("depth_w", meta.get("w", 0)) or 0)
        depth_h = int(meta.get("depth_h", meta.get("h", 0)) or 0)
        if color_w > 0 and color_h > 0:
            self._last_color_shape = (color_w, color_h)
        if depth_w > 0 and depth_h > 0:
            self._last_depth_shape = (depth_w, depth_h)
        return self._last_color_shape, self._last_depth_shape

    def run(self) -> None:
        while not self._stop_event.is_set():
            did_work = False
            (color_w, color_h), (depth_w, depth_h) = self._resolve_shapes()

            color_data = self.latest_buffer.take_if_new(TOPIC_COLOR, self._last_color_seq)
            if color_data is not None:
                payload, ts, rx_fps, bytes_len, seq = color_data
                image = decode_color_raw(payload, color_w, color_h)
                if image is not None:
                    self.frames.update_color(image, ts, bytes_len, rx_fps=rx_fps)
                    self._last_color_seq = seq
                else:
                    if self.log_cb and (time.monotonic() - self._last_log) > 2.0:
                        self._last_log = time.monotonic()
                        self.log_cb(f"❌ decode color failed (bytes={bytes_len})")
                    self._last_color_seq = seq
                did_work = True

            depth_data = self.latest_buffer.take_if_new(TOPIC_DEPTH, self._last_depth_seq)
            if depth_data is not None:
                payload, ts, rx_fps, bytes_len, seq = depth_data
                depth = decode_depth_raw(payload, depth_w, depth_h)
                if depth is not None:
                    depth_image = depth_to_colormap(depth)
                    if depth_image is not None:
                        self.frames.update_depth(depth, depth_image, ts, bytes_len, rx_fps=rx_fps)
                    self._last_depth_seq = seq
                else:
                    if self.log_cb and (time.monotonic() - self._last_log) > 2.0:
                        self._last_log = time.monotonic()
                        self.log_cb(f"❌ decode depth failed (bytes={bytes_len})")
                    self._last_depth_seq = seq
                did_work = True

            if not did_work:
                time.sleep(0.01)

    def stop(self) -> None:
        self._stop_event.set()


class VideoWidget(QtWidgets.QWidget):
    clicked = Signal(int, int)
    moved = Signal(int, int)

    def __init__(self, title: str) -> None:
        super().__init__()
        self.setMinimumSize(320, 240)
        self.setMouseTracking(True)

        self.title = title
        self.image: Optional[QtGui.QImage] = None
        self.overlay_lines: Tuple[str, ...] = ()
        self.measure_points: Tuple[Tuple[int, int], ...] = ()

        self._last_rect = QtCore.QRect()
        self._img_size = QtCore.QSize()

    def set_image(self, image: Optional[QtGui.QImage]) -> None:
        self.image = image
        self._img_size = image.size() if image is not None else QtCore.QSize()
        self.update()

    def set_overlay(self, lines: Tuple[str, ...]) -> None:
        self.overlay_lines = lines
        self.update()

    def set_measure_points(self, points: Tuple[Tuple[int, int], ...]) -> None:
        self.measure_points = points
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#111418"))
        rect = self.rect().adjusted(8, 8, -8, -8)

        if self.image is not None:
            img_rect = QtCore.QRect(QtCore.QPoint(0, 0), self.image.size())
            scaled = img_rect.size()
            scaled.scale(rect.size(), KEEP_ASPECT)

            target = QtCore.QRect(QtCore.QPoint(0, 0), scaled)
            target.moveCenter(rect.center())

            self._last_rect = target
            painter.drawImage(target, self.image)
        else:
            self._last_rect = rect
            painter.setPen(QtGui.QColor("#8a8f98"))
            painter.drawText(rect, ALIGN_CENTER, "No frame")

        painter.setPen(QtGui.QColor("#dfe6ee"))
        painter.setFont(QtGui.QFont("Inter", 10, FONT_BOLD))
        painter.drawText(rect.adjusted(6, 4, -6, -4), ALIGN_LEFT | ALIGN_TOP, self.title)

        painter.setFont(QtGui.QFont("Inter", 9))
        y_offset = 24
        for line in self.overlay_lines:
            painter.drawText(rect.adjusted(6, y_offset, -6, -4), ALIGN_LEFT | ALIGN_TOP, line)
            y_offset += 14

        if self.measure_points and self.image is not None:
            painter.setRenderHint(AA_HINT, True)
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffcc00"), 2))

            for point in self.measure_points:
                widget_pt = self._image_to_widget(point)
                if widget_pt is None:
                    continue
                painter.drawEllipse(widget_pt, 5, 5)

            if len(self.measure_points) == 2:
                p1 = self._image_to_widget(self.measure_points[0])
                p2 = self._image_to_widget(self.measure_points[1])
                if p1 is not None and p2 is not None:
                    painter.drawLine(p1, p2)

    def _image_to_widget(self, point: Tuple[int, int]) -> Optional[QtCore.QPoint]:
        if self._img_size.isEmpty() or self._last_rect.isNull():
            return None
        img_w, img_h = self._img_size.width(), self._img_size.height()
        if img_w <= 0 or img_h <= 0:
            return None

        scale_x = self._last_rect.width() / img_w
        scale_y = self._last_rect.height() / img_h

        x = int(self._last_rect.left() + point[0] * scale_x)
        y = int(self._last_rect.top() + point[1] * scale_y)
        return QtCore.QPoint(x, y)

    def _widget_to_image(self, pos: QtCore.QPoint) -> Optional[Tuple[int, int]]:
        if self._img_size.isEmpty() or self._last_rect.isNull():
            return None
        if not self._last_rect.contains(pos):
            return None

        img_w, img_h = self._img_size.width(), self._img_size.height()
        scale_x = img_w / max(1, self._last_rect.width())
        scale_y = img_h / max(1, self._last_rect.height())

        u = int((pos.x() - self._last_rect.left()) * scale_x)
        v = int((pos.y() - self._last_rect.top()) * scale_y)
        return u, v

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == LEFT_BUTTON:
            pt = self._widget_to_image(event.position().toPoint())
            if pt is not None:
                self.clicked.emit(pt[0], pt[1])

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        pt = self._widget_to_image(event.position().toPoint())
        if pt is not None:
            self.moved.emit(pt[0], pt[1])


class ImuPlotWidget(QtWidgets.QWidget):
    def __init__(self, parent=None, max_points=400, update_hz=15):
        super().__init__(parent)
        self.max_points = int(max_points)
        self.update_hz = int(update_hz)
        self._sample = 0

        self._last_redraw = 0.0
        self._no_data = True
        self._last_push = 0.0

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        self.title = QtWidgets.QLabel("IMU Plot — Accel, Gyro, Mag")
        self.title.setStyleSheet("font-size: 14px; font-weight: 700; color: #c9d1d9;")
        layout.addWidget(self.title)

        self.status = QtWidgets.QLabel("Waiting for IMU data…")
        self.status.setStyleSheet("font-size: 12px; color: #8b949e;")
        layout.addWidget(self.status)

        self.accel_chart = QChart()
        self.accel_chart.setTitle("Accel (g)")
        self.accel_chart.legend().setVisible(True)

        self.accel_series = {
            "ax": QLineSeries(),
            "ay": QLineSeries(),
            "az": QLineSeries(),
        }
        for k, s in self.accel_series.items():
            s.setName(k)
            self.accel_chart.addSeries(s)

        self.accel_axis_x = QValueAxis()
        self.accel_axis_x.setTitleText("Samples")
        self.accel_axis_x.setLabelFormat("%d")
        self.accel_axis_x.setRange(0, self.max_points)

        self.accel_axis_y = QValueAxis()
        self.accel_axis_y.setTitleText("Value")

        self.accel_chart.addAxis(self.accel_axis_x, Qt.AlignmentFlag.AlignBottom)
        self.accel_chart.addAxis(self.accel_axis_y, Qt.AlignmentFlag.AlignLeft)

        for s in self.accel_series.values():
            s.attachAxis(self.accel_axis_x)
            s.attachAxis(self.accel_axis_y)

        self.accel_view = QChartView(self.accel_chart)
        self.accel_view.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        self.gyro_chart = QChart()
        self.gyro_chart.setTitle("Gyro (dps)")
        self.gyro_chart.legend().setVisible(True)

        self.gyro_series = {
            "gx": QLineSeries(),
            "gy": QLineSeries(),
            "gz": QLineSeries(),
        }
        for k, s in self.gyro_series.items():
            s.setName(k)
            self.gyro_chart.addSeries(s)

        self.gyro_axis_x = QValueAxis()
        self.gyro_axis_x.setTitleText("Samples")
        self.gyro_axis_x.setLabelFormat("%d")
        self.gyro_axis_x.setRange(0, self.max_points)

        self.gyro_axis_y = QValueAxis()
        self.gyro_axis_y.setTitleText("Value")

        self.gyro_chart.addAxis(self.gyro_axis_x, Qt.AlignmentFlag.AlignBottom)
        self.gyro_chart.addAxis(self.gyro_axis_y, Qt.AlignmentFlag.AlignLeft)

        for s in self.gyro_series.values():
            s.attachAxis(self.gyro_axis_x)
            s.attachAxis(self.gyro_axis_y)

        self.gyro_view = QChartView(self.gyro_chart)
        self.gyro_view.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        self.mag_chart = QChart()
        self.mag_chart.setTitle("Mag (uT)")
        self.mag_chart.legend().setVisible(True)

        self.mag_series = {
            "mx": QLineSeries(),
            "my": QLineSeries(),
            "mz": QLineSeries(),
        }
        for k, s in self.mag_series.items():
            s.setName(k)
            self.mag_chart.addSeries(s)

        self.mag_axis_x = QValueAxis()
        self.mag_axis_x.setTitleText("Samples")
        self.mag_axis_x.setLabelFormat("%d")
        self.mag_axis_x.setRange(0, self.max_points)

        self.mag_axis_y = QValueAxis()
        self.mag_axis_y.setTitleText("Value")

        self.mag_chart.addAxis(self.mag_axis_x, Qt.AlignmentFlag.AlignBottom)
        self.mag_chart.addAxis(self.mag_axis_y, Qt.AlignmentFlag.AlignLeft)

        for s in self.mag_series.values():
            s.attachAxis(self.mag_axis_x)
            s.attachAxis(self.mag_axis_y)

        self.mag_view = QChartView(self.mag_chart)
        self.mag_view.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        layout.addWidget(self.accel_view, 1)
        layout.addWidget(self.gyro_view, 1)
        layout.addWidget(self.mag_view, 1)

        self._accel_vals = deque(maxlen=self.max_points)
        self._gyro_vals = deque(maxlen=self.max_points)
        self._mag_vals = deque(maxlen=self.max_points)

    def _append(self, series: QLineSeries, x: int, y: float):
        series.append(x, y)
        extra = series.count() - self.max_points
        if extra > 0:
            series.removePoints(0, extra)

    def _safe_float(self, value: Any) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(number):
            return 0.0
        return number

    def set_no_data(self, no_data: bool) -> None:
        if self._no_data == no_data:
            return
        self._no_data = no_data
        if no_data:
            self.status.setText("No IMU data (waiting…)")
            self.accel_chart.setTitle("Accel (no data)")
            self.gyro_chart.setTitle("Gyro (no data)")
            self.mag_chart.setTitle("Mag (no data)")
        else:
            self.status.setText("IMU streaming")
            self.accel_chart.setTitle("Accel (g)")
            self.gyro_chart.setTitle("Gyro (dps)")
            self.mag_chart.setTitle("Mag (uT)")

    def push(self, imu: Dict[str, Any]) -> None:
        now = time.monotonic()
        self._last_push = now
        if self.update_hz > 0 and (now - self._last_redraw) < (1.0 / self.update_hz):
            fast_only = True
        else:
            fast_only = False
            self._last_redraw = now

        accel = imu.get("accel", {}) if isinstance(imu.get("accel", {}), dict) else {}
        gyro = imu.get("gyro", {}) if isinstance(imu.get("gyro", {}), dict) else {}
        mag = imu.get("mag", {}) if isinstance(imu.get("mag", {}), dict) else {}

        ax = self._safe_float(accel.get("x", 0.0))
        ay = self._safe_float(accel.get("y", 0.0))
        az = self._safe_float(accel.get("z", 0.0))
        gx = self._safe_float(gyro.get("x", 0.0))
        gy = self._safe_float(gyro.get("y", 0.0))
        gz = self._safe_float(gyro.get("z", 0.0))
        mx = self._safe_float(mag.get("x", 0.0))
        my = self._safe_float(mag.get("y", 0.0))
        mz = self._safe_float(mag.get("z", 0.0))

        x = self._sample
        self._sample += 1

        self._append(self.accel_series["ax"], x, ax)
        self._append(self.accel_series["ay"], x, ay)
        self._append(self.accel_series["az"], x, az)

        self._append(self.gyro_series["gx"], x, gx)
        self._append(self.gyro_series["gy"], x, gy)
        self._append(self.gyro_series["gz"], x, gz)

        self._append(self.mag_series["mx"], x, mx)
        self._append(self.mag_series["my"], x, my)
        self._append(self.mag_series["mz"], x, mz)

        if fast_only:
            return

        x0 = max(0, x - self.max_points)
        self.accel_axis_x.setRange(x0, x)
        self.gyro_axis_x.setRange(x0, x)
        self.mag_axis_x.setRange(x0, x)

        self._accel_vals.extend([ax, ay, az])
        self._gyro_vals.extend([gx, gy, gz])
        self._mag_vals.extend([mx, my, mz])

        accel_vals = list(self._accel_vals) or [0.0]
        gyro_vals = list(self._gyro_vals) or [0.0]
        mag_vals = list(self._mag_vals) or [0.0]

        def pad(lo, hi, p=0.1):
            if lo == hi:
                return lo - 1.0, hi + 1.0
            span = hi - lo
            return lo - span * p, hi + span * p

        a_lo, a_hi = pad(min(accel_vals), max(accel_vals), 0.15)
        g_lo, g_hi = pad(min(gyro_vals), max(gyro_vals), 0.15)
        m_lo, m_hi = pad(min(mag_vals), max(mag_vals), 0.15)

        self.accel_axis_y.setRange(a_lo, a_hi)
        self.gyro_axis_y.setRange(g_lo, g_hi)
        self.mag_axis_y.setRange(m_lo, m_hi)


class DashboardWindow(QtWidgets.QMainWindow):

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Jetson RealSense Dashboard")
        self.setMinimumSize(1280, 720)
        self.resize(1440, 860)

        self.setStyleSheet(
            """
            QWidget { background-color: #0f1216; color: #e6edf3; font-family: 'Inter', 'Segoe UI', sans-serif; }
            QGroupBox { border: 1px solid #2a313b; border-radius: 6px; margin-top: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
            QLabel { color: #e6edf3; }
            QPushButton { background-color: #1f6feb; border: none; padding: 6px 12px; border-radius: 4px; }
            QPushButton:disabled { background-color: #2a313b; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background-color: #14181e; border: 1px solid #2a313b; padding: 4px; }
            QPlainTextEdit { background-color: #0d1117; border: 1px solid #2a313b; }
            QCheckBox::indicator { width: 14px; height: 14px; }
            """
        )

        self.frames = LatestFrames()
        self.state = SharedState()
        self.latest_buffer = LatestPayloadBuffer((TOPIC_COLOR, TOPIC_DEPTH))
        self.logs: list[str] = []
        self.decoder_thread = DecoderWorker(self.latest_buffer, self.frames, self.state, self.log)
        self.decoder_thread.start()

        self.mqtt_thread = MqttClient(self.latest_buffer, self.state, self.log)
        if not DEMO_MODE:
            self.mqtt_thread.start()

        self._measure_mode = False
        self._measure_points: list[Tuple[int, int]] = []
        self._cursor_depth = "--"
        self._distance_text = "--"
        self._last_imu_update = 0.0

        self.recording_active = False
        self.record_rows: list[Dict[str, Any]] = []
        self._last_record_ts = 0.0
        self.last_export_path: Optional[Path] = None

        self._build_ui()
        self._connect_signals()

        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_ui)
        self.refresh_timer.start(33)

        self.demo_timer = QtCore.QTimer(self)
        self.demo_timer.timeout.connect(self.demo_update)
        if DEMO_MODE:
            self.demo_timer.start(33)

    def _build_imu_plot_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("IMU Plot")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(10, 10, 10, 10)

        self.imu_plot = ImuPlotWidget(max_points=400, update_hz=15)
        self.imu_plot.setMinimumHeight(600)
        layout.addWidget(self.imu_plot)

        return group

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        header = QtWidgets.QHBoxLayout()
        header.setSpacing(12)

        logo_box = QtWidgets.QLabel()
        logo_box.setFixedSize(120, 60)
        logo_box.setAlignment(ALIGN_CENTER)
        logo_path = Path(__file__).parent / "assets" / "logo.png"
        if logo_path.exists():
            pix = QtGui.QPixmap(str(logo_path)).scaled(120, 60, KEEP_ASPECT, SMOOTH_TRANSFORM)
            logo_box.setPixmap(pix)
        else:
            logo_box.setText("LOGO")
            logo_box.setStyleSheet("border: 1px dashed #2a313b; color: #8b949e;")

        title_layout = QtWidgets.QVBoxLayout()
        self.title_label = QtWidgets.QLabel("Jetson + RealSense Streaming Dashboard")
        self.title_label.setStyleSheet("font-size: 20px; font-weight: 700;")
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #f85149; font-weight: 700;")
        title_layout.addWidget(self.title_label)
        title_layout.addWidget(self.status_label)

        right_header = QtWidgets.QVBoxLayout()
        self.quick_stats = QtWidgets.QLabel("Latency: N/A | RX FPS: -- | Overwritten: --")
        self.quick_stats.setAlignment(ALIGN_RIGHT | ALIGN_VCENTER)
        self.quick_stats.setStyleSheet("font-size: 14px; color: #c9d1d9;")
        self.recording_status = QtWidgets.QLabel("Recording: OFF")
        self.recording_status.setAlignment(ALIGN_RIGHT | ALIGN_VCENTER)
        self.recording_status.setStyleSheet("font-size: 13px; color: #f85149; font-weight: 700;")
        right_header.addWidget(self.quick_stats)
        right_header.addWidget(self.recording_status)

        header.addWidget(logo_box, 0)
        header.addLayout(title_layout, 1)
        header.addLayout(right_header, 0)

        root.addLayout(header)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)

        self.tabs.addTab(self._build_camera_tab(), "Camera")
        self.tabs.addTab(self._build_imu_tab(), "IMU")
        self.tabs.addTab(self._build_gps_tab(), "GPS")
        self.tabs.addTab(self._build_data_tab(), "Data / Logs")

        root.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

    def _build_camera_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        video_split = QtWidgets.QSplitter(Qt.Orientation.Vertical)
        video_split.setChildrenCollapsible(False)

        self.color_widget = VideoWidget("Color / Normal")
        self.depth_widget = VideoWidget("Depth")

        self.color_widget.setMinimumSize(640, 360)
        self.depth_widget.setMinimumSize(640, 300)

        video_split.addWidget(self.color_widget)
        video_split.addWidget(self.depth_widget)
        video_split.setStretchFactor(0, 3)
        video_split.setStretchFactor(1, 2)

        layout.addWidget(video_split, 1)

        controls_row = QtWidgets.QHBoxLayout()
        controls_row.addWidget(self._build_controls_group())
        controls_row.addWidget(self._build_measure_group())
        controls_row.addStretch(1)
        layout.addLayout(controls_row)

        return page

    def _build_imu_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(12)

        self.imu_plot_group = self._build_imu_plot_group()
        layout.addWidget(self.imu_plot_group, 3)

        imu_panel = QtWidgets.QGroupBox("IMU Live Values")
        imu_layout = QtWidgets.QVBoxLayout(imu_panel)
        imu_layout.setSpacing(10)

        self.imu_values: Dict[str, QtWidgets.QLabel] = {}
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(6)

        rows = [
            ("accel", "Accel (x, y, z)"),
            ("gyro", "Gyro (x, y, z)"),
            ("mag", "Mag (x, y, z)"),
            ("env", "Pressure / Temp / Alt"),
            ("rate", "IMU Hz"),
            ("ts", "Last update"),
        ]
        for idx, (key, label) in enumerate(rows):
            label_widget = QtWidgets.QLabel(label)
            value_widget = QtWidgets.QLabel("--")
            value_widget.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            self.imu_values[key] = value_widget
            grid.addWidget(label_widget, idx, 0)
            grid.addWidget(value_widget, idx, 1)

        imu_layout.addLayout(grid)
        legend = QtWidgets.QLabel("Signals plotted: Accel (x,y,z), Gyro (x,y,z), Mag (x,y,z)")
        legend.setStyleSheet("color: #8b949e;")
        imu_layout.addWidget(legend)
        imu_layout.addStretch(1)

        layout.addWidget(imu_panel, 1)

        return page

    def _build_gps_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(12)

        map_group = QtWidgets.QGroupBox("Coordinate Map")
        map_layout = QtWidgets.QVBoxLayout(map_group)
        map_placeholder = QtWidgets.QLabel("Map view placeholder")
        map_placeholder.setAlignment(ALIGN_CENTER)
        map_placeholder.setMinimumHeight(360)
        map_placeholder.setStyleSheet("border: 1px dashed #2a313b; color: #8b949e;")
        map_layout.addWidget(map_placeholder)

        gps_group = QtWidgets.QGroupBox("GPS Data")
        gps_layout = QtWidgets.QFormLayout(gps_group)
        gps_layout.addRow("Latitude", QtWidgets.QLabel("--"))
        gps_layout.addRow("Longitude", QtWidgets.QLabel("--"))
        gps_layout.addRow("Altitude", QtWidgets.QLabel("--"))
        gps_layout.addRow("Fix status", QtWidgets.QLabel("--"))

        layout.addWidget(map_group, 2)
        layout.addWidget(gps_group, 1)
        layout.addStretch(1)

        return page

    def _build_data_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        top_split = QtWidgets.QSplitter(Qt.Orientation.Horizontal)
        top_split.setChildrenCollapsible(False)

        left_panel = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        left_layout.addWidget(self._build_data_group())
        left_layout.addWidget(self._build_record_group())
        left_layout.addStretch(1)

        top_split.addWidget(left_panel)
        top_split.addWidget(self._build_log_group())
        top_split.setStretchFactor(0, 2)
        top_split.setStretchFactor(1, 1)

        layout.addWidget(top_split, 1)
        return page

    def _build_controls_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Controls")
        layout = QtWidgets.QGridLayout(group)
        layout.setSpacing(8)

        self.btn_connect = QtWidgets.QPushButton("Reconnect")
        self.btn_start = QtWidgets.QPushButton("Start Stream")
        self.btn_stop = QtWidgets.QPushButton("Stop Stream")
        self.btn_snapshot = QtWidgets.QPushButton("Snapshot")
        self.btn_clear = QtWidgets.QPushButton("Clear Measurements")

        layout.addWidget(self.btn_connect, 0, 0)
        layout.addWidget(self.btn_start, 0, 1)
        layout.addWidget(self.btn_stop, 0, 2)
        layout.addWidget(self.btn_snapshot, 1, 0)
        layout.addWidget(self.btn_clear, 1, 1, 1, 2)

        return group

    def _build_record_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Data Recording & Export")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(8)

        self.record_state_label = QtWidgets.QLabel("Recording: OFF")
        self.record_state_label.setStyleSheet("font-weight: 700; color: #f85149;")
        self.record_count_label = QtWidgets.QLabel("Samples: 0")
        self.record_path_label = QtWidgets.QLabel("Last export: --")

        buttons = QtWidgets.QHBoxLayout()
        self.btn_record_start = QtWidgets.QPushButton("Start Recording")
        self.btn_record_export = QtWidgets.QPushButton("Stop & Export CSV")
        buttons.addWidget(self.btn_record_start)
        buttons.addWidget(self.btn_record_export)

        layout.addWidget(self.record_state_label)
        layout.addWidget(self.record_count_label)
        layout.addWidget(self.record_path_label)
        layout.addLayout(buttons)

        return group

    def _build_measure_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Measurements")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(6)
        self.measure_toggle = QtWidgets.QCheckBox("Measurement mode")
        self.cursor_depth_label = QtWidgets.QLabel("--")
        self.distance_label = QtWidgets.QLabel("--")
        self.center_depth_label = QtWidgets.QLabel("--")
        layout.addRow(self.measure_toggle)
        layout.addRow("Depth @ cursor", self.cursor_depth_label)
        layout.addRow("2-point distance", self.distance_label)
        layout.addRow("Center distance", self.center_depth_label)
        return group

    def _build_data_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Runtime Data")
        layout = QtWidgets.QGridLayout(group)
        layout.setHorizontalSpacing(16)
        layout.setVerticalSpacing(8)
        layout.setContentsMargins(12, 12, 12, 12)
        group.setStyleSheet("QGroupBox { font-size: 14px; font-weight: 600; } QLabel { font-size: 13px; }")

        self.data_labels: Dict[str, QtWidgets.QLabel] = {}
        fields = [
            ("cam_seq", "CAM Seq"),
            ("cam_res", "Resolution"),
            ("cam_pub_fps", "Publish FPS"),
            ("cam_latency", "Latency (ms)"),
            ("cam_color_bytes", "Color bytes"),
            ("cam_depth_bytes", "Depth bytes"),
            ("cam_rx_fps", "RX FPS"),
            ("dropped", "Overwritten frames"),
            ("imu_accel", "Accel (g)"),
            ("imu_gyro", "Gyro (dps)"),
            ("imu_mag", "Mag (uT)"),
            ("imu_rate", "IMU Hz"),
            ("imu_ts", "IMU t_wall"),
            ("conn", "Broker"),
            ("status", "Status"),
        ]
        mid = (len(fields) + 1) // 2
        for idx, (key, label) in enumerate(fields):
            col_base = 0 if idx < mid else 2
            row = idx if idx < mid else idx - mid

            label_widget = QtWidgets.QLabel(label)
            label_widget.setAlignment(ALIGN_LEFT | ALIGN_VCENTER)

            value_widget = QtWidgets.QLabel("--")
            value_widget.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            value_widget.setMinimumWidth(220)
            value_widget.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Preferred)

            self.data_labels[key] = value_widget
            layout.addWidget(label_widget, row, col_base)
            layout.addWidget(value_widget, row, col_base + 1)

        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(3, 1)
        return group

    def _build_log_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Logs / Console")
        layout = QtWidgets.QVBoxLayout(group)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        layout.addWidget(self.log_view)
        return group

    def _connect_signals(self) -> None:
        self.btn_connect.clicked.connect(self.handle_reconnect)
        self.btn_start.clicked.connect(self.start_streaming)
        self.btn_stop.clicked.connect(self.stop_streaming)
        self.btn_snapshot.clicked.connect(self.take_snapshot)
        self.btn_clear.clicked.connect(self.clear_measurements)
        self.measure_toggle.clicked.connect(self.toggle_measurement)
        self.btn_record_start.clicked.connect(self.start_recording)
        self.btn_record_export.clicked.connect(self.stop_and_export)

        self.color_widget.clicked.connect(self.handle_click)
        self.color_widget.moved.connect(self.handle_move)

    def log(self, message: str) -> None:
        if QtCore.QThread.currentThread() != self.thread():
            QtCore.QMetaObject.invokeMethod(
                self,
                "_append_log",
                Qt.ConnectionType.QueuedConnection,
                QtCore.Q_ARG(str, message),
            )
            return
        self._append_log(message)

    @Slot(str)
    def _append_log(self, message: str) -> None:
        timestamped = f"[{datetime.now().strftime('%H:%M:%S')}] {message}"
        self.logs.append(timestamped)
        if len(self.logs) > LOG_MAX_LINES:
            self.logs = self.logs[-LOG_MAX_LINES:]
        self.log_view.setPlainText("\n".join(self.logs))
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def start_recording(self) -> None:
        self.recording_active = True
        self.record_rows = []
        self._last_record_ts = 0.0
        self.record_state_label.setText("Recording: ON")
        self.record_state_label.setStyleSheet("font-weight: 700; color: #2ea043;")
        self.recording_status.setText("Recording: ON")
        self.recording_status.setStyleSheet("font-size: 13px; color: #2ea043; font-weight: 700;")
        self.record_count_label.setText("Samples: 0")
        self.log("⏺ Recording started")

    def stop_and_export(self) -> None:
        if not self.recording_active:
            self.log("⚠️ Recording already stopped")
        self.recording_active = False
        self.record_state_label.setText("Recording: OFF")
        self.record_state_label.setStyleSheet("font-weight: 700; color: #f85149;")
        self.recording_status.setText("Recording: OFF")
        self.recording_status.setStyleSheet("font-size: 13px; color: #f85149; font-weight: 700;")
        if not self.record_rows:
            self.log("⚠️ No data to export")
            return
        output_dir = Path(__file__).parent / "exports"
        output_dir.mkdir(parents=True, exist_ok=True)
        ts = timestamp_now()
        path = output_dir / f"imu_data_{ts}.csv"
        fieldnames = list(self.record_rows[0].keys())
        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.record_rows)
        self.last_export_path = path
        self.record_path_label.setText(f"Last export: {path.name}")
        self.log(f"💾 Exported CSV to {path}")

    def collect_control_payload(self, streaming_enabled: bool) -> Dict[str, Any]:
        return {"streaming_enabled": streaming_enabled}

    def handle_reconnect(self) -> None:
        if DEMO_MODE:
            self.log("ℹ️ Demo mode: reconnect ignored")
            return
        self.mqtt_thread.reconnect()

    def start_streaming(self) -> None:
        payload = self.collect_control_payload(streaming_enabled=True)
        if not DEMO_MODE:
            self.mqtt_thread.publish_control(payload)
        self.log("▶️ Start streaming")

    def stop_streaming(self) -> None:
        payload = self.collect_control_payload(streaming_enabled=False)
        if not DEMO_MODE:
            self.mqtt_thread.publish_control(payload)
        self.log("⏸ Stop streaming")

    def take_snapshot(self) -> None:
        snapshot = self.frames.snapshot()
        if snapshot.color_image is None:
            self.log("⚠️ Snapshot failed: no color frame")
            return
        output_dir = Path(__file__).parent / "snapshots"
        output_dir.mkdir(parents=True, exist_ok=True)
        ts = timestamp_now()
        color_path = output_dir / f"color_{ts}.png"
        snapshot.color_image.save(str(color_path))
        if snapshot.depth_raw is not None:
            depth_path = output_dir / f"depth_{ts}.png"
            cv2.imwrite(str(depth_path), snapshot.depth_raw)
        self.log(f"📸 Snapshot saved to {output_dir}")

    def clear_measurements(self) -> None:
        self._measure_points = []
        self._distance_text = "--"
        self.distance_label.setText(self._distance_text)
        self.color_widget.set_measure_points(())
        self.log("🧹 Measurements cleared")

    def toggle_measurement(self) -> None:
        self._measure_mode = self.measure_toggle.isChecked()
        self.clear_measurements()

    def handle_click(self, u: int, v: int) -> None:
        if not self._measure_mode:
            return
        self._measure_points.append((u, v))
        if len(self._measure_points) > 2:
            self._measure_points = self._measure_points[-2:]
        self.color_widget.set_measure_points(tuple(self._measure_points))

        if len(self._measure_points) == 2:
            snapshot = self.frames.snapshot()
            state = self.state.snapshot()

            intr = state.calib or state.meta.get("intrinsics", {})
            depth_scale = (state.calib.get("depth_scale") if state.calib else None) or state.meta.get("depth_scale")

            dist, msg = compute_distance(
                self._measure_points[0],
                self._measure_points[1],
                snapshot.depth_raw,
                intr if isinstance(intr, dict) else {},
                depth_scale,
            )
            self._distance_text = msg if dist is None else f"{dist:.3f} m"
            self.distance_label.setText(self._distance_text)

    def handle_move(self, u: int, v: int) -> None:
        if not self._measure_mode:
            return

        snapshot = self.frames.snapshot()
        state = self.state.snapshot()
        depth_scale = (state.calib.get("depth_scale") if state.calib else None) or state.meta.get("depth_scale")

        if snapshot.depth_raw is None or depth_scale is None:
            self._cursor_depth = "--"
        else:
            h, w = snapshot.depth_raw.shape[:2]
            if 0 <= u < w and 0 <= v < h:
                z = float(snapshot.depth_raw[v, u]) * float(depth_scale)
                self._cursor_depth = f"{z:.3f} m" if z > 0 else "No valid depth"
            else:
                self._cursor_depth = "--"

        self.cursor_depth_label.setText(self._cursor_depth)

    def demo_update(self) -> None:
        t = time.monotonic()
        color, depth_preview, depth_raw = demo_frames(t)
        if color is not None:
            self.frames.update_color(color, t, color.sizeInBytes())
        if depth_preview is not None and depth_raw is not None:
            self.frames.update_depth(depth_raw, depth_preview, t, depth_raw.nbytes)
        self.state.update(
            "imu",
            {
                "accel": {"x": 0.0, "y": 0.0, "z": 1.0 + 0.1 * np.sin(t)},
                "gyro": {"x": 0.01 * np.cos(t), "y": 0.02 * np.sin(t), "z": 0.01},
                "mag": {"x": 12.0, "y": 5.0, "z": 30.0 + 0.1 * np.cos(t)},
                "imu_hz": 200,
                "t_wall": time.time(),
                "pressure_hpa": 1013.0,
                "temp_c": 24.0,
                "alt_m": 50.0,
            },
        )
        self.state.set_connected(False)

    def _append_record(self, imu: Dict[str, Any], state_ts: float) -> None:
        accel = imu.get("accel", {}) if isinstance(imu.get("accel", {}), dict) else {}
        gyro = imu.get("gyro", {}) if isinstance(imu.get("gyro", {}), dict) else {}
        mag = imu.get("mag", {}) if isinstance(imu.get("mag", {}), dict) else {}
        row = {
            "t_wall": imu.get("t_wall", state_ts),
            "ax_g": accel.get("x", 0.0),
            "ay_g": accel.get("y", 0.0),
            "az_g": accel.get("z", 0.0),
            "gx_dps": gyro.get("x", 0.0),
            "gy_dps": gyro.get("y", 0.0),
            "gz_dps": gyro.get("z", 0.0),
            "mx_uT": mag.get("x", 0.0),
            "my_uT": mag.get("y", 0.0),
            "mz_uT": mag.get("z", 0.0),
            "p_hpa": imu.get("pressure_hpa", 0.0),
            "t_C": imu.get("temp_c", 0.0),
            "alt_m": imu.get("alt_m", 0.0),
        }
        self.record_rows.append(row)
        self.record_count_label.setText(f"Samples: {len(self.record_rows)}")

    def refresh_ui(self) -> None:
        snapshot = self.frames.snapshot()
        state = self.state.snapshot()

        overwritten = self.latest_buffer.overwritten_counts()
        dropped_color = overwritten.get(TOPIC_COLOR, 0)
        dropped_depth = overwritten.get(TOPIC_DEPTH, 0)

        self.status_label.setText("Connected" if state.connected else "Disconnected")
        self.status_label.setStyleSheet("color: #2ea043;" if state.connected else "color: #f85149;")

        if snapshot.color_image is not None:
            self.color_widget.set_image(snapshot.color_image)
        if snapshot.depth_image is not None:
            self.depth_widget.set_image(snapshot.depth_image)

        meta = state.meta
        latency_ms = "N/A"
        if meta.get("t_wall") is not None:
            latency_ms = f"{(time.time() - float(meta['t_wall'])) * 1000.0:.1f}"
        rx_fps = f"{snapshot.color_rx_fps:.1f}/{snapshot.depth_rx_fps:.1f}"
        dropped = dropped_color + dropped_depth
        latency_text = f"{latency_ms} ms" if latency_ms != "N/A" else latency_ms
        self.quick_stats.setText(f"Latency: {latency_text} | RX FPS: {rx_fps} | Overwritten: {dropped}")

        resolution = f"{meta.get('color_w', '--')}x{meta.get('color_h', '--')}"
        overlay_color = (
            f"Res: {resolution}",
            f"Latency: {latency_text}",
            f"RX FPS: {snapshot.color_rx_fps:.1f}",
            f"Bytes: {snapshot.color_bytes}",
            f"Overwritten: {dropped_color}",
        )
        overlay_depth = (
            f"Res: {meta.get('depth_w', '--')}x{meta.get('depth_h', '--')}",
            f"RX FPS: {snapshot.depth_rx_fps:.1f}",
            f"Bytes: {snapshot.depth_bytes}",
            f"Overwritten: {dropped_depth}",
        )
        self.color_widget.set_overlay(overlay_color)
        self.depth_widget.set_overlay(overlay_depth)

        self.data_labels["cam_seq"].setText(str(meta.get("seq", "--")))
        self.data_labels["cam_res"].setText(resolution)
        self.data_labels["cam_pub_fps"].setText(f"{meta.get('pub_fps', '--')}")
        self.data_labels["cam_latency"].setText(latency_text)
        self.data_labels["cam_color_bytes"].setText(str(snapshot.color_bytes))
        self.data_labels["cam_depth_bytes"].setText(str(snapshot.depth_bytes))
        self.data_labels["cam_rx_fps"].setText(f"{snapshot.color_rx_fps:.1f}")
        self.data_labels["dropped"].setText(str(dropped))

        imu = state.imu
        if isinstance(state.imu, dict) and state.imu and hasattr(self, "imu_plot"):
            self.imu_plot.push(state.imu)
            self._last_imu_update = time.monotonic()
        if hasattr(self, "imu_plot"):
            no_data = (time.monotonic() - self._last_imu_update) > 2.0
            self.imu_plot.set_no_data(no_data)

        accel = imu.get("accel", {}) if isinstance(imu.get("accel", {}), dict) else {}
        gyro = imu.get("gyro", {}) if isinstance(imu.get("gyro", {}), dict) else {}
        mag = imu.get("mag", {}) if isinstance(imu.get("mag", {}), dict) else {}
        accel_text = f"{accel.get('x', '--')} / {accel.get('y', '--')} / {accel.get('z', '--')}"
        gyro_text = f"{gyro.get('x', '--')} / {gyro.get('y', '--')} / {gyro.get('z', '--')}"
        mag_text = f"{mag.get('x', '--')} / {mag.get('y', '--')} / {mag.get('z', '--')}"
        self.data_labels["imu_accel"].setText(accel_text)
        self.data_labels["imu_gyro"].setText(gyro_text)
        self.data_labels["imu_mag"].setText(mag_text)
        self.data_labels["imu_rate"].setText(f"{imu.get('imu_hz', '--'):.2f}" if imu.get("imu_hz") else "--")
        self.data_labels["imu_ts"].setText(str(imu.get("t_wall", "--")))

        if hasattr(self, "imu_values"):
            self.imu_values["accel"].setText(accel_text)
            self.imu_values["gyro"].setText(gyro_text)
            self.imu_values["mag"].setText(mag_text)
            env_text = f"{imu.get('pressure_hpa', '--')} / {imu.get('temp_c', '--')} / {imu.get('alt_m', '--')}"
            self.imu_values["env"].setText(env_text)
            self.imu_values["rate"].setText(self.data_labels["imu_rate"].text())
            self.imu_values["ts"].setText(self.data_labels["imu_ts"].text())

        if self.recording_active and state.last_imu_ts > self._last_record_ts and imu:
            self._last_record_ts = state.last_imu_ts
            self._append_record(imu, state.last_imu_ts)

        status = state.status
        ok = status.get("ok")
        status_text = "OK" if ok else "--"
        if status.get("errors"):
            status_text = f"Errors: {status['errors']}"
        self.data_labels["status"].setText(status_text)
        self.data_labels["conn"].setText(f"{BROKER_HOST}:{BROKER_PORT}")

        if snapshot.depth_raw is not None:
            center_distance = self.compute_center_distance(snapshot.depth_raw, state)
            self.center_depth_label.setText(center_distance)

    def compute_center_distance(self, depth_raw: np.ndarray, state: SharedState) -> str:
        depth_scale = (state.calib.get("depth_scale") if state.calib else None) or state.meta.get("depth_scale")
        if depth_scale is None:
            return "--"
        h, w = depth_raw.shape[:2]
        half = 20
        r0 = max(0, h // 2 - half)
        r1 = min(h, h // 2 + half)
        c0 = max(0, w // 2 - half)
        c1 = min(w, w // 2 + half)
        roi = depth_raw[r0:r1, c0:c1]
        valid = roi[roi > 0]
        if valid.size == 0:
            return "No valid depth"
        mean_depth = float(np.mean(valid)) * float(depth_scale)
        return f"{mean_depth:.3f} m"

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.refresh_timer.stop()
        self.demo_timer.stop()
        self.decoder_thread.stop()
        if not DEMO_MODE:
            self.mqtt_thread.stop()
        super().closeEvent(event)


def main() -> None:
    app = QtWidgets.QApplication([])
    window = DashboardWindow()
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
