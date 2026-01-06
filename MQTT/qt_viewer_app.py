#!/usr/bin/env python3
"""High-performance Qt dashboard for Jetson + RealSense MQTT streams."""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import paho.mqtt.client as mqtt
try:
    # PySide6
    from PySide6 import QtCore, QtGui, QtWidgets
    Signal = QtCore.Signal
    Slot = QtCore.Slot
except ImportError:
    # PyQt6
    from PyQt6 import QtCore, QtGui, QtWidgets
    Signal = QtCore.pyqtSignal
    Slot = QtCore.pyqtslot

BROKER_HOST = os.getenv("MQTT_HOST", "test.mosquitto.org")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")
CLIENT_ID_PREFIX = "qt-viewer"

TOPIC_COLOR = "cam/jetson01/color_jpg"
TOPIC_DEPTH = "cam/jetson01/depth_jpg"
TOPIC_DEPTH_Z16 = "cam/jetson01/depth_z16_png"
TOPIC_META = "cam/jetson01/meta"
TOPIC_STATUS = "cam/jetson01/status"
TOPIC_CALIB = "cam/jetson01/calib"
TOPIC_IMU = "cam/jetson01/imu"
TOPIC_CONTROL = "cam/jetson01/control"

DEMO_MODE = os.getenv("DEMO_MODE", "0") == "1"

DEFAULT_CONTROL = {
    "jpeg_quality": 20,
    "pub_hz": 20,
    "color_w": 424,
    "color_h": 240,
    "color_fps": 30,
    "publish_depth_preview": True,
    "publish_depth_z16": False,
    "imu_enabled": True,
    "imu_hz": 200,
    "record_enabled": False,
    "streaming_enabled": True,
}

RESOLUTION_OPTIONS = [
    (424, 240),
    (640, 480),
    (848, 480),
    (1280, 720),
]

MAX_QUEUE = 6
LOG_MAX_LINES = 400


def timestamp_now() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


@dataclass
class LatestFrames:
    color_image: Optional[QtGui.QImage] = None
    depth_image: Optional[QtGui.QImage] = None
    depth_raw: Optional[np.ndarray] = None
    color_bytes: int = 0
    depth_bytes: int = 0
    depth_raw_bytes: int = 0
    color_rx_fps: float = 0.0
    depth_rx_fps: float = 0.0
    last_color_ts: float = 0.0
    last_depth_ts: float = 0.0
    dropped_color: int = 0
    dropped_depth: int = 0
    dropped_depth_raw: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_color(self, image: QtGui.QImage, ts: float, payload_bytes: int) -> None:
        with self.lock:
            if self.last_color_ts > 0:
                self.color_rx_fps = 1.0 / max(1e-6, ts - self.last_color_ts)
            self.last_color_ts = ts
            self.color_image = image
            self.color_bytes = payload_bytes

    def update_depth(self, image: QtGui.QImage, ts: float, payload_bytes: int) -> None:
        with self.lock:
            if self.last_depth_ts > 0:
                self.depth_rx_fps = 1.0 / max(1e-6, ts - self.last_depth_ts)
            self.last_depth_ts = ts
            self.depth_image = image
            self.depth_bytes = payload_bytes

    def update_depth_raw(self, depth_raw: np.ndarray, payload_bytes: int) -> None:
        with self.lock:
            self.depth_raw = depth_raw
            self.depth_raw_bytes = payload_bytes

    def snapshot(self) -> "LatestFrames":
        with self.lock:
            return LatestFrames(
                color_image=self.color_image,
                depth_image=self.depth_image,
                depth_raw=self.depth_raw.copy() if self.depth_raw is not None else None,
                color_bytes=self.color_bytes,
                depth_bytes=self.depth_bytes,
                depth_raw_bytes=self.depth_raw_bytes,
                color_rx_fps=self.color_rx_fps,
                depth_rx_fps=self.depth_rx_fps,
                last_color_ts=self.last_color_ts,
                last_depth_ts=self.last_depth_ts,
                dropped_color=self.dropped_color,
                dropped_depth=self.dropped_depth,
                dropped_depth_raw=self.dropped_depth_raw,
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
    h, w, _ = rgb.shape
    image = QtGui.QImage(rgb.data, w, h, QtGui.QImage.Format_RGB888)
    return image.copy()


def decode_jpg(payload: bytes) -> Optional[QtGui.QImage]:
    if not payload:
        return None
    arr = np.frombuffer(payload, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return make_qimage_from_rgb(rgb)


def decode_depth_png(payload: bytes) -> Optional[np.ndarray]:
    if not payload:
        return None
    arr = np.frombuffer(payload, dtype=np.uint8)
    depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if depth is None or depth.dtype != np.uint16:
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
    fx = intr.get("fx", 1.0)
    fy = intr.get("fy", 1.0)
    ppx = intr.get("ppx", 0.0)
    ppy = intr.get("ppy", 0.0)
    x = (u - ppx) / fx * depth_m
    y = (v - ppy) / fy * depth_m
    return x, y, depth_m


def compute_distance(p1: Tuple[int, int], p2: Tuple[int, int], depth: np.ndarray, intr: Dict[str, float], depth_scale: float) -> Tuple[Optional[float], str]:
    if depth is None or depth_scale is None or not intr:
        return None, "No depth/intrinsics"
    h, w = depth.shape[:2]
    (u1, v1), (u2, v2) = p1, p2
    if not (0 <= u1 < w and 0 <= v1 < h and 0 <= u2 < w and 0 <= v2 < h):
        return None, "Point out of range"
    z1 = depth[v1, u1] * depth_scale
    z2 = depth[v2, u2] * depth_scale
    if z1 <= 0 or z2 <= 0:
        return None, "No valid depth"
    x1, y1, z1 = project_pixel_to_3d(u1, v1, z1, intr)
    x2, y2, z2 = project_pixel_to_3d(u2, v2, z2, intr)
    dist = float(np.linalg.norm([x2 - x1, y2 - y1, z2 - z1]))
    return dist, ""


class BoundedQueue:
    def __init__(self, maxsize: int) -> None:
        self.queue: "queue.Queue[Tuple[str, bytes, float]]" = queue.Queue(maxsize=maxsize)
        self.dropped: Dict[str, int] = {TOPIC_COLOR: 0, TOPIC_DEPTH: 0, TOPIC_DEPTH_Z16: 0}

    def put(self, topic: str, payload: bytes) -> None:
        try:
            self.queue.put_nowait((topic, payload, time.monotonic()))
        except queue.Full:
            self.dropped[topic] = self.dropped.get(topic, 0) + 1
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait((topic, payload, time.monotonic()))
            except queue.Full:
                self.dropped[topic] = self.dropped.get(topic, 0) + 1


class MqttClient(threading.Thread):
    def __init__(self, frame_queue: BoundedQueue, state: SharedState, log_cb) -> None:
        super().__init__(daemon=True)
        self.frame_queue = frame_queue
        self.state = state
        self.log_cb = log_cb
        self.client = mqtt.Client(client_id=f"{CLIENT_ID_PREFIX}-{int(time.time())}", clean_session=True)
        if MQTT_USER:
            self.client.username_pw_set(MQTT_USER, MQTT_PASS)
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.log_cb("✅ MQTT connected")
            client.subscribe([(TOPIC_COLOR, 0), (TOPIC_DEPTH, 0), (TOPIC_DEPTH_Z16, 0),
                              (TOPIC_META, 0), (TOPIC_STATUS, 0), (TOPIC_CALIB, 0), (TOPIC_IMU, 0)])
            self.state.set_connected(True)
            self._connected_event.set()
        else:
            self.log_cb(f"⚠️ MQTT connect failed rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        self.state.set_connected(False)
        self.log_cb(f"⚠️ MQTT disconnected rc={rc} (will retry)")

    def _on_message(self, client, userdata, msg):
        if msg.topic in {TOPIC_COLOR, TOPIC_DEPTH, TOPIC_DEPTH_Z16}:
            self.frame_queue.put(msg.topic, msg.payload)
            return
        if msg.topic in {TOPIC_META, TOPIC_STATUS, TOPIC_CALIB, TOPIC_IMU}:
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
            elif msg.topic == TOPIC_IMU:
                self.state.update("imu", data)

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
    def __init__(self, frame_queue: BoundedQueue, frames: LatestFrames) -> None:
        super().__init__(daemon=True)
        self.frame_queue = frame_queue
        self.frames = frames
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                topic, payload, ts = self.frame_queue.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if topic == TOPIC_COLOR:
                image = decode_jpg(payload)
                if image is not None:
                    self.frames.update_color(image, ts, len(payload))
            elif topic == TOPIC_DEPTH:
                image = decode_jpg(payload)
                if image is not None:
                    self.frames.update_depth(image, ts, len(payload))
            elif topic == TOPIC_DEPTH_Z16:
                depth = decode_depth_png(payload)
                if depth is not None:
                    self.frames.update_depth_raw(depth, len(payload))

    def stop(self) -> None:
        self._stop_event.set()


class VideoWidget(QtWidgets.QWidget):
    clicked = Signal(int, int)
    moved = Signal(int, int)
    @Slot(int, int)
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
        if image is not None:
            self._img_size = image.size()
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
            scaled.scale(rect.size(), QtCore.Qt.KeepAspectRatio)
            target = QtCore.QRect(QtCore.QPoint(0, 0), scaled)
            target.moveCenter(rect.center())
            self._last_rect = target
            painter.drawImage(target, self.image)
        else:
            self._last_rect = rect
            painter.setPen(QtGui.QColor("#8a8f98"))
            painter.drawText(rect, QtCore.Qt.AlignCenter, "No frame")

        painter.setPen(QtGui.QColor("#dfe6ee"))
        painter.setFont(QtGui.QFont("Inter", 10, QtGui.QFont.Bold))
        painter.drawText(rect.adjusted(6, 4, -6, -4), QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop, self.title)

        painter.setFont(QtGui.QFont("Inter", 9))
        y_offset = 24
        for line in self.overlay_lines:
            painter.drawText(rect.adjusted(6, y_offset, -6, -4), QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop, line)
            y_offset += 14

        if self.measure_points and self.image is not None:
            painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffcc00"), 2))
            for point in self.measure_points:
                widget_pt = self._image_to_widget(point)
                if widget_pt is None:
                    continue
                painter.drawEllipse(widget_pt, 5, 5)
            if len(self.measure_points) == 2:
                p1 = self._image_to_widget(self.measure_points[0])
                p2 = self._image_to_widget(self.measure_points[1])
                if p1 and p2:
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
        scale_x = img_w / self._last_rect.width()
        scale_y = img_h / self._last_rect.height()
        u = int((pos.x() - self._last_rect.left()) * scale_x)
        v = int((pos.y() - self._last_rect.top()) * scale_y)
        return u, v

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            pt = self._widget_to_image(event.position().toPoint())
            if pt:
                self.clicked.emit(pt[0], pt[1])

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        pt = self._widget_to_image(event.position().toPoint())
        if pt:
            self.moved.emit(pt[0], pt[1])


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
        self.queue = BoundedQueue(MAX_QUEUE)
        self.logs: list[str] = []

        self.mqtt_thread = MqttClient(self.queue, self.state, self.log)
        self.decoder_thread = DecoderWorker(self.queue, self.frames)
        self.decoder_thread.start()
        if not DEMO_MODE:
            self.mqtt_thread.start()

        self._measure_mode = False
        self._measure_points: list[Tuple[int, int]] = []
        self._cursor_depth = "--"
        self._distance_text = "--"

        self._auto_apply_timer = QtCore.QTimer(self)
        self._auto_apply_timer.setSingleShot(True)
        self._auto_apply_timer.timeout.connect(self.send_control)

        self._build_ui()
        self._connect_signals()

        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_ui)
        self.refresh_timer.start(33)

        self.demo_timer = QtCore.QTimer(self)
        self.demo_timer.timeout.connect(self.demo_update)
        if DEMO_MODE:
            self.demo_timer.start(33)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setContentsMargins(16, 16, 16, 16)
        main_layout.setSpacing(12)

        header = QtWidgets.QHBoxLayout()
        header.setSpacing(12)

        logo_box = QtWidgets.QLabel()
        logo_box.setFixedSize(120, 60)
        logo_box.setAlignment(QtCore.Qt.AlignCenter)
        logo_path = Path(__file__).parent / "assets" / "logo.png"
        if logo_path.exists():
            pix = QtGui.QPixmap(str(logo_path)).scaled(120, 60, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            logo_box.setPixmap(pix)
        else:
            logo_box.setText("LOGO")
            logo_box.setStyleSheet("border: 1px dashed #2a313b; color: #8b949e;")

        title_layout = QtWidgets.QVBoxLayout()
        self.title_label = QtWidgets.QLabel("Jetson + RealSense Streaming Dashboard")
        self.title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #f85149; font-weight: 600;")
        title_layout.addWidget(self.title_label)
        title_layout.addWidget(self.status_label)

        self.quick_stats = QtWidgets.QLabel("Latency: -- ms | RX FPS: -- | Dropped: --")
        self.quick_stats.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        header.addWidget(logo_box, 0)
        header.addLayout(title_layout, 1)
        header.addWidget(self.quick_stats, 0)

        body_layout = QtWidgets.QHBoxLayout()
        body_layout.setSpacing(12)

        video_layout = QtWidgets.QVBoxLayout()
        video_layout.setSpacing(12)
        self.color_widget = VideoWidget("Color")
        self.depth_widget = VideoWidget("Depth")
        video_layout.addWidget(self.color_widget, 3)
        video_layout.addWidget(self.depth_widget, 2)

        side_panel = QtWidgets.QVBoxLayout()
        side_panel.setSpacing(10)
        side_panel.setContentsMargins(0, 0, 0, 0)

        side_panel.addWidget(self._build_controls_group())
        side_panel.addWidget(self._build_stream_group())
        side_panel.addWidget(self._build_record_group())
        side_panel.addWidget(self._build_imu_group())
        side_panel.addWidget(self._build_measure_group())
        side_panel.addWidget(self._build_data_group())
        side_panel.addWidget(self._build_log_group(), 1)

        body_layout.addLayout(video_layout, 3)
        body_layout.addLayout(side_panel, 1)

        main_layout.addLayout(header)
        main_layout.addLayout(body_layout)

        self.setCentralWidget(central)

    def _build_controls_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Controls")
        layout = QtWidgets.QGridLayout(group)
        layout.setSpacing(8)

        self.btn_connect = QtWidgets.QPushButton("Reconnect")
        self.btn_start = QtWidgets.QPushButton("Start Stream")
        self.btn_stop = QtWidgets.QPushButton("Stop Stream")
        self.btn_snapshot = QtWidgets.QPushButton("Snapshot")
        self.btn_clear = QtWidgets.QPushButton("Clear Measurements")
        self.btn_reset = QtWidgets.QPushButton("Reset Defaults")
        self.btn_apply = QtWidgets.QPushButton("Apply")
        self.auto_apply = QtWidgets.QCheckBox("Auto apply")

        layout.addWidget(self.btn_connect, 0, 0)
        layout.addWidget(self.btn_apply, 0, 1)
        layout.addWidget(self.auto_apply, 0, 2)
        layout.addWidget(self.btn_start, 1, 0)
        layout.addWidget(self.btn_stop, 1, 1)
        layout.addWidget(self.btn_snapshot, 1, 2)
        layout.addWidget(self.btn_clear, 2, 0)
        layout.addWidget(self.btn_reset, 2, 1, 1, 2)

        return group

    def _build_stream_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Stream Settings")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(8)

        self.pub_hz = QtWidgets.QSpinBox()
        self.pub_hz.setRange(1, 60)
        self.pub_hz.setValue(DEFAULT_CONTROL["pub_hz"])

        self.jpeg_quality = QtWidgets.QSpinBox()
        self.jpeg_quality.setRange(5, 95)
        self.jpeg_quality.setValue(DEFAULT_CONTROL["jpeg_quality"])

        self.color_fps = QtWidgets.QSpinBox()
        self.color_fps.setRange(1, 90)
        self.color_fps.setValue(DEFAULT_CONTROL["color_fps"])

        self.resolution = QtWidgets.QComboBox()
        for w, h in RESOLUTION_OPTIONS:
            self.resolution.addItem(f"{w} x {h}", (w, h))
        self.resolution.setCurrentIndex(0)

        self.depth_preview = QtWidgets.QCheckBox("Depth preview")
        self.depth_preview.setChecked(True)
        self.depth_z16 = QtWidgets.QCheckBox("Depth Z16")
        self.depth_z16.setChecked(DEFAULT_CONTROL["publish_depth_z16"])

        layout.addRow("Publish Hz", self.pub_hz)
        layout.addRow("JPEG Quality", self.jpeg_quality)
        layout.addRow("Color FPS", self.color_fps)
        layout.addRow("Resolution", self.resolution)
        layout.addRow(self.depth_preview)
        layout.addRow(self.depth_z16)

        return group

    def _build_record_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Recording")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(6)
        self.record_toggle = QtWidgets.QCheckBox("Record enabled")
        self.record_toggle.setChecked(False)
        layout.addWidget(self.record_toggle)
        return group

    def _build_imu_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("IMU")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(6)
        self.imu_enable = QtWidgets.QCheckBox("IMU enabled")
        self.imu_enable.setChecked(True)
        self.imu_rate = QtWidgets.QSpinBox()
        self.imu_rate.setRange(1, 400)
        self.imu_rate.setValue(DEFAULT_CONTROL["imu_hz"])
        layout.addRow(self.imu_enable)
        layout.addRow("IMU Hz", self.imu_rate)
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
        group = QtWidgets.QGroupBox("Data")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(6)

        self.data_labels: Dict[str, QtWidgets.QLabel] = {}
        fields = [
            ("cam_seq", "CAM Seq"),
            ("cam_res", "Resolution"),
            ("cam_pub_hz", "Pub Hz"),
            ("cam_jpeg", "JPEG Quality"),
            ("cam_depth_pub", "Depth publish"),
            ("cam_stream", "Streaming"),
            ("cam_record", "Recording"),
            ("cam_rx_fps", "RX FPS"),
            ("cam_latency", "Latency (ms)"),
            ("cam_bytes", "Color bytes"),
            ("depth_bytes", "Depth bytes"),
            ("depth_raw", "Depth Z16 bytes"),
            ("dropped", "Dropped frames"),
            ("imu_accel", "Accel (m/s²)"),
            ("imu_gyro", "Gyro (rad/s)"),
            ("imu_rate", "IMU Hz"),
            ("imu_ts", "IMU t_wall"),
            ("conn", "Broker"),
            ("status", "Status"),
        ]
        for key, label in fields:
            widget = QtWidgets.QLabel("--")
            self.data_labels[key] = widget
            layout.addRow(label, widget)
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
        self.btn_apply.clicked.connect(self.send_control)
        self.btn_start.clicked.connect(self.start_streaming)
        self.btn_stop.clicked.connect(self.stop_streaming)
        self.btn_snapshot.clicked.connect(self.take_snapshot)
        self.btn_clear.clicked.connect(self.clear_measurements)
        self.btn_reset.clicked.connect(self.reset_defaults)
        self.record_toggle.clicked.connect(self.queue_auto_apply)
        self.measure_toggle.clicked.connect(self.toggle_measurement)
        self.imu_enable.clicked.connect(self.queue_auto_apply)
        self.imu_rate.valueChanged.connect(self.queue_auto_apply)
        self.pub_hz.valueChanged.connect(self.queue_auto_apply)
        self.jpeg_quality.valueChanged.connect(self.queue_auto_apply)
        self.color_fps.valueChanged.connect(self.queue_auto_apply)
        self.resolution.currentIndexChanged.connect(self.queue_auto_apply)
        self.depth_preview.clicked.connect(self.queue_auto_apply)
        self.depth_z16.clicked.connect(self.queue_auto_apply)

        self.color_widget.clicked.connect(self.handle_click)
        self.color_widget.moved.connect(self.handle_move)

    def log(self, message: str) -> None:
        if QtCore.QThread.currentThread() != self.thread():
            QtCore.QMetaObject.invokeMethod(
                self,
                "_append_log",
                QtCore.Qt.QueuedConnection,
                QtCore.Q_ARG(str, message),
            )
            return
        self._append_log(message)

    @QtCore.Slot(str)
    def _append_log(self, message: str) -> None:
        timestamped = f"[{datetime.now().strftime('%H:%M:%S')}] {message}"
        self.logs.append(timestamped)
        if len(self.logs) > LOG_MAX_LINES:
            self.logs = self.logs[-LOG_MAX_LINES:]
        self.log_view.setPlainText("\n".join(self.logs))
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def queue_auto_apply(self) -> None:
        if self.auto_apply.isChecked():
            self._auto_apply_timer.start(400)

    def send_control(self) -> None:
        payload = self.collect_control_payload()
        if DEMO_MODE:
            self.log("ℹ️ Demo mode: control payload not sent")
            return
        self.mqtt_thread.publish_control(payload)
        self.log(f"➡️ Sent control: {payload}")

    def collect_control_payload(self) -> Dict[str, Any]:
        resolution = self.resolution.currentData()
        if resolution is None:
            resolution = (DEFAULT_CONTROL["color_w"], DEFAULT_CONTROL["color_h"])
        color_w, color_h = resolution
        return {
            "jpeg_quality": self.jpeg_quality.value(),
            "pub_hz": self.pub_hz.value(),
            "color_w": color_w,
            "color_h": color_h,
            "color_fps": self.color_fps.value(),
            "publish_depth_preview": self.depth_preview.isChecked(),
            "publish_depth_z16": self.depth_z16.isChecked(),
            "imu_enabled": self.imu_enable.isChecked(),
            "imu_hz": self.imu_rate.value(),
            "record_enabled": self.record_toggle.isChecked(),
            "streaming_enabled": True,
        }

    def handle_reconnect(self) -> None:
        if DEMO_MODE:
            self.log("ℹ️ Demo mode: reconnect ignored")
            return
        self.mqtt_thread.reconnect()

    def start_streaming(self) -> None:
        payload = self.collect_control_payload()
        payload["streaming_enabled"] = True
        self.mqtt_thread.publish_control(payload)
        self.log("▶️ Start streaming")

    def stop_streaming(self) -> None:
        payload = self.collect_control_payload()
        payload["streaming_enabled"] = False
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

    def reset_defaults(self) -> None:
        self.pub_hz.setValue(DEFAULT_CONTROL["pub_hz"])
        self.jpeg_quality.setValue(DEFAULT_CONTROL["jpeg_quality"])
        self.color_fps.setValue(DEFAULT_CONTROL["color_fps"])
        self.resolution.setCurrentIndex(0)
        self.depth_preview.setChecked(True)
        self.depth_z16.setChecked(DEFAULT_CONTROL["publish_depth_z16"])
        self.imu_enable.setChecked(DEFAULT_CONTROL["imu_enabled"])
        self.imu_rate.setValue(DEFAULT_CONTROL["imu_hz"])
        self.record_toggle.setChecked(DEFAULT_CONTROL["record_enabled"])
        self.log("🔄 Reset to defaults")

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
            dist, msg = compute_distance(self._measure_points[0], self._measure_points[1], snapshot.depth_raw, intr, depth_scale)
            if dist is None:
                self._distance_text = msg
            else:
                self._distance_text = f"{dist:.3f} m"
            self.distance_label.setText(self._distance_text)

    def handle_move(self, u: int, v: int) -> None:
        if not self._measure_mode:
            return
        snapshot = self.frames.snapshot()
        state = self.state.snapshot()

        dropped_color = self.queue.dropped.get(TOPIC_COLOR, 0)
        dropped_depth = self.queue.dropped.get(TOPIC_DEPTH, 0)
        dropped_raw = self.queue.dropped.get(TOPIC_DEPTH_Z16, 0)
        depth_scale = (state.calib.get("depth_scale") if state.calib else None) or state.meta.get("depth_scale")
        if snapshot.depth_raw is None or depth_scale is None:
            self._cursor_depth = "--"
        else:
            h, w = snapshot.depth_raw.shape[:2]
            if 0 <= u < w and 0 <= v < h:
                z = snapshot.depth_raw[v, u] * depth_scale
                self._cursor_depth = f"{z:.3f} m" if z > 0 else "No valid depth"
            else:
                self._cursor_depth = "--"
        self.cursor_depth_label.setText(self._cursor_depth)

    def demo_update(self) -> None:
        t = time.monotonic()
        color, depth_preview, depth_raw = demo_frames(t)
        if color is not None:
            self.frames.update_color(color, t, color.sizeInBytes())
        if depth_preview is not None:
            self.frames.update_depth(depth_preview, t, depth_preview.sizeInBytes())
        if depth_raw is not None:
            self.frames.update_depth_raw(depth_raw, depth_raw.nbytes)
        self.state.update("imu", {
            "accel": {"x": 0.0, "y": 0.0, "z": 9.81 + 0.1 * np.sin(t)},
            "gyro": {"x": 0.01 * np.cos(t), "y": 0.02 * np.sin(t), "z": 0.01},
            "imu_hz": 200,
            "t_wall": time.time(),
        })
        self.state.set_connected(False)

    def refresh_ui(self) -> None:
        snapshot = self.frames.snapshot()
        state = self.state.snapshot()

        self.status_label.setText("Connected" if state.connected else "Disconnected")
        self.status_label.setStyleSheet("color: #2ea043;" if state.connected else "color: #f85149;")

        if snapshot.color_image is not None:
            self.color_widget.set_image(snapshot.color_image)
        if snapshot.depth_image is not None:
            self.depth_widget.set_image(snapshot.depth_image)
        elif snapshot.depth_raw is not None:
            self.depth_widget.set_image(depth_to_colormap(snapshot.depth_raw))

        meta = state.meta
        latency_ms = "--"
        if meta.get("t_wall"):
            latency_ms = f"{(time.time() - float(meta['t_wall'])) * 1000:.1f}"
        rx_fps = f"{snapshot.color_rx_fps:.1f}/{snapshot.depth_rx_fps:.1f}"
        dropped = dropped_color + dropped_depth + dropped_raw
        self.quick_stats.setText(f"Latency: {latency_ms} ms | RX FPS: {rx_fps} | Dropped: {dropped}")

        resolution = f"{meta.get('w', '--')}x{meta.get('h', '--')}"
        overlay_color = (
            f"Res: {resolution}",
            f"Latency: {latency_ms} ms",
            f"RX FPS: {snapshot.color_rx_fps:.1f}",
            f"Bytes: {snapshot.color_bytes}",
            f"Dropped: {dropped_color}",
        )
        overlay_depth = (
            f"Res: {resolution}",
            f"RX FPS: {snapshot.depth_rx_fps:.1f}",
            f"Bytes: {snapshot.depth_bytes}",
            f"Dropped: {dropped_depth + dropped_raw}",
        )
        self.color_widget.set_overlay(overlay_color)
        self.depth_widget.set_overlay(overlay_depth)

        self.data_labels["cam_seq"].setText(str(meta.get("seq", "--")))
        self.data_labels["cam_res"].setText(f"{meta.get('w', '--')} x {meta.get('h', '--')}")
        self.data_labels["cam_pub_hz"].setText(str(meta.get("pub_hz", "--")))
        self.data_labels["cam_jpeg"].setText(str(meta.get("jpeg_quality", "--")))
        self.data_labels["cam_depth_pub"].setText(
            f"Preview: {meta.get('publish_depth_preview', '--')} | Z16: {meta.get('publish_depth_z16', '--')}"
        )
        self.data_labels["cam_stream"].setText(str(meta.get("streaming_enabled", "--")))
        self.data_labels["cam_record"].setText(str(meta.get("record_enabled", "--")))
        self.data_labels["cam_rx_fps"].setText(f"{snapshot.color_rx_fps:.1f}")
        self.data_labels["cam_latency"].setText(latency_ms)
        self.data_labels["cam_bytes"].setText(str(snapshot.color_bytes))
        self.data_labels["depth_bytes"].setText(str(snapshot.depth_bytes))
        self.data_labels["depth_raw"].setText(str(snapshot.depth_raw_bytes))
        self.data_labels["dropped"].setText(str(dropped))

        imu = state.imu
        accel = imu.get("accel", {})
        gyro = imu.get("gyro", {})
        accel_text = f"{accel.get('x', '--')} / {accel.get('y', '--')} / {accel.get('z', '--')}"
        gyro_text = f"{gyro.get('x', '--')} / {gyro.get('y', '--')} / {gyro.get('z', '--')}"
        self.data_labels["imu_accel"].setText(accel_text)
        self.data_labels["imu_gyro"].setText(gyro_text)
        self.data_labels["imu_rate"].setText(str(imu.get("imu_hz", "--")))
        self.data_labels["imu_ts"].setText(str(imu.get("t_wall", "--")))

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
        mean_depth = float(np.mean(valid)) * depth_scale
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
