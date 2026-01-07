#!/usr/bin/env python3
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import lz4.frame as lz4_frame
except Exception:
    lz4_frame = None

MAGIC = b"RSF1"
HEADER_FMT = "<4sBIQHHBBI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

KIND_COLOR = 0
KIND_DEPTH = 1

DTYPE_CODE_FROM = {
    np.dtype(np.uint8): 1,
    np.dtype(np.uint16): 2,
}

DTYPE_FROM_CODE = {
    1: np.uint8,
    2: np.uint16,
}


@dataclass
class FramePacket:
    kind: int
    seq: int
    t_cap_ns: int
    width: int
    height: int
    channels: int
    dtype: np.dtype
    payload_len: int
    array: np.ndarray
    compressed: bool = False


def _shape_and_channels(array: np.ndarray) -> Tuple[int, int, int]:
    if array.ndim == 2:
        h, w = array.shape
        return w, h, 1
    if array.ndim == 3:
        h, w, c = array.shape
        return w, h, c
    raise ValueError(f"Unsupported array shape: {array.shape}")


def pack_frame(
    kind: int,
    seq: int,
    t_cap_ns: int,
    array: np.ndarray,
    compress: bool = False,
) -> bytes:
    if kind not in (KIND_COLOR, KIND_DEPTH):
        raise ValueError(f"Invalid kind: {kind}")

    array = np.ascontiguousarray(array)
    w, h, channels = _shape_and_channels(array)

    dtype = np.dtype(array.dtype)
    dtype_code = DTYPE_CODE_FROM.get(dtype)
    if dtype_code is None:
        raise ValueError(f"Unsupported dtype: {dtype}")

    payload = array.tobytes()
    payload_len = len(payload)
    compressed = False

    if compress and lz4_frame is not None:
        compressed_payload = lz4_frame.compress(payload)
        if len(compressed_payload) < payload_len:
            payload = compressed_payload
            payload_len = len(payload)
            compressed = True

    header = struct.pack(
        HEADER_FMT,
        MAGIC,
        int(kind),
        int(seq) & 0xFFFFFFFF,
        int(t_cap_ns) & 0xFFFFFFFFFFFFFFFF,
        int(w) & 0xFFFF,
        int(h) & 0xFFFF,
        int(channels) & 0xFF,
        int(dtype_code) & 0xFF,
        int(payload_len) & 0xFFFFFFFF,
    )
    return header + payload


def unpack_frame(payload: bytes, allow_lz4: bool = True) -> FramePacket:
    if len(payload) < HEADER_SIZE:
        raise ValueError("Payload too small for header")

    magic, kind, seq, t_cap_ns, w, h, channels, dtype_code, payload_len = struct.unpack(
        HEADER_FMT,
        payload[:HEADER_SIZE],
    )
    if magic != MAGIC:
        raise ValueError("Invalid magic header")

    dtype = DTYPE_FROM_CODE.get(dtype_code)
    if dtype is None:
        raise ValueError(f"Unsupported dtype code: {dtype_code}")

    data = payload[HEADER_SIZE:]
    if len(data) != payload_len:
        raise ValueError("Payload length mismatch")

    expected_len = int(w) * int(h) * int(channels) * np.dtype(dtype).itemsize
    raw = data
    compressed = False

    if payload_len != expected_len:
        if allow_lz4 and lz4_frame is not None:
            raw = lz4_frame.decompress(data)
            if len(raw) != expected_len:
                raise ValueError("Decompressed payload size mismatch")
            compressed = True
        else:
            raise ValueError("Unexpected payload size")

    array = np.frombuffer(raw, dtype=dtype)
    if channels == 1:
        array = array.reshape((int(h), int(w)))
    else:
        array = array.reshape((int(h), int(w), int(channels)))

    return FramePacket(
        kind=kind,
        seq=seq,
        t_cap_ns=t_cap_ns,
        width=w,
        height=h,
        channels=channels,
        dtype=np.dtype(dtype),
        payload_len=payload_len,
        array=array,
        compressed=compressed,
    )
