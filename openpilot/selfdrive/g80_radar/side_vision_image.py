#!/usr/bin/env python3
from __future__ import annotations

"""Small image helpers for G80 side-vision setup.

This module intentionally avoids OpenCV/Pillow so the 28994 setup page and
polygon snapshot still work on stock AGNOS images where cv2 is not installed.
"""

import struct
import zlib
from pathlib import Path

import numpy as np


def nv12_to_rgb(raw_image: np.ndarray, width: int, height: int, y_plane_rows: int) -> np.ndarray:
  """Convert a padded NV12 VisionIPC frame to RGB uint8 using NumPy only."""
  width = int(width)
  height = int(height)
  y_plane_rows = int(y_plane_rows)
  if width <= 0 or height <= 0 or y_plane_rows < height:
    raise ValueError("invalid NV12 geometry")

  y = raw_image[:height, :width].astype(np.int16, copy=False)
  uv = raw_image[y_plane_rows:y_plane_rows + height // 2, :width]
  if y.shape != (height, width) or uv.shape[0] < height // 2 or uv.shape[1] < width:
    raise ValueError(f"short NV12 frame y={y.shape} uv={uv.shape}")

  u_small = uv[:height // 2, 0:width:2].astype(np.int16, copy=False)
  v_small = uv[:height // 2, 1:width:2].astype(np.int16, copy=False)
  u = np.repeat(np.repeat(u_small, 2, axis=0), 2, axis=1)[:height, :width]
  v = np.repeat(np.repeat(v_small, 2, axis=0), 2, axis=1)[:height, :width]

  # BT.601 limited-range YUV -> RGB. Good enough for setup/annotation imagery.
  c = y - 16
  d = u - 128
  e = v - 128
  r = (298 * c + 409 * e + 128) >> 8
  g = (298 * c - 100 * d - 208 * e + 128) >> 8
  b = (298 * c + 516 * d + 128) >> 8
  return np.stack((r, g, b), axis=-1).clip(0, 255).astype(np.uint8)


def _png_chunk(tag: bytes, payload: bytes) -> bytes:
  body = tag + payload
  return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)


def rgb_to_png_bytes(rgb: np.ndarray, compress_level: int = 3) -> bytes:
  """Encode HxWx3 uint8 RGB as a simple non-interlaced PNG using stdlib only."""
  a = np.ascontiguousarray(rgb, dtype=np.uint8)
  if a.ndim != 3 or a.shape[2] != 3:
    raise ValueError("RGB image must be HxWx3")
  h, w, _ = a.shape
  # Filter type 0 per row. This favors low CPU over maximum compression.
  scan = b"".join(b"\x00" + a[y].tobytes() for y in range(h))
  ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
  return b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", ihdr) + _png_chunk(b"IDAT", zlib.compress(scan, compress_level)) + _png_chunk(b"IEND", b"")


def write_png_atomic(path: Path, rgb: np.ndarray) -> None:
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_suffix(path.suffix + ".tmp")
  tmp.write_bytes(rgb_to_png_bytes(rgb))
  tmp.replace(path)
