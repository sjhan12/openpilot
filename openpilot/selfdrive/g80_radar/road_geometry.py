#!/usr/bin/env python3
"""C4 modelV2 road geometry helpers for G80 radar monitor V21.

MONITOR-ONLY: this module only reads modelV2 geometry and annotates display
objects. It does not publish radarTracks/radarState and never sends CAN.

Coordinate convention used by g80_radar:
  x: forward from ego [m]
  y: left positive [m]
openpilot modelV2 uses the opposite sign for lateral y in the UI/radard paths,
so model y is negated when copied into this module.
"""
from __future__ import annotations

import bisect
import math
from typing import Iterable

LANE_W_M = 3.6
MAX_MODEL_POINTS = 65
ROAD_MODEL_MAX_AGE_NS = 700_000_000
# Do not project a radar object far beyond the C4 path horizon. V20 used the
# last path segment endpoint and could create impossible lane-28/lane-40 tags.
PATH_PROJECTION_MARGIN_M = 4.0
PATH_BACK_MARGIN_M = 2.0
MAX_PROJECTION_D_M = 12.5


def _finite(v, default=math.nan):
  try:
    f = float(v)
  except Exception:
    return default
  return f if math.isfinite(f) else default


def _sample_xy(xs: Iterable, ys: Iterable, max_points: int = MAX_MODEL_POINTS) -> list[dict]:
  xs = list(xs)
  ys = list(ys)
  n = min(len(xs), len(ys))
  if n <= 0:
    return []
  step = max(1, math.ceil(n / max_points))
  out = []
  for i in range(0, n, step):
    x = _finite(xs[i])
    y = -_finite(ys[i])  # model convention -> radar/display convention (+left)
    if math.isfinite(x) and math.isfinite(y) and -5.0 <= x <= 250.0 and abs(y) <= 80.0:
      out.append({'x': round(x, 4), 'y': round(y, 4)})
  return out


def _path_y(path: list[dict], x: float, clamp: bool = True) -> float | None:
  if not path:
    return None
  if len(path) == 1:
    return float(path[0]['y'])
  xs = [float(p['x']) for p in path]
  if not clamp and (x < xs[0] or x > xs[-1]):
    return None
  i = bisect.bisect_left(xs, x)
  if i <= 0:
    return float(path[0]['y']) if clamp else None
  if i >= len(path):
    return float(path[-1]['y']) if clamp else None
  p0, p1 = path[i - 1], path[i]
  dx = float(p1['x']) - float(p0['x'])
  if abs(dx) < 1e-6:
    return float(p0['y'])
  t = (x - float(p0['x'])) / dx
  return float(p0['y']) + (float(p1['y']) - float(p0['y'])) * t


def extract_road_model(model, recv_ns: int, max_points: int = MAX_MODEL_POINTS) -> dict:
  """Copy path/lane/edge geometry out of a modelV2 message into JSON-safe data."""
  try:
    path = _sample_xy(model.position.x, model.position.y, max_points)
  except Exception:
    path = []

  try:
    probs = list(model.laneLineProbs)
  except Exception:
    probs = []

  lane_lines = []
  try:
    for i, line in enumerate(model.laneLines):
      pts = _sample_xy(line.x, line.y, max_points)
      if not pts:
        continue
      prob = _finite(probs[i], 0.0) if i < len(probs) else 0.0
      lane_lines.append({'index': i, 'prob': round(max(0.0, min(1.0, prob)), 4), 'points': pts})
  except Exception:
    lane_lines = []

  road_edges = []
  try:
    for i, edge in enumerate(model.roadEdges):
      pts = _sample_xy(edge.x, edge.y, max_points)
      if pts:
        road_edges.append({'index': i, 'points': pts})
  except Exception:
    road_edges = []

  y20 = _path_y(path, 20.0)
  y40 = _path_y(path, 40.0)
  y60 = _path_y(path, 60.0)
  ref_y = y40 if y40 is not None else (y20 if y20 is not None else 0.0)
  if abs(ref_y) < 0.35:
    curve = 'STRAIGHT'
  else:
    curve = 'LEFT' if ref_y > 0.0 else 'RIGHT'

  valid_lines = sum(1 for ln in lane_lines if ln['prob'] >= 0.35)
  path_min = min((float(p['x']) for p in path), default=None)
  path_max = max((float(p['x']) for p in path), default=None)
  return {
    'valid': len(path) >= 2,
    'recv_ns': int(recv_ns),
    'path': path,
    'lane_lines': lane_lines,
    'lane_line_probs': [round(max(0.0, min(1.0, _finite(p, 0.0))), 4) for p in probs],
    'road_edges': road_edges,
    'confident_lane_lines': valid_lines,
    'path_x_min_m': None if path_min is None else round(path_min, 3),
    'path_x_max_m': None if path_max is None else round(path_max, 3),
    'path_projection_margin_m': PATH_PROJECTION_MARGIN_M,
    'path_y_20m': None if y20 is None else round(y20, 3),
    'path_y_40m': None if y40 is None else round(y40, 3),
    'path_y_60m': None if y60 is None else round(y60, 3),
    'curve_direction': curve,
  }


def road_model_with_age(road_model: dict | None, now_ns: int) -> dict:
  if not road_model:
    return {'valid': False, 'fresh': False, 'age_ms': None, 'path': [], 'lane_lines': [], 'road_edges': [],
            'path_x_min_m': None, 'path_x_max_m': None, 'lane_line_probs': []}
  out = dict(road_model)
  recv_ns = int(out.get('recv_ns', 0) or 0)
  age_ns = int(now_ns) - recv_ns if recv_ns > 0 else ROAD_MODEL_MAX_AGE_NS + 1
  out['age_ms'] = round(age_ns / 1e6, 1) if recv_ns > 0 else None
  out['fresh'] = bool(out.get('valid')) and -50_000_000 <= age_ns <= ROAD_MODEL_MAX_AGE_NS
  return out


def path_as_tuples(road_model: dict | None) -> list[tuple[float, float]]:
  if not road_model:
    return []
  return [(float(p['x']), float(p['y'])) for p in road_model.get('path', [])]


def path_y_at_x(road_model: dict | None, x: float) -> float | None:
  if not road_model:
    return None
  return _path_y(road_model.get('path', []), float(x), clamp=False)


def _path_coverage(path: list[dict]) -> tuple[float, float] | None:
  if len(path) < 2:
    return None
  xs = [float(p['x']) for p in path]
  return min(xs), max(xs)


def project_to_path(x: float, y: float, road_model: dict | None) -> dict | None:
  """Project a radar Cartesian point onto the C4 path and return Frenet-like s,d.

  V21 refuses projection outside the C4 path horizon (except a small margin),
  preventing far targets from being snapped onto the last path endpoint.
  d is positive to the left. s is arc length from the first model point.
  """
  if not road_model or not road_model.get('fresh', road_model.get('valid', False)):
    return None
  path = road_model.get('path', [])
  coverage = _path_coverage(path)
  if coverage is None:
    return None
  xmin, xmax = coverage
  px, py = float(x), float(y)
  if px < xmin - PATH_BACK_MARGIN_M or px > xmax + PATH_PROJECTION_MARGIN_M:
    return None

  best = None
  s_acc = 0.0
  for i in range(len(path) - 1):
    x0, y0 = float(path[i]['x']), float(path[i]['y'])
    x1, y1 = float(path[i + 1]['x']), float(path[i + 1]['y'])
    dx, dy = x1 - x0, y1 - y0
    seg2 = dx * dx + dy * dy
    if seg2 < 1e-8:
      continue
    seg = math.sqrt(seg2)
    t = ((px - x0) * dx + (py - y0) * dy) / seg2
    tc = max(0.0, min(1.0, t))
    qx, qy = x0 + tc * dx, y0 + tc * dy
    ex, ey = px - qx, py - qy
    dist2 = ex * ex + ey * ey
    cross = dx * ey - dy * ex
    d = math.copysign(math.sqrt(dist2), cross) if dist2 > 0.0 else 0.0
    s = s_acc + tc * seg
    candidate = (dist2, s, d, qx, qy, i)
    if best is None or candidate[0] < best[0]:
      best = candidate
    s_acc += seg

  if best is None:
    return None
  dist2, s, d, qx, qy, seg_i = best
  if math.sqrt(dist2) > MAX_PROJECTION_D_M:
    return None
  return {
    's': round(s, 3),
    'd': round(d, 3),
    'path_x': round(qx, 3),
    'path_y': round(qy, 3),
    'distance_to_path_m': round(math.sqrt(dist2), 3),
    'segment': int(seg_i),
  }


def lane_index_from_d(d: float) -> int:
  return int(round(float(d) / LANE_W_M))


def lane_name(i: int) -> str:
  return {0: 'ego', 1: 'left1', -1: 'right1', 2: 'left2', -2: 'right2'}.get(i, f'lane{i:+d}')


def annotate_object(o: dict, road_model: dict | None) -> dict:
  dct = dict(o)
  try:
    x, y = float(dct['x']), float(dct['y'])
  except Exception:
    return dct

  # C4 position is a forward road model. Keep rear-zone classification in raw ego y.
  if x < -0.5:
    dct['road_s'] = None
    dct['road_d'] = round(y, 3)
    dct['road_path_x'] = None
    dct['road_path_y'] = None
    dct['road_lane_index'] = lane_index_from_d(y)
    dct['road_lane'] = lane_name(dct['road_lane_index'])
    dct['road_lane_source'] = 'raw_y_rear'
    dct['road_projection_valid'] = False
    return dct

  projection = project_to_path(x, y, road_model)
  if projection is not None:
    dct['road_s'] = projection['s']
    dct['road_d'] = projection['d']
    dct['road_path_x'] = projection['path_x']
    dct['road_path_y'] = projection['path_y']
    dct['road_lane_index'] = lane_index_from_d(projection['d'])
    dct['road_lane'] = lane_name(dct['road_lane_index'])
    dct['road_lane_source'] = 'c4_path'
    dct['road_projection_valid'] = True
    return dct

  # With a fresh C4 model, an object outside its forward coverage is deliberately
  # left UNCLASSIFIED rather than assigning a nonsense far-away lane number.
  fresh = bool((road_model or {}).get('fresh'))
  xmax = (road_model or {}).get('path_x_max_m')
  if fresh and xmax is not None and x > float(xmax) + PATH_PROJECTION_MARGIN_M:
    source = 'c4_path_out_of_range'
  elif fresh:
    source = 'c4_path_no_projection'
  else:
    source = 'raw_y_fallback'

  dct['road_s'] = None
  dct['road_path_x'] = None
  dct['road_path_y'] = None
  dct['road_projection_valid'] = False
  if source == 'raw_y_fallback':
    dct['road_d'] = round(y, 3)
    dct['road_lane_index'] = lane_index_from_d(y)
    dct['road_lane'] = lane_name(dct['road_lane_index'])
  else:
    dct['road_d'] = None
    dct['road_lane_index'] = None
    dct['road_lane'] = None
  dct['road_lane_source'] = source
  return dct


def annotate_objects(objects: list[dict], road_model: dict | None) -> list[dict]:
  return [annotate_object(o, road_model) for o in objects]
