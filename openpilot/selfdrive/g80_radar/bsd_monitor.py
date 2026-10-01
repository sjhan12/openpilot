"""Receive-only carState BSD adapter. No CAN decoding or control writes."""
from __future__ import annotations

FRESH_NS = 500_000_000
CLEAR_NS = 400_000_000


class BsdMonitor:
  def __init__(self):
    self.recv_ns = 0
    self.valid = False
    self.supported = None
    self.ever_on = False
    self.raw = {'left': None, 'right': None}
    self.block = {'left': False, 'right': False}
    self.clear_since = {'left': 0, 'right': 0}
    self.error = 'waiting_carState'
    self.next_params_ns = 0

  def refresh_support(self, now_ns):
    if now_ns < self.next_params_ns:
      return
    self.next_params_ns = now_ns + 5_000_000_000
    try:
      from openpilot.common.params import Params
      data = Params().get('CarParams')
      if not data:
        return
      try:
        from opendbc.car import structs
        schema = structs.CarParams
      except ImportError:
        from openpilot.cereal import car
        schema = car.CarParams
      with schema.from_bytes(data) as cp:
        self.supported = bool(cp.enableBsm)
    except Exception:
      # Missing schema/support must not silently mean BSD OFF.
      self.supported = None

  def update(self, cs, recv_ns, source_ns, event_valid=True):
    previous = self.recv_ns
    self.recv_ns = int(recv_ns)
    try:
      vals = {'left': bool(cs.leftBlindspot), 'right': bool(cs.rightBlindspot)}
      valid = bool(event_valid and cs.canValid and
                   -50_000_000 <= recv_ns-source_ns <= FRESH_NS)
    except Exception:
      vals = {'left': None, 'right': None}
      valid = False
    self.raw = vals
    self.valid = valid
    self.error = '' if valid else 'invalid_or_missing_carState_BSD'
    if not valid:
      self.clear_since = {'left': 0, 'right': 0}
      return
    for side, on in vals.items():
      if on:
        self.ever_on = True
        self.block[side] = True
        self.clear_since[side] = 0
      else:
        if not previous or recv_ns-previous > FRESH_NS:
          self.clear_since[side] = 0
        if not self.clear_since[side]:
          self.clear_since[side] = recv_ns
        if recv_ns-self.clear_since[side] >= CLEAR_NS:
          self.block[side] = False

  def snapshot(self, now_ns):
    fresh = bool(self.valid and self.recv_ns and 0 <= now_ns-self.recv_ns <= FRESH_NS)
    available = bool(fresh and (self.supported is True or self.ever_on))
    return dict(source='carState.leftBlindspot/rightBlindspot',
                fresh=fresh, available=available, enable_bsm=self.supported,
                observed_on=self.ever_on, recv_ns=self.recv_ns,
                age_ms=None if not self.recv_ns else round((now_ns-self.recv_ns)/1e6,1),
                raw=dict(self.raw), blocked=dict(self.block),
                state={s:('ON' if self.block[s] else 'OFF' if available else 'UNKNOWN')
                       for s in ('left','right')},
                error=self.error or ('' if available else 'BSD_support_or_freshness_unconfirmed'))
