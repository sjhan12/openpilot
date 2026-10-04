#!/usr/bin/env python3
"""V50 shadow logger compatibility shim.

V50 is ML-case-only. Continuous shadow logging is hard-disabled in code.
This module keeps the old ShadowLogger API so live_service remains compatible,
but it never creates, opens, writes, rotates, flushes, or renames any file.
"""
from __future__ import annotations
from openpilot.selfdrive.g80_radar.build_info import BUILD_VERSION

LOGGER_SERVICE_VERSION = BUILD_VERSION
HARD_DISABLED = True

class ShadowLogger:
  def __init__(self, *args, **kwargs):
    self.enabled = False
    self.files_created = 0
    self.records = 0
    self.event_records = 0
    self.last_write_ns = 0
    self.last_error = ''

  def maybe_write(self, *args, **kwargs):
    return False

  def close(self):
    return None

  def status(self) -> dict:
    return {
      'enabled': False,
      'hard_disabled': True,
      'service_version': LOGGER_SERVICE_VERSION,
      'files_created': 0,
      'records': 0,
      'event_records': 0,
      'last_write_ns': 0,
      'path': None,
      'last_error': '',
      'policy': 'V50 ML-only: continuous shadow log permanently disabled',
    }
