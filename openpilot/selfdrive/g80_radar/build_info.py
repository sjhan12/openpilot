#!/usr/bin/env python3
BUILD_VERSION = 32
BUILD_TAG = 'v32-canonical360-imm-realtime-l1handoff-monitor'
LOGGER_FORMAT_VERSION = 13
KALMAN_API_VERSION = 3
IMM_API_VERSION = 2
ANDROID_PROTOCOL_VERSION = 15

# Display/reference coordinate convention. Decoded object x values are preserved
# to keep the empirically validated SCC/rear-teacher calibration unchanged.
COORDINATE_X_ORIGIN = 'ego_front_bumper_display_reference'
EGO_DISPLAY_LENGTH_M = 4.8
EGO_DISPLAY_CENTER_X_M = -EGO_DISPLAY_LENGTH_M / 2.0
OBJECT_X_ADJUSTMENT_M = 0.0
