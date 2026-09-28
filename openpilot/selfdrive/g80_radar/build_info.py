#!/usr/bin/env python3
BUILD_VERSION = 33
BUILD_TAG = 'v33-canonical360-imm2-future-gap-occupancy-monitor'
LOGGER_FORMAT_VERSION = 14
KALMAN_API_VERSION = 3
IMM_API_VERSION = 2
FUTURE_GAP_API_VERSION = 1
ANDROID_PROTOCOL_VERSION = 16

# Display/reference coordinate convention. Decoded object x values are preserved
# to keep the empirically validated SCC/rear-teacher calibration unchanged.
COORDINATE_X_ORIGIN = 'ego_front_bumper_display_reference'
EGO_DISPLAY_LENGTH_M = 4.8
EGO_DISPLAY_CENTER_X_M = -EGO_DISPLAY_LENGTH_M / 2.0
OBJECT_X_ADJUSTMENT_M = 0.0
