#!/usr/bin/env python3
BUILD_VERSION = 31
BUILD_TAG = 'v31-canonical360-imm-identity-fastui-monitor'
LOGGER_FORMAT_VERSION = 12
KALMAN_API_VERSION = 3
IMM_API_VERSION = 1
ANDROID_PROTOCOL_VERSION = 14

# Display/reference coordinate convention. Decoded object x values are preserved
# to keep the empirically validated SCC/rear-teacher calibration unchanged.
COORDINATE_X_ORIGIN = 'ego_front_bumper_display_reference'
EGO_DISPLAY_LENGTH_M = 4.8
EGO_DISPLAY_CENTER_X_M = -EGO_DISPLAY_LENGTH_M / 2.0
OBJECT_X_ADJUSTMENT_M = 0.0
