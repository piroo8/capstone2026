"""Shared configuration for the parking_enforcement ROS 2 package.

Edit this file first when you move between demo setups.  The package keeps the
parking mission separate from the old flight3 package, but it preserves the same
service interface and general offboard-control flow.
"""

from pathlib import Path

# Drone namespace and frame assumptions.
DRONE_NS = 'rob498_drone_8'
FRAME_ID = 'map'
GRID_FRAME = 'base_link'
STEREO_FRAME_ID = 'camera_fisheye1_optical_frame'
CAMERA_BODY_ALIGNED = True

# Hardcoded parking waypoints for the current demo phase.
# Replace these tuples with your real parking-spot coordinates before flight.
PARKING_WAYPOINTS = [
    (-1.0, -1.0, 1.0),
    (1.0, -1.0, 2.0),
    (1.0, 1.0, 1.0),
    (-1.0, 1.0, 0.5),
]

# Mission behavior.
LAUNCH_ALT = 0.5
TARGET_RADIUS = 0.20
POSITION_HOLD_RADIUS = 0.12
WAYPOINT_SETTLE_TIME = 1.0
MISSION_RATE_HZ = 20.0
Z_OFFSET = 0.0

# Reactive avoidance behavior.
AVOID_STEP = 0.60
AVOID_FORWARD_STEP = 0.20
BACK_UP_STEP = 0.25
AVOID_TIMEOUT_SEC = 4.0

# Scan behavior at each parking spot.
SCAN_YAW_RIGHT_DEG = 90.0
SCAN_AMPLITUDE = 0.18
SCAN_PERIOD_SEC = 4.0
SCAN_TIMEOUT_SEC = 12.0

# Revisit / overstay behavior.
REVISIT_OVERSTAY_THRESHOLD_SEC = 120.0

# Shared log locations.
LOG_DIR = Path.home() / '.ros' / 'parking_enforcement'
FIRST_PASS_LOG_PATH = LOG_DIR / 'plate_log.json'
REVISIT_LOG_PATH = LOG_DIR / 'revisit_log.json'

# MAVROS topics and services.
LOCAL_POSE_TOPIC = 'mavros/local_position/pose'
SETPOINT_TOPIC = 'mavros/setpoint_position/local'
MAVROS_STATE_TOPIC = 'mavros/state'
MAVROS_ARM_TOPIC = 'mavros/cmd/arming'
MAVROS_SET_MODE_TOPIC = 'mavros/set_mode'

# Stereo camera topics.
LEFT_CAMERA_INFO_TOPIC = '/camera/fisheye1/camera_info'
RIGHT_CAMERA_INFO_TOPIC = '/camera/fisheye2/camera_info'
LEFT_IMAGE_TOPIC = '/camera/fisheye1/image_raw'
RIGHT_IMAGE_TOPIC = '/camera/fisheye2/image_raw'

# Depth topics produced by the parking package stereo node.
DEPTH_TOPIC = '/stereo/depth'
DEPTH_INFO_TOPIC = '/stereo/depth/camera_info'
DISPARITY_VIS_TOPIC = '/stereo/disp_vis'

# Plate-reader topics.
RGB_IMAGE_TOPIC = '/camera/image_raw'
PLATE_ENABLED_TOPIC = '/plate_reader/enabled'
PLATE_CONFIRMED_TOPIC = '/plate_reader/confirmed'
PLATE_CURRENT_TOPIC = '/plate_reader/current'
PLATE_DEBUG_IMAGE_TOPIC = '/plate_reader/debug_image'

# Local grid topics.
LOCAL_GRID_TOPIC = '/local_grid'
AVOID_TOPIC = '/avoid_dir'

# TensorRT engine locations.
LPD_ENGINE_PATH = '/home/jetson/engines/lpdnet_fp32.engine'
LPR_ENGINE_PATH = '/home/jetson/engines/lprnet_fp32.engine'

# Stereo depth settings.
MIN_DEPTH_METERS = 0.20
MAX_DEPTH_METERS = 4.00
SKIP_FRAMES = 3
NUM_DISPARITIES = 64
VERTICAL_OFFSET_PIXELS = -2
DOWNSCALE = 0.5

# Left-side crop handling.
# Set LEFT_CROP_OVERRIDE_PIXELS to a positive integer if you want to force the
# crop width instead of estimating it from the rectification overlap.
LEFT_CROP_OVERRIDE_PIXELS = None
LEFT_CROP_MIN_OVERLAP_RATIO = 0.85
LEFT_CROP_EXTRA_PADDING_PIXELS = 2
MIN_PUBLISHED_WIDTH_PIXELS = 32

# Occupancy-grid settings.
GRID_X_MIN = 0.0
GRID_X_MAX = 3.0
GRID_Y_MIN = -1.5
GRID_Y_MAX = 1.5
GRID_RESOLUTION = 0.10
POINT_MIN_DEPTH = 0.25
POINT_MAX_DEPTH = 3.0
POINT_STRIDE = 6
POINT_Z_MIN = -0.80
POINT_Z_MAX = 0.80

# Sector windows used for avoidance decisions.
SECTOR_X_MIN = 0.40
SECTOR_X_MAX = 1.40
CENTER_HALF_WIDTH = 0.35
SIDE_HALF_WIDTH = 1.20
CENTER_OCC_THRESHOLD = 0.10
SIDE_OCC_THRESHOLD = 0.12
CENTER_MIN_KNOWN_RATIO = 0.35
SIDE_MIN_KNOWN_RATIO = 0.20
HYSTERESIS_LEN = 5

# Plate-reader behavior.
LPD_W = 640
LPD_H = 480
LPR_W = 96
LPR_H = 48
CHARS = '0123456789ABCDEFGHIJKLMNPQRSTUVWXYZ'
BLANK = 35
DETECT_EVERY = 6
PAD_LEFT = 15
PAD_RIGHT = 0
PAD_TOP = 10
PAD_BOTTOM = 10
MIN_DET_CONF = 0.60
MIN_READ_CONF = 0.45
MIN_VOTES = 10
VOTE_WINDOW = 40
