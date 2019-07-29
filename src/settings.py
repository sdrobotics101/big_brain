from master import ControlInput, SensorReset
from sensor import Angular, Linear
from navigation import Kill
from serialization import pack, unpack
from constants import Sensor, Motor, Vision, Master, ServerID, PiIP, Axes, Quaternion, LocationType, Goal
from vision import Detection, DetectionArray

CLIENT_ID = 97

SEN_IP = "10.0.0.43"
NAV_IP = "10.0.0.44"
FOR_IP = "10.0.0.45"
DOW_IP = "10.0.0.46"

SEN_SID = 43
NAV_SID = 44
FOR_SID = 45
DOW_SID = 46

HEADING_TOLERANCE = 8 # degrees
DEPTH_TOLERANCE = 0.1 # meters

STARTING_DEPTH = 0.5 # meters

VISION_X_TOLERANCE = 0.05

### DETECTION_UTILS CONSTANTS ###
CENTER_POLE_HEIGHT_MULTIPLIER = 0.75

### GATE CONSTANTS ###
GATE_TARGET_DEPTH = 0.5 # meters
GATE_VELOCITY = 35
GATE_FAST_VELOCITY = 100
GATE_HEADING_ADJUST = 1
# true if 40% side is on the left, false otherwise
GATE_40_LEFT = False

### BUOY CONSTANTS ###
BUOY_TARGET_DEPTH = 0.5
BUOY_VELOCITY = 25
BUOY_HEADING_ADJUST = 2.5
BUOY_SIZE_THRESH = 0.05
BUOY_START_TURN = 0 # degrees

BUOY_TOUCH_VELOCITY = 70
BUOY_TOUCH_TIME = 4000 # milliseconds
BUOY_STOP_TIME = 2000 # milliseconds
BUOY_BACKOFF_VELOCITY = -150
BUOY_BACKOFF_TIME = 10000 # milliseconds

### VISION CONSTANTS ###
FOR_ASWANG = 0
FOR_DRAUGR = 1
FOR_VETALAS = 2
FOR_JIANGSHI = 3
FOR_GATE = 4
FOR_BIN = 5
FOR_MARKER = 6
DOW_MARKER = 0
DOW_CHECKER = 1
DOW_WOLF = 2
DOW_BAT = 3

remote_buffers = [
    ("angular", SEN_IP, SEN_SID, Angular),
    ("linear", SEN_IP, SEN_SID, Linear),
    ("kill", NAV_IP, NAV_SID, Kill),
    ("forwarddetection", FOR_IP, FOR_SID, DetectionArray),
]
