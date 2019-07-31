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
GATE_VELOCITY = 70
GATE_HEADING_ADJUST = 2
# true if 40% side is on the left, false otherwise
GATE_40_LEFT = False
GATE_THROUGH_CYCLES = 400 # 100 = 1 seconds

### GATE_TO_BUOY CONSTANTS ###
G2B_TARGET_DEPTH = 1 # meters
G2B_INITIAL_TURN_AMOUNT = -25 # degrees
G2B_FORWARD_VELOCITY = 70
G2B_FORWARD_TIME = 25000 # milliseconds
G2B_FINAL_TURN_AMOUNT = 180 # degrees

### FIND BUOY CONSTANTS ###
BUOY_TARGET_DEPTH = 2.5
BUOY_VELOCITY = 35
BUOY_HEADING_ADJUST = 2
BUOY_SIZE_THRESH = 0.05
BUOY_TRIANGLE_LEFT = True

### TOUCH_BUOY CONSTANTS ###
BUOY_TOUCH_VELOCITY = 70
BUOY_TOUCH_TIME = 6000 # milliseconds
BUOY_STOP_TIME = 2000 # milliseconds
BUOY_BACKOFF_VELOCITY = -100
BUOY_BACKOFF_TIME = 15000 # milliseconds

### VISION CONSTANTS ###
ASWANG = 0
DRAUGR = 1
VETALAS = 2
JIANGSHI = 3
GATE = 4
BIN = 5
FOR_MARKER = 6
DOW_MARKER = 0
CHECKER = 1
WOLF = 2
BAT = 3

remote_buffers = [
    ("angular", SEN_IP, SEN_SID, Angular),
    ("linear", SEN_IP, SEN_SID, Linear),
    ("kill", NAV_IP, NAV_SID, Kill),
    ("forwarddetection", FOR_IP, FOR_SID, DetectionArray),
]
