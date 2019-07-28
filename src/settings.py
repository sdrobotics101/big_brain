from master import ControlInput, SensorReset
from sensor import Angular, Linear
from navigation import Kill
from serialization import pack, unpack
from constants import Sensor, Motor, Vision, Master, ServerID, PiIP, Axes, Quaternion, LocationType, Goal
from vision import Detection, DetectionArray

CLIENT_ID = 9

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

STARTING_DEPTH = 0.35 # meters
TARGET_DEPTH = 0.5 # meters

MOVE_BACK_VEL = -50
HEADING_CHANGE = 1

VISION_X_TOLERANCE = 0.05

CENTER_POLE_HEIGHT_MULTIPLIER = 0.75

LINED_UP_THRESH = 5
HEADING_THRESH = 3

RUN_AVG_THRESH = 0.3

### GATE CONSTANTS ###
GATE_TARGET_DEPTH = 0.5 # meters
GATE_VELOCITY = 35
GATE_FAST_VELOCITY = 150
# true if 40% side is on the left, false otherwise
GATE_40_LEFT = False

remote_buffers = [
    ("angular", SEN_IP, SEN_SID, Angular),
    ("linear", SEN_IP, SEN_SID, Linear),
    ("kill", NAV_IP, NAV_SID, Kill),
    ("forwarddetection", FOR_IP, FOR_SID, DetectionArray),
]
