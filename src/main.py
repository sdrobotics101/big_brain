import time
import sys
from bitstring import BitArray
from ctypes import sizeof
sys.path.append("./dependencies/python-shared-buffers/shared_buffers/")

import pydsm

from master import ControlInput, SensorReset
from sensor import Angular, Linear
from navigation import Kill
from serialization import pack, unpack
from constants import Sensor, Motor, Vision, Master, ServerID, PiIP, Axes, Quaternion, LocationType, Goal
from vision import Detection, DetectionArray
from settings import *

from condition import Condition

controlInput = ControlInput()
sensorReset = SensorReset()
angular = Angular()
linear = Linear()
kill = Kill()
detections = DetectionArray()

controlInput.angular[Axes.xaxis].pos[0] = 0
controlInput.angular[Axes.xaxis].pos[1] = 0

controlInput.angular[Axes.yaxis].pos[0] = 0
controlInput.angular[Axes.yaxis].pos[1] = 0

controlInput.angular[Axes.zaxis].pos[0] = 0
controlInput.angular[Axes.zaxis].pos[1] = 0

controlInput.linear[Axes.xaxis].vel = 0
controlInput.linear[Axes.yaxis].vel = 0
controlInput.linear[Axes.zaxis].pos[0] = 0
controlInput.linear[Axes.zaxis].pos[1] = 0

VEL = 0
POS = 1
mode = [0,0,POS,VEL,VEL,POS,POS,POS]
controlInput.mode = BitArray(mode).uint

sensorReset.pos[Axes.xaxis] = 0
sensorReset.pos[Axes.yaxis] = 0
sensorReset.pos[Axes.zaxis] = 0
sensorReset.reset      = False

for i in range(3):
    angular.pos[i] = 0
    angular.vel[i] = 0
    angular.acc[i] = 0
    linear.pos[i]  = 0
    linear.vel[i]  = 0
    linear.acc[i]  = 0
angular.pos[0] = 1
angular.pos[3] = 0

kill.isKilled = True

for i in range(8):
    detections.detections[i].x = 0
    detections.detections[i].y = 0
    detections.detections[i].size = 0
    detections.detections[i].type = 0

CLIENT_ID = 100
client = pydsm.Client(42, 100, True)

client.registerLocalBuffer("control", sizeof(ControlInput), False)
client.registerLocalBuffer("sensorreset", sizeof(SensorReset),  False)
time.sleep(0.5)
client.setLocalBufferContents("control", pack(controlInput))
client.setLocalBufferContents("sensorreset", pack(sensorReset))
print("Created local buffers: control, sensorreset")

client.registerRemoteBuffer("angular", "10.0.0.43", 43)
client.registerRemoteBuffer("linear", "10.0.0.43", 43)
client.registerRemoteBuffer("kill", "10.0.0.44", 44)
client.registerRemoteBuffer("forwarddetection", "10.0.0.45", 45)
time.sleep(0.5)
print("Registered remote buffers: angular,linear,kill,targetlocation")

def is_at_target_depth():
    return abs(controlInput.linear[Axes.zaxis].pos[0] - linear.pos[Axes.zaxis]) < DEPTH_TOLERANCE
at_depth = Condition(is_at_target_depth, 10, 1, False)

def is_at_target_heading():
    return abs(controlInput.angular[Axes.zaxis].pos[0] - angular.acc[Axes.zaxis]) < HEADING_TOLERANCE
at_heading = Condition(is_at_target_heading, 10, 1, False)

def is_settled():
    return at_depth.get_value() and at_heading.get_value()
settled = Condition(is_settled, 1, 1, False)

killed = Condition(lambda: kill.isKilled, 1, 1, True)
has_detection = Condition(lambda: detections.detections[0].type != -1, 1, 1, False)
too_close_to_detection = Condition(lambda: has_detection.get_value() and \
                                           detections.detections[0].size > TARGET_AREA, \
                                           5, \
                                           5, \
                                           False)

def update_buffers():
    angularData, ang_active = client.getRemoteBufferContents("angular", "10.0.0.43", 43)
    linearData, lin_active = client.getRemoteBufferContents("linear", "10.0.0.43", 43)
    killData, kill_active = client.getRemoteBufferContents("kill", "10.0.0.44", 44)
    detectData, det_active = client.getRemoteBufferContents("forwarddetection", "10.0.0.45", 45)
    if not ang_active:
        print("WARNING: sensor angular not active")
    if not lin_active:
        print("WARNING: sensor linear not active")
    if not kill_active:
        print("WARNING: navigation kill not active")
    if not det_active:
        print("WARNING: forward detect not active")
    angular = unpack(Angular, angularData)
    linear = unpack(Linear, linearData)
    kill = unpack(Kill, killData)
    detections = unpack(DetectionArray, detectData)

was_killed = True
heading = 0
while True:
    try:
        time.sleep(0.1)
        update_buffers()
        at_depth()
        at_heading()
        settled()
        killed()
        has_detection()
        too_close_to_detection()

        if killed.get_value():
            was_killed = True
            controlInput.angular[Axes.zaxis].pos[0] = 0
            controlInput.linear[Axes.xaxis].vel = 0
            controlInput.linear[Axes.zaxis].pos[0] = 0
            client.setLocalBufferContents("control", pack(controlInput))
            continue
        if was_killed:
            was_killed = False
            heading = angular.acc[Axes.zaxis]
            controlInput.angular[Axes.zaxis].pos[0] = heading
            controlInput.linear[Axes.zaxis].pos[0] = TARGET_DEPTH
            client.setLocalBufferContents("control", pack(controlInput))
            print("unkilled, heading: ", heading)
            continue

        if too_close_to_detection().get_value():
            controlInput.linear[Axes.xaxis].vel = MOVE_BACK_VEL
            print("too close to target")
        else:
            controlInput.linear[Axes.xaxis].vel = 0

        if settled.get_value() and has_detection.get_value():
            if detections[0].x < (0.5 - VISION_X_TOLERANCE):
                heading = heading - HEADING_CHANGE
            if detections[0].x > (0.5 + VISION_X_TOLERANCE):
                heading = heading + HEADING_CHANGE
            controlInput.angular[Axes.zaxis].pos[0] = heading
            print("target heading: ", heading)

        client.setLocalBufferContents("control", pack(controlInput))

    except KeyboardInterrupt:
        print("Caught control-C, exiting")
        break