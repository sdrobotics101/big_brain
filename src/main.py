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

def is_settled():
    return abs(controlInput.angular[Axes.zaxis].pos[0] - angular.acc[Axes.zaxis]) < 3 and \
           abs(controlInput.linear[Axes.zaxis].pos[0] - linear.pos[Axes.zaxis]) < 0.1
settled = Condition(is_settled, 10, 1, False)


