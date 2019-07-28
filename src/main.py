import time
import sys
from bitstring import BitArray
from ctypes import sizeof
sys.path.append("./dependencies/python-shared-buffers/shared_buffers/")

import pydsm

from master import ControlInput, SensorReset, Status
from sensor import Angular, Linear
from navigation import Kill
from serialization import pack, unpack
from constants import Sensor, Motor, Vision, Master, ServerID, PiIP, Axes, Quaternion, LocationType, Goal
from vision import Detection, DetectionArray
import settings
from buffers import (
    register_remote_buffers,
    update_remote_buffers
)
from detection_utils import active_detections

from condition import Condition

from state import KilledState, Gate

controlInput = ControlInput()
sensorReset = SensorReset()
status = Status()

sensorReset.pos[Axes.xaxis] = 0
sensorReset.pos[Axes.yaxis] = 0
sensorReset.pos[Axes.zaxis] = 0
sensorReset.reset      = False

status.macrostate = 0
status.microstate = 0

conditions = {}

def control(args):
    return args[0]

def data(args):
    return args[1]

def is_at_target_depth(args):
    setpoint = control(args).linear[Axes.zaxis].pos[0]
    actual = data(args)["linear"].pos[Axes.zaxis]
    return abs(setpoint - actual) < settings.DEPTH_TOLERANCE
at_depth = Condition(is_at_target_depth, 50, 30, False)
conditions["at_depth"] = at_depth

def is_at_target_heading(args):
    setpoint = control(args).angular[Axes.zaxis].pos[0]
    actual = data(args)["angular"].pos[Axes.zaxis]
    return abs(setpoint - actual) < settings.HEADING_TOLERANCE
at_heading = Condition(is_at_target_heading, 30, 10, False)
conditions["at_heading"] = at_heading

def is_settled(args):
    return at_depth(args) and at_heading(args)
settled = Condition(is_settled, 1, 1, False)
conditions["settled"] = settled

def is_killed(args):
    return data(args)["kill"].isKilled
killed = Condition(is_killed, 1, 1, True)
conditions["killed"] = killed

def active_detection(args):
    dets = data(args)["forwarddetection"].detections
    return active_detections(dets) > 0
has_active_detection = Condition(active_detection, 1, 1, False)
conditions["has_active_detection"] = has_active_detection

prev_x = 0
def new_frame(args):
    dets = data(args)["forwarddetection"].detections
    global prev_x
    if has_active_detection(args) and dets[0].x != prev_x:
        prev_x = dets[0].x
        return True
    return False
has_new_frame = Condition(new_frame, 1, 1, False)
conditions["has_new_frame"] = has_new_frame

def new_interesting_frame(args):
    return has_new_frame(args) and has_active_detection(args)
interesting_frame = Condition(new_interesting_frame, 1, 1, False)
conditions["interesting_frame"] = interesting_frame

def is_below_starting_depth(args):
    return data(args)["linear"].pos[Axes.zaxis] > settings.STARTING_DEPTH
below_starting_depth = Condition(is_below_starting_depth, 100, 1, False)
conditions["below_starting_depth"] = below_starting_depth

def set_control(client, heading, depth, velocity):
    control = ControlInput()
    control.angular[Axes.xaxis].pos[0] = 0
    control.angular[Axes.xaxis].pos[1] = 0
    control.angular[Axes.yaxis].pos[0] = 0
    control.angular[Axes.yaxis].pos[1] = 0
    control.angular[Axes.zaxis].pos[0] = heading
    control.angular[Axes.zaxis].pos[1] = 0
    control.linear[Axes.xaxis].vel = velocity
    control.linear[Axes.yaxis].vel = 0
    control.linear[Axes.zaxis].pos[0] = depth
    control.linear[Axes.zaxis].pos[1] = 0
    VEL = 0
    POS = 1
    mode = [0,0,POS,VEL,VEL,POS,POS,POS]
    control.mode = BitArray(mode).uint
    client.setLocalBufferContents("control", pack(control))
    return control

def main():
    client = pydsm.Client(42, settings.CLIENT_ID, True)
    client.registerLocalBuffer("control", sizeof(ControlInput), False)
    client.registerLocalBuffer("sensorreset", sizeof(SensorReset), False)
    client.registerLocalBuffer("status", sizeof(Status), False)
    time.sleep(0.5)
    control = set_control(client, 0, 0, 0)
    client.setLocalBufferContents("sensorreset", pack(sensorReset))
    client.setLocalBufferContents("status", pack(status))
    print("Created local buffers: control, sensorreset, status")

    register_remote_buffers(client, settings.remote_buffers)
    print("Registered remote buffers")

    state = KilledState()
    while True:
        try:
            # get new data
            data = update_remote_buffers(client, settings.remote_buffers)
            # update conditions
            list(map(lambda x: x.prepare(), conditions.values()))
            list(map(lambda x: x((control, data)), conditions.values()))
            # advance one state
            next_state = state((control, data, conditions))
            # set outputs
            control = set_control(client, state.heading(), state.depth(), state.velocity())
            print(state)
            state = next_state
            # sleep a bit
            time.sleep(0.01)
        except KeyboardInterrupt:
            print("Caught control-C, exiting")
            break

if __name__ == "__main__":
    main()

# was_killed = True
# heading = 0
# move_back_count = 0
# prev_heading = 0
# while True:
#     try:
#         time.sleep(0.1)
#         update_buffers()
#         at_depth()
#         at_heading()
#         settled()
#         killed()
#         has_detection()

#         if killed.get_value():
#             was_killed = True
#             controlInput.angular[Axes.zaxis].pos[0] = 0
#             controlInput.linear[Axes.xaxis].vel = 0
#             controlInput.linear[Axes.zaxis].pos[0] = 0
#             client.setLocalBufferContents("control", pack(controlInput))
#             print("killed")
#             continue
#         if was_killed:
#             was_killed = False
#             heading = angular.acc[Axes.zaxis]
#             controlInput.angular[Axes.zaxis].pos[0] = heading
#             controlInput.linear[Axes.zaxis].pos[0] = TARGET_DEPTH
#             client.setLocalBufferContents("control", pack(controlInput))
#             print("unkilled, heading: ", heading)
#             continue

#         if move_back_count == 100:
#             move_back_count = 0
#         if move_back_count > 50:
#             controlInput.linear[Axes.xaxis].vel = MOVE_BACK_VEL
#         else:
#             controlInput.linear[Axes.xaxis].vel = 25
#         move_back_count = move_back_count + 1

#         running_avg = sum(run_avg_adjust) / 4
#         if running_avg < RUN_AVG_THRESH and running_avg > -RUN_AVG_THRESH:
#             controlInput.linear[Axes.xaxis].vel = 100

#         if new_frame and has_detection.get_value():
#             heading = align_to_gate(heading, detections)
#             controlInput.angular[Axes.zaxis].pos[0] = heading
#             new_frame = False

#         print("target heading: ", round(heading, 2), " settled: ", settled.get_value(), " detection: ", has_detection.get_value(), " x: ", round(detections.detections[0].x, 2), " size: ", round(detections.detections[0].size, 2))
#         print("avg ", running_avg)
#         print("adj ", run_avg_adjust)
#         client.setLocalBufferContents("control", pack(controlInput))

#     except KeyboardInterrupt:
#         print("Caught control-C, exiting")
#         break

