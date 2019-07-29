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
from detection_utils import (
    active_detections,
    gate_dets
)

from functools import partial

from condition import Condition

from state import KilledState, Gate

sensorReset = SensorReset()
sensorReset.pos[Axes.xaxis] = 0
sensorReset.pos[Axes.yaxis] = 0
sensorReset.pos[Axes.zaxis] = 0
sensorReset.reset      = False

status = Status()
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
    actual = data(args)["angular"].acc[Axes.zaxis]
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
    return len(active_detections(dets)) > 0
has_active_detection = Condition(active_detection, 1, 1, False)
conditions["has_active_detection"] = has_active_detection

def n_gates(args, n):
    dets = data(args)["forwarddetection"].detections
    return len(gate_dets(dets)) == n
one_gate = partial(n_gates, n=1)
two_gates = partial(n_gates, n=2)
three_gates = partial(n_gates, n=3)
has_one_gate = Condition(one_gate, 150, 1, False)
has_two_gates = Condition(two_gates, 1, 1, False)
has_three_gates = Condition(three_gates, 1, 1, False)
conditions["has_one_gate"] = has_one_gate
conditions["has_two_gates"] = has_two_gates
conditions["has_three_gates"] = has_three_gates

prev_x = 0
def new_frame(args):
    dets = data(args)["forwarddetection"].detections
    global prev_x
    if dets[0].x != prev_x:
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
    # client.registerLocalBuffer("status", sizeof(Status), False)
    time.sleep(0.5)
    control = set_control(client, 0, 0, 0)
    client.setLocalBufferContents("sensorreset", pack(sensorReset))
    # client.setLocalBufferContents("status", pack(status))
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
            strState = str(state)
            if not ("new_frame" in strState):
                print(state)
            state = next_state
            # sleep a bit
            time.sleep(0.01)
        except KeyboardInterrupt:
            print("Caught control-C, exiting")
            break

if __name__ == "__main__":
    main()
