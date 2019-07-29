from constants import Axes
from detection_utils import (
    active_detections,
    align_adjustment,
    center_pole_index
)

import settings

def control(args):
    return args[0]

def data(args):
    return args[1]

def conditions(args):
    return args[2]

class State:
    def __init__(self):
        pass
    def __call__(self, args):
        return NotImplementedError
    def heading(self):
        return NotImplementedError
    def depth(self):
        return NotImplementedError
    def velocity(self):
        return NotImplementedError
    def status(self):
        return NotImplementedError
    def __str__(self):
        return NotImplementedError

class KilledState(State):
    def __init__(self):
        self.microstate = self.check_unkilled
        self.heading_to_follow = 0
    def __call__(self, args):
        return self.microstate(args)
    def check_unkilled(self, args):
        if not conditions(args)["killed"](args):
            self.heading_to_follow = data(args)["angular"].acc[Axes.zaxis]
            self.microstate = self.wait_for_depth
            conditions(args)["below_starting_depth"].reset(False)
        return self
    def wait_for_depth(self, args):
        if conditions(args)["killed"](args):
            return KilledState()
        if conditions(args)["below_starting_depth"](args):
            # return Gate(self.heading_to_follow)
            return Buoy(self.heading_to_follow)
        return self
    def heading(self):
        return 0
    def depth(self):
        return 0
    def velocity(self):
        return 0
    def __str__(self):
        return " - ".join([self.__class__.__name__, self.microstate.__name__])

class Gate(State):
    def __init__(self, gate_heading):
        self.microstate = self.sink
        self.gate_heading = gate_heading
        self.set_heading = 0
        self.set_depth = 0
        self.set_velocity = 0
        self.processed_frame = False
    def __call__(self, args):
        if conditions(args)["killed"](args):
            return KilledState()
        return self.microstate(args)
    def sink(self, args):
        self.set_depth = settings.GATE_TARGET_DEPTH
        self.set_heading = data(args)["angular"].acc[Axes.zaxis]
        if conditions(args)["at_depth"](args):
            self.microstate = self.initial_align
        return self
    def initial_align(self, args):
        self.set_heading = self.gate_heading
        if conditions(args)["settled"](args):
            self.microstate = self.initial_dead_reckon
        return self
    def initial_dead_reckon(self, args):
        self.set_velocity = settings.GATE_VELOCITY
        if conditions(args)["interesting_frame"](args):
            dets = data(args)["forwarddetection"].detections
            num_dets = active_detections(dets)
            if num_dets == 1:
                self.microstate = self.initial_one_det
            else:
                self.microstate = self.transition_on_num_dets
        return self
    def dead_reckon(self, args):
        self.set_velocity = settings.GATE_VELOCITY
        self.microstate = self.wait_for_new_frame
        return self
    def transition_on_num_dets(self, args):
        dets = data(args)["forwarddetection"].detections
        num_dets = active_detections(dets)
        if conditions(args)["has_one_detection"](args):
            self.microstate = self.one_det
        elif conditions(args)["has_two_detections"](args):
            self.microstate = self.two_dets
        elif conditions(args)["has_three_detections"](args):
            self.microstate = self.three_dets
        else:
            self.microstate = self.dead_reckon
        return self
    def initial_one_det(self, args):
        if not self.processed_frame:
            self.processed_frame = True
            dets = data(args)["forwarddetection"].detections
            if dets[0].x > 0.5 + settings.VISION_X_TOLERANCE:
                self.set_heading += settings.GATE_HEADING_ADJUST
            if dets[0].x < 0.5 - settings.VISION_X_TOLERANCE:
                self.set_heading -= settings.GATE_HEADING_ADJUST
        if conditions(args)["has_new_frame"](args):
            self.processed_frame = False
            dets = data(args)["forwarddetection"].detections
            num_dets = active_detections(dets)
            if num_dets == 0:
                self.microstate = self.initial_dead_reckon
            elif num_dets > 1:
                self.microstate = self.transition_on_num_dets
        return self
    def two_dets(self, args):
        self.set_velocity = settings.GATE_VELOCITY
        dets = data(args)["forwarddetection"].detections
        cpi = center_pole_index(dets)
        if cpi == -1:
            self.microstate = self.align01
        elif cpi == 0:
            if settings.GATE_40_LEFT:
                self.microstate = self.turn_left
            else:
                self.microstate = self.align01
        elif cpi == 1:
            if settings.GATE_40_LEFT:
                self.microstate = self.align01
            else:
                self.microstate = self.turn_right
        return self
    def three_dets(self, args):
        self.set_velocity = settings.GATE_VELOCITY
        dets = data(args)["forwarddetection"].detections
        if settings.GATE_40_LEFT:
            self.microstate = self.align01;
        else:
            self.microstate = self.align12;
        return self
    def one_det(self, args):
        dets = data(args)["forwarddetection"].detections
        if dets[0].x >= 0.5:
            self.microstate = self.turn_left
        if dets[0].x < 0.5:
            self.microstate = self.turn_right
        return self
    def wait_for_new_frame(self, args):
        if conditions(args)["has_new_frame"](args):
            self.microstate = self.transition_on_num_dets
        return self
    def align01(self, args):
        dets = data(args)["forwarddetection"].detections
        self.set_heading += settings.GATE_HEADING_ADJUST * align_adjustment(dets[0], dets[1])
        self.microstate = self.wait_for_new_frame
        return self
    def align12(self, args):
        dets = data(args)["forwarddetection"].detections
        self.set_heading += settings.GATE_HEADING_ADJUST * align_adjustment(dets[1], dets[2])
        self.microstate = self.wait_for_new_frame
        return self
    def turn_right(self, args):
        self.set_heading += settings.GATE_HEADING_ADJUST
        self.microstate = self.wait_for_new_frame
        return self
    def turn_left(self, args):
        self.set_heading -= settings.GATE_HEADING_ADJUST
        self.microstate = self.wait_for_new_frame
        return self
    def heading(self):
        return self.set_heading
    def depth(self):
        return self.set_depth
    def velocity(self):
        return self.set_velocity
    def __str__(self):
        return " - ".join([self.__class__.__name__,
                           self.microstate.__name__,
                           str(self.set_heading),
                           str(self.set_depth),
                           str(self.set_velocity)])

class Buoy(State):
    def __init__(self, start_heading):
        self.microstate = self.sink
        self.start_heading = start_heading
        self.set_heading = 0
        self.set_depth = 0
        self.set_velocity = 0
        self.touch_counter = 0
    def __call__(self, args):
        if conditions(args)["killed"](args):
            return KilledState()
        return self.microstate(args)
    def sink(self, args):
        self.set_velocity = 0
        self.set_depth = settings.BUOY_TARGET_DEPTH
        self.set_heading = self.start_heading
        if conditions(args)["settled"](args):
            self.microstate = self.wait_for_new_frame
        return self
    def transition_on_new_frame(self, args):
        dets = data(args)["forwarddetection"].detections
        num_dets = active_detections(dets)
        if num_dets == 0:
            self.microstate = self.drive_forward
        # TODO magic number 3
        elif num_dets >= 1 and dets[0].cls == 3:
            if dets[0].size >= settings.BUOY_SIZE_THRESH:
                self.microstate = self.touch_and_back_off
            else:
                self.microstate = self.align0
        return self
    def wait_for_new_frame(self, args):
        if conditions(args)["has_new_frame"](args):
            self.microstate = self.transition_on_new_frame
        return self
    def drive_forward(self, args):
        self.set_velocity = settings.BUOY_VELOCITY
        self.microstate = self.wait_for_new_frame
        return self
    def align0(self, args):
        dets = data(args)["forwarddetection"].detections
        if dets[0].x < (0.5 - settings.VISION_X_TOLERANCE):
            self.microstate = self.turn_left
        elif dets[0].x > (0.5 + settings.VISION_X_TOLERANCE):
            self.microstate = self.turn_right
        else:
            self.microstate = self.wait_for_new_frame
        return self
    def touch_and_back_off(self, args):
        if (self.touch_counter < settings.BUOY_TOUCH_PRE_CYCLE_COUNT):
            self.set_velocity = settings.BUOY_VELOCITY
        elif (self.touch_counter < settings.BUOY_TOUCH_POST_CYCLE_COUNT):
            self.set_velocity = settings.BUOY_REVERSE_VELOCITY
        else:
            self.microstate = self.dead
        self.touch_counter += 1
        return self
    def dead(self, args):
        self.set_velocity = 0
        return self
    def turn_right(self, args):
        self.set_heading += settings.BUOY_HEADING_ADJUST
        self.microstate = self.wait_for_new_frame
        return self
    def turn_left(self, args):
        self.set_heading -= settings.BUOY_HEADING_ADJUST
        self.microstate = self.wait_for_new_frame
        return self
    def heading(self):
        return self.set_heading
    def depth(self):
        return self.set_depth
    def velocity(self):
        return self.set_velocity
    def __str__(self):
        return " - ".join([self.__class__.__name__,
                           self.microstate.__name__,
                           str(self.set_heading),
                           str(self.set_depth),
                           str(self.set_velocity)])

