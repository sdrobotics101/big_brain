from datetime import datetime, timedelta

from constants import Axes
from detection_utils import (
    active_detections,
    align_adjustment,
    center_pole_index,
    gate_dets,
    buoy_dets,
    jianshi_dets,
    triangle_dets
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
        self.set_heading = 0
        self.set_depth = 0
        self.set_velocity = 0
        self.microstate = None
    def __call__(self, args):
        return NotImplementedError
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

class KilledState(State):
    def __init__(self):
        super(KilledState, self).__init__()
        self.microstate = self.check_unkilled
        self.unkilled_heading = 0
    def __call__(self, args):
        return self.microstate(args)
    def check_unkilled(self, args):
        if not conditions(args)["killed"](args):
            self.unkilled_heading = data(args)["angular"].acc[Axes.zaxis]
            self.microstate = self.wait_for_depth
            conditions(args)["below_starting_depth"].reset(False)
        return self
    def wait_for_depth(self, args):
        if conditions(args)["killed"](args):
            return KilledState()
        if conditions(args)["below_starting_depth"](args):
            return SinkAndFindHeading(Gate(self.unkilled_heading),
                                      data(args)["angular"].acc[Axes.zaxis],
                                      self.unkilled_heading,
                                      settings.GATE_TARGET_DEPTH)
            # return SinkAndFindHeading(FindTriangleBuoy(self.unkilled_heading),
            #                           data(args)["angular"].acc[Axes.zaxis],
            #                           self.unkilled_heading,
            #                           settings.BUOY_TARGET_DEPTH)
        return self
    def heading(self):
        return 0
    def depth(self):
        return 0
    def velocity(self):
        return 0

class KillableState(State):
    def __init__(self):
        super(KillableState, self).__init__()
    def __call__(self, args):
        if conditions(args)["killed"](args):
            return KilledState()
        return self.microstate(args)

class Surface(KillableState):
    def __init__(self):
        super(Surface, self).__init__()
        self.set_heading = 0
        self.set_depth = 0
        self.set_velocity = 0
        self.microstate = self.dead
    def dead(self, args):
        return self

class Gate(KillableState):
    def __init__(self, gate_heading):
        super(Gate, self).__init__()
        self.set_heading = gate_heading
        self.set_depth = settings.GATE_TARGET_DEPTH
        self.set_velocity = settings.GATE_VELOCITY
        self.processed_frame = False
        self.left_det = None
        self.right_det = None
        self.dets = []
        self.microstate = self.initial_dead_reckon
    def initial_dead_reckon(self, args):
        if conditions(args)["interesting_frame"](args):
            d = data(args)["forwarddetection"].detections
            self.dets = gate_dets(d)
            num_dets = len(self.dets)
            if num_dets == 1:
                self.microstate = self.initial_one_det
            else:
                self.microstate = self.transition_on_new_frame
        return self
    def initial_one_det(self, args):
        if not self.processed_frame:
            self.processed_frame = True
            if self.dets[0].x > 0.5 + settings.VISION_X_TOLERANCE:
                self.set_heading += settings.GATE_HEADING_ADJUST
            if self.dets[0].x < 0.5 - settings.VISION_X_TOLERANCE:
                self.set_heading -= settings.GATE_HEADING_ADJUST
        if conditions(args)["has_new_frame"](args):
            self.processed_frame = False
            d = data(args)["forwarddetection"].detections
            self.dets = gate_dets(d)
            num_dets = len(self.dets)
            if num_dets == 0:
                self.microstate = self.initial_dead_reckon
            elif num_dets > 1:
                self.microstate = self.transition_on_new_frame
        return self
    def transition_on_new_frame(self, args):
        d = data(args)["forwarddetection"].detections
        self.dets = gate_dets(d)
        if conditions(args)["has_one_gate"](args):
            self.microstate = self.one_det
        elif conditions(args)["has_two_gates"](args):
            self.microstate = self.two_dets
        elif conditions(args)["has_three_gates"](args):
            self.microstate = self.three_dets
        else:
            self.microstate = self.dead_reckon
        return self
    def dead_reckon(self, args):
        self.microstate = self.wait_for_new_frame
        return self
    def two_dets(self, args):
        cpi = center_pole_index(self.dets)
        if cpi == -1:
            self.left_det = self.dets[0]
            self.right_det = self.dets[1]
            self.microstate = self.align_between
        elif cpi == 0:
            if settings.GATE_40_LEFT:
                self.microstate = self.turn_left
            else:
                self.left_det = self.dets[0]
                self.right_det = self.dets[1]
                self.microstate = self.align_between
        elif cpi == 1:
            if settings.GATE_40_LEFT:
                self.left_det = self.dets[0]
                self.right_det = self.dets[1]
                self.microstate = self.align_between
            else:
                self.microstate = self.turn_right
        return self
    def three_dets(self, args):
        if settings.GATE_40_LEFT:
            self.left_det = self.dets[0]
            self.right_det = self.dets[1]
            self.microstate = self.align_between
        else:
            self.left_det = self.dets[1]
            self.right_det = self.dets[2]
            self.microstate = self.align_between
        return self
    def one_det(self, args):
        if self.dets[0].x >= 0.5:
            self.microstate = self.turn_left
        if self.dets[0].x < 0.5:
            self.microstate = self.turn_right
        return self
    def wait_for_new_frame(self, args):
        if conditions(args)["is_through_gate"](args):
            self.microstate = self.do_720
        elif conditions(args)["has_new_frame"](args):
            self.microstate = self.transition_on_new_frame
        return self
    def align_between(self, args):
        self.set_heading += settings.GATE_HEADING_ADJUST * \
                            align_adjustment(self.left_det, self.right_det)
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
    def do_720(self, args):
        self.microstate = self.prepare_for_buoys
        heading_0 = self.set_heading + 0
        heading_90 = self.set_heading + 90
        heading_180 = self.set_heading + 180
        heading_270 = self.set_heading + 270
        turn7 = SinkAndFindHeading(self, heading_270, heading_0, settings.GATE_TARGET_DEPTH)
        turn6 = SinkAndFindHeading(turn7, heading_180, heading_270, settings.GATE_TARGET_DEPTH)
        turn5 = SinkAndFindHeading(turn6, heading_90, heading_180, settings.GATE_TARGET_DEPTH)
        turn4 = SinkAndFindHeading(turn5, heading_0, heading_90, settings.GATE_TARGET_DEPTH)
        turn3 = SinkAndFindHeading(turn4, heading_270, heading_0, settings.GATE_TARGET_DEPTH)
        turn2 = SinkAndFindHeading(turn3, heading_180, heading_270, settings.GATE_TARGET_DEPTH)
        turn1 = SinkAndFindHeading(turn2, heading_90, heading_180, settings.GATE_TARGET_DEPTH)
        turn0 = SinkAndFindHeading(turn1, heading_0, heading_90, settings.GATE_TARGET_DEPTH)
        return turn0
    # def __init__(self, return_state, start_heading, target_heading, target_depth):
    def prepare_for_buoys(self, args):
        initial_turn_heading = self.set_heading + settings.G2B_INITIAL_TURN_AMOUNT
        final_turn_heading = initial_turn_heading + settings.G2B_FINAL_TURN_AMOUNT
        # build the steps in reverse
        enter_buoys = FindTriangleBuoy(final_turn_heading)
        turn_around = SinkAndFindHeading(enter_buoys,
                                         initial_turn_heading,
                                         final_turn_heading,
                                         settings.BUOY_TARGET_DEPTH)
        drive_forward = DriveForward(turn_around,
                                     initial_turn_heading,
                                     settings.G2B_TARGET_DEPTH,
                                     settings.G2B_FORWARD_VELOCITY,
                                     settings.G2B_FORWARD_TIME)
        initial_turn = SinkAndFindHeading(drive_forward,
                                          self.set_heading,
                                          initial_turn_heading,
                                          settings.G2B_TARGET_DEPTH)
        return initial_turn

class FindTriangleBuoy(KillableState):
    def __init__(self, start_heading):
        super(FindTriangleBuoy, self).__init__()
        self.set_heading = start_heading
        self.set_depth = settings.BUOY_TARGET_DEPTH
        self.align_det = None
        self.microstate = self.drive_forward
    def transition_on_new_frame(self, args):
        dets = data(args)["forwarddetection"].detections
        dets = buoy_dets(dets)
        if triangle_dets(dets):
            d = triangle_dets(dets)[0]
            if d.cxt > settings.BUOY_SIZE_THRESH:
                find_jianshi = FindJianshiBuoy(self.set_heading)
                return TouchBuoy(find_jianshi, self.set_heading)
            self.align_det = d
            self.microstate = self.align_to
        elif jianshi_dets(dets):
            if settings.BUOY_TRIANGLE_LEFT:
                self.microstate = self.turn_left
            else:
                self.microstate = self.turn_right
        else:
            self.microstate = self.drive_forward
        return self
    def wait_for_new_frame(self, args):
        if conditions(args)["has_new_frame"](args):
            self.microstate = self.transition_on_new_frame
        return self
    def drive_forward(self, args):
        self.set_velocity = settings.BUOY_VELOCITY
        self.microstate = self.wait_for_new_frame
        return self
    def align_to(self, args):
        self.set_velocity = settings.BUOY_VELOCITY
        if self.align_det.x < (0.5 - settings.VISION_X_TOLERANCE):
            self.microstate = self.turn_left
        elif self.align_det.x > (0.5 + settings.VISION_X_TOLERANCE):
            self.microstate = self.turn_right
        else:
            self.microstate = self.wait_for_new_frame
        return self
    def turn_right(self, args):
        self.set_heading += settings.BUOY_HEADING_ADJUST
        self.microstate = self.wait_for_new_frame
        return self
    def turn_left(self, args):
        self.set_heading -= settings.BUOY_HEADING_ADJUST
        self.microstate = self.wait_for_new_frame
        return self

class FindJianshiBuoy(KillableState):
    def __init__(self, start_heading):
        super(FindJianshiBuoy, self).__init__()
        self.set_heading = start_heading
        self.set_depth = settings.BUOY_TARGET_DEPTH
        self.align_det = None
        self.microstate = self.drive_forward
    def transition_on_new_frame(self, args):
        dets = data(args)["forwarddetection"].detections
        dets = buoy_dets(dets)
        if jianshi_dets(dets) > 0:
            d = jianshi_dets(dets)[0]
            if d.cxt > settings.BUOY_SIZE_THRESH:
                # TODO
                return TouchBuoy(Surface(), self.set_heading)
            self.align_det = d
            self.microstate = self.align_to
        elif triangle_dets(dets) > 0:
            if settings.BUOY_TRIANGLE_LEFT:
                self.microstate = self.turn_right
            else:
                self.microstate = self.turn_left
        else:
            self.microstate = self.drive_forward
        return self
    def wait_for_new_frame(self, args):
        if conditions(args)["has_new_frame"](args):
            self.microstate = self.transition_on_new_frame
        return self
    def drive_forward(self, args):
        self.set_velocity = settings.BUOY_VELOCITY
        self.microstate = self.wait_for_new_frame
        return self
    def align_to(self, args):
        self.set_velocity = settings.BUOY_VELOCITY
        if self.align_det.x < (0.5 - settings.VISION_X_TOLERANCE):
            self.microstate = self.turn_left
        elif self.align_det.x > (0.5 + settings.VISION_X_TOLERANCE):
            self.microstate = self.turn_right
        else:
            self.microstate = self.wait_for_new_frame
        return self
    def turn_right(self, args):
        self.set_heading += settings.BUOY_HEADING_ADJUST
        self.microstate = self.wait_for_new_frame
        return self
    def turn_left(self, args):
        self.set_heading -= settings.BUOY_HEADING_ADJUST
        self.microstate = self.wait_for_new_frame
        return self

class TouchBuoy(KillableState):
    def __init__(self, return_state, start_heading):
        super(TouchBuoy, self).__init__()
        self.return_state = return_state
        self.set_heading = start_heading
        self.set_depth = settings.BUOY_TARGET_DEPTH
        self.set_velocity = settings.BUOY_TOUCH_VELOCITY
        self.start_time = datetime.now()
        self.microstate = self.drive_into_buoy
    def drive_into_buoy(self, args):
        self.set_velocity = settings.BUOY_TOUCH_VELOCITY
        end_time = self.start_time + timedelta(milliseconds=settings.BUOY_TOUCH_TIME)
        cur_time = datetime.now()
        if cur_time > end_time:
            self.start_time = cur_time
            self.microstate = self.stop
        return self
    def stop(self, args):
        self.set_velocity = 0
        end_time = self.start_time + timedelta(milliseconds=settings.BUOY_STOP_TIME)
        cur_time = datetime.now()
        if cur_time > end_time:
            self.start_time = cur_time
            self.microstate = self.back_off
        return self
    def back_off(self, args):
        self.set_velocity = settings.BUOY_BACKOFF_VELOCITY
        end_time = self.start_time + timedelta(milliseconds=settings.BUOY_BACKOFF_TIME)
        cur_time = datetime.now()
        if cur_time > end_time:
            return self.return_state
        return self

class SinkAndFindHeading(KillableState):
    def __init__(self, return_state, start_heading, target_heading, target_depth):
        super(SinkAndFindHeading, self).__init__()
        self.set_heading = start_heading
        self.set_depth = target_depth
        self.target_heading = target_heading
        self.return_state = return_state
        self.microstate = self.sink
    def sink(self, args):
        if conditions(args)["at_depth"](args):
            self.microstate = self.align
            conditions(args)["at_heading"].reset(False)
        return self
    def align(self, args):
        self.set_heading = self.target_heading
        if conditions(args)["settled"](args):
            return self.return_state
        return self

class DriveForward(KillableState):
    def __init__(self, return_state, heading, depth, velocity, time):
        super(DriveForward, self).__init__()
        self.set_heading = heading
        self.set_depth = depth
        self.set_velocity = velocity
        self.end_time = datetime.now() + timedelta(milliseconds=time)
        self.microstate = self.drive_forward
        self.return_state = return_state
    def drive_forward(self, args):
        if datetime.now() > self.end_time:
            return self.return_state
        return self
