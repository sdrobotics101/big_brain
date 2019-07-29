import settings
from functools import reduce, partial

def active_detections(dets):
    f = lambda x: x.cls != 255
    return list(filter(f, dets))

def detections_of_class(dets, cls):
    f = lambda x: x.cls == cls
    return list(filter(f, dets))

gate_dets = partial(detections_of_class, cls=settings.FOR_GATE)

def center_pole_index(dets):
    dets = gate_dets(dets)
    if dets[0].h < dets[1].h * settings.CENTER_POLE_HEIGHT_MULTIPLIER:
        return 0
    elif dets[1].h < dets[0].h * settings.CENTER_POLE_HEIGHT_MULTIPLIER:
        return 1
    else:
        return -1

def align_adjustment(left_det, right_det):
    left_dist = abs(0.5 - left_det.x)
    right_dist = abs(0.5 - right_det.x)
    d = right_dist - left_dist
    if d > settings.VISION_X_TOLERANCE:
        return 1
    elif d < -settings.VISION_X_TOLERANCE:
        return -1
    else:
        return 0
