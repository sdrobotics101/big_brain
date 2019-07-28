import settings
from functools import reduce

def active_detections(dets):
    m = map(lambda x: x.cls != 255, dets)
    return reduce(lambda x,y: x+y, m)

def center_pole_index(dets):
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
