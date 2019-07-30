import settings
from functools import reduce, partial

def active_detections(dets):
    f = lambda x: x.cls != 255
    return list(filter(f, dets))

def detections_of_class(dets, cls):
    f = lambda x: x.cls == cls
    return list(filter(f, dets))

aswang_dets = partial(detections_of_class, cls=settings.ASWANG)
draugr_dets = partial(detections_of_class, cls=settings.DRAUGR)
vetalas_dets = partial(detections_of_class, cls=settings.VETALAS)
jianshi_dets = partial(detections_of_class, cls=settings.JIANGSHI)
gate_dets = partial(detections_of_class, cls=settings.GATE)
bin_dets = partial(detections_of_class, cls=settings.BIN)
fmarker_dets = partial(detections_of_class, cls=settings.FOR_MARKER)

dmarker_dets = partial(detections_of_class, cls=settings.DOW_MARKER)
checker_dets = partial(detections_of_class, cls=settings.CHECKER)
wolf_dets = partial(detections_of_class, cls=settings.WOLF)
bat_dets = partial(detections_of_class, cls=settings.BAT)

def is_triangle_cls(cls):
    return cls == settings.ASWANG or \
           cls == settings.DRAUGR or \
           cls == settings.VETALAS

def triangle_dets(dets):
    f = lambda x: is_triangle_cls(x.cls)
    return list(filter(f, dets))

def is_buoy_cls(cls):
    return cls == settings.JIANGSHI or \
           is_triangle_cls(cls)

def buoy_dets(dets):
    f = lambda x: is_buoy_cls(x.cls)
    return list(filter(f, dets))

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
