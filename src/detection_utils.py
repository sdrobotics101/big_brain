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

# run_avg_adjust = [10, 10, 10, 10]
# def align_to_gate(heading, detections):
#     dets = detections.detections
#     num_dets = active_detections(dets)
#     new_heading = heading
#     global run_avg_adjust
#     if num_dets == 1:
#         if dets[0].x > 0.5 + VISION_X_TOLERANCE:
#             new_heading += HEADING_CHANGE
#         if dets[0].x < 0.5 - VISION_X_TOLERANCE:
#             new_heading -= HEADING_CHANGE
#     # if num_dets == 2:
#     #     cpi = center_pole_index(dets)
#     #     if cpi == -1:
#     #         new_heading += align_adjustment(dets[0], dets[1])
#     #     elif cpi == 0:
#     #         if GATE_40_LEFT:
#     #             new_heading -= HEADING_CHANGE
#     #         else:
#     #             adjustments = align_adjustment(dets[0], dets[1])
#     #             run_avg_adjust.append(adjustments)
#     #             run_avg_adjust.pop(0)
#     #             new_heading += adjustments
#     #     elif cpi == 1:
#     #         if GATE_40_LEFT:
#     #             adjustments = align_adjustment(dets[0], dets[1])
#     #             run_avg_adjust.append(adjustments)
#     #             run_avg_adjust.pop(0)
#     #             new_heading += adjustments
#     #         else:
#     #             new_heading += HEADING_CHANGE
#     if num_dets == 3:
#         if GATE_40_LEFT:
#             adjustments = align_adjustment(dets[0], dets[1])
#             run_avg_adjust.append(adjustments)
#             run_avg_adjust.pop(0)
#             new_heading += adjustments
#         else:
#             adjustments = align_adjustment(dets[1], dets[2])
#             run_avg_adjust.append(adjustments)
#             run_avg_adjust.pop(0)
#             new_heading += adjustments
#     return new_heading

