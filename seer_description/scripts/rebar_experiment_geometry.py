"""Shared metre-scale collision geometry for the rebar loading experiment."""
import math

SLOT_X = (0.2927, 0.1969, 0.1138, 0.0118)
RACK_Y = 0.0
RACK_REBAR_Z = 0.715
# Preserve the calibrated release target; leave only 5 mm of free fall.
RACK_RELEASE_TCP_Z = 0.720
RACK_PEDESTAL_BOTTOM_Z = 0.602


def saddle_boxes(axis, center, width=0.08, radius=0.017, thickness=0.006,
                 segments=14):
    """Approximate an open concave arc with overlapping oriented box segments.

    The inner arc spans +/-70 degrees from its lowest point. Unlike a single
    convex hull, the colliders preserve the hollow seat and do not fill it.
    """
    step = math.radians(140) / segments
    radial = radius + thickness / 2
    tangent = 2 * radial * math.tan(step / 2) * 1.03
    for index in range(segments):
        angle = -math.radians(70) + (index + 0.5) * step
        offset = radial * math.sin(angle)
        height = center[2] - radial * math.cos(angle)
        if axis == "X":
            yield (center[0], center[1] + offset, height), (
                width, tangent, thickness), (angle, 0.0, 0.0)
        elif axis == "Y":
            yield (center[0] + offset, center[1], height), (
                tangent, width, thickness), (0.0, -angle, 0.0)
        else:
            raise ValueError("saddle axis must be X or Y")


def rack_boxes():
    for slot, x in enumerate(SLOT_X, 1):
        for support, y in enumerate((-0.19, 0.19), 1):
            pedestal_top = RACK_REBAR_Z - .018
            yield f"Slot{slot}Support{support}", (
                x, y, (RACK_PEDESTAL_BOTTOM_Z + pedestal_top) / 2), (
                0.060, 0.080, pedestal_top - RACK_PEDESTAL_BOTTOM_Z), (0.0, 0.0, 0.0)
            for segment, (position, size, rpy) in enumerate(
                saddle_boxes("Y", (x, y, RACK_REBAR_Z + .005)), 1
            ):
                yield f"Slot{slot}Saddle{support}_{segment}", position, size, rpy
            for face, position, size, rpy in v_seat_boxes(x, y):
                yield f"Slot{slot}VSeat{support}_{face}", position, size, rpy


def v_seat_boxes(x, y, angle_degrees=30, width=.08, thickness=.006):
    """Two tangent planes at +/-30 degrees hold the 24 mm smooth cylinder.

    Keep the bar centre at the shared calibrated rack height. The outer curved walls
    remain; the middle arc boxes are visual only in the Isaac rack layer.
    """
    radial = .012 + thickness/2
    for index, sign in enumerate((-1, 1), 1):
        angle = math.radians(angle_degrees) * sign
        yield index, (x + radial*math.sin(angle), y,
                      RACK_REBAR_Z-radial*math.cos(angle)), (
                          .028, width, thickness), (0., -angle, 0.)
