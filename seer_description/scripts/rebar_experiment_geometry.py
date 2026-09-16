"""Shared metre-scale collision geometry for the rebar loading experiment."""
import math

SLOT_X = (0.2927, 0.1969, 0.1138, 0.0118)
RACK_Y = 0.0
RACK_REBAR_Z = 0.670


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
            yield f"Slot{slot}Support{support}", (x, y, 0.627), (
                0.060, 0.080, 0.050), (0.0, 0.0, 0.0)
            for segment, (position, size, rpy) in enumerate(
                saddle_boxes("Y", (x, y, 0.675)), 1
            ):
                yield f"Slot{slot}Saddle{support}_{segment}", position, size, rpy
