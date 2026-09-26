"""Geometry constants for the rebar tensile tester insertion workflow.

All values come from urdf/rebar_test_machine_lab.usda (generated at
(6.0, 4.0) with yaw 90 deg) and scripts/rebar_tester_control.py. World
frame is Z-up, metres. Local->world map for the machine: local (lx, ly, lz)
-> world (6.0 - ly, 4.0 + lx, lz); the grip line is world x = 6.00,
y = 3.92 and the jaws open along world X.
"""

from __future__ import annotations

# --- Tester pose in the lab scene -----------------------------------------
MACHINE_BASE_XY = (6.0, 4.0)
MACHINE_YAW_DEG = 90.0
GRIP_LINE_XY = (6.00, 3.92)      # where the bar hangs, between the columns
COLUMNS_WORLD_XY = ((5.49, 4.03), (6.51, 4.03))   # r = 0.07, top 2.235
CABINET_X_RANGE = (5.36, 6.64)   # world x span of the base cabinet
CABINET_Y_RANGE = (3.59, 4.41)
CABINET_TOP_Z = 0.76
LOWER_PLATEN_TOP_Z = 0.87        # pedestal directly under the grip line
CONSOLE_X_RANGE = (6.77, 7.53)   # floor console, world +X side
CONSOLE_Y_RANGE = (3.60, 4.32)
CONSOLE_TOP_Z = 1.05
MACHINE_TOP_Z = 2.40

# --- Carriage / jaw travel (mirrors rebar_tester_control.py) --------------
JAW_OPENING_MIN = 0.024          # equals the rebar diameter when closed on it
JAW_OPENING_MAX = 0.16
UPPER_Z_LIMITS = (1.50, 1.88)
LOWER_Z_LIMITS = (0.92, 1.34)
TIP_PAD_HALFSPAN = (0.0575, 0.0925)  # tip pad z offsets toward the gap
JAW_PLATE_HALFHEIGHT = 0.065

# --- Insertion pose for the 0.6 m lab rebar --------------------------------
# The bar hangs vertically on the grip line; the robot approaches from the
# south (-Y) with the wrist horizontal. With both jaws pre-set open at the
# extremes there is clear horizontal corridor: bar bottom 1.20 sits above
# the lower plate top (0.92 + 0.065 = 0.985) and bar top 1.80 stays below
# the upper plate bottom (1.88 - 0.065 = 1.815).
BAR_LENGTH = 0.6
BAR_DIAMETER = 0.024
BAR_CENTER_Z = 1.50
BAR_BOTTOM_Z = BAR_CENTER_Z - BAR_LENGTH / 2      # 1.20
BAR_TOP_Z = BAR_CENTER_Z + BAR_LENGTH / 2         # 1.80
# The physical pickup grips the bar at its centre. The temporary grasp joint
# preserves that offset, so the TCP and bar centre must have the same height.
# An IK probe confirms this is reachable from the 3.05 m parking position.
BAR_HOLD_Z = BAR_CENTER_Z                         # 1.50
APPROACH_FROM_SOUTH_Y = GRIP_LINE_XY[1] - 0.16    # wrist plane while inserting
# The exact grip-line point is IK-inreachable from the parking spot (probe:
# 20/21 corridor points solve, only y-offset 0 fails); 2 cm south stays
# inside the jaw-tip grip span (y +/-0.05 around the grip line) and is
# reliably reachable.
INSERT_Y_INSET = 0.02

# Jaw z commands that grip the bar ends after insertion: the tip pads
# (z = carriage_z +/- [0.0575, 0.0925]) must straddle the bar ends.
LOWER_JAW_GRIP_Z = 1.12   # tips 1.1775..1.2125 -> grips bottom end 1.20
UPPER_JAW_GRIP_Z = 1.87   # tips 1.7775..1.8125 -> grips top end 1.80
JAW_CLOSE_OPENING = JAW_OPENING_MIN
JAW_INSERT_OPENING = JAW_OPENING_MAX

# --- Mobile base staging ---------------------------------------------------
# Final pose: facing south (yaw -90 deg) so the arm (mounted toward robot
# -X, i.e. world +Y) reaches the grip line. The chassis box is 1.00 x 0.70,
# so the rear edge stays 0.09 m clear of the cabinet front (y = 3.59).
# Parking at y 3.05 makes the 1.50 m insertion centre reachable while
# leaving 4 cm between the rear chassis edge and the cabinet front.
BASE_TARGET_XY = (6.0, 3.05)
BASE_TARGET_YAW = -1.5707963267948966      # -90 deg
# Route avoiding the work-table legs (x 5.99..6.11 at y +/-0.42, which
# block a straight (0,0)->(6,0) run) and the bollards at (5.4, +/-1).
# The final turn to face south happens at y 2.6 (corner-sweep radius 0.61 m
# stays clear of the cabinet front at y 3.59); the last 0.45 m is a short
# reverse so the chassis never sweeps near the cabinet corner.
BASE_DRIVE_WAYPOINTS_XY = (
    (0.0, 0.0), (4.5, 0.0), (4.5, 2.6), (6.0, 2.6), BASE_TARGET_XY,
)
BASE_TOLERANCE_XY = 0.02
BASE_TOLERANCE_YAW = 0.02
BASE_MAX_LINEAR = 0.3
BASE_MAX_ANGULAR = 0.5
BASE_PUBLISH_HZ = 10.0

# Carry pose while driving: bar held high and retracted close to the body
# (bollards are 0.9 m tall, the worktable 0.81 m; the bar rides at ~1.2 m).
CARRY_TCP_BASE = (-0.70, 0.30, 1.20)       # TCP in base_footprint while driving
CARRY_CLEARANCE_Z = 1.20

# Verticalisation staging point: offset outside the arm plane (the links
# live around y = +/-0.19 in base frame; the bar sweeping upright at
# y = 0.35 keeps ~9 cm lateral clearance) and far enough forward that the
# 0.6 m bar sweep stays clear of the shoulder. z 1.10 keeps the wrist-down
# arrival radius at sqrt(0.65^2+0.50^2) ~ 0.82 m (0.90 m is the measured
# edge of the envelope) while the hanging bar bottom (0.79 m) still clears
# the deck rack (0.73 m); after rotating upright the wrist drops to ~0.62 m.
VERTICALIZE_TCP_BASE = (-0.85, 0.35, 1.10)


def machine_boxes():
    """Yield (name, center_xyz, size_xyz) world-frame boxes for MoveIt.

    Covers the static frame: cabinet, both columns, top beam, lower platen
    and the floor console. Moving carriages are intentionally omitted -- a
    box spanning their travel would block the insertion corridor; the
    workflow keeps the jaws at commanded positions instead.
    """
    yield "cabinet", (6.00, 4.00, 0.38), (1.28, 0.82, 0.76)
    for index, (cx, cy) in enumerate(COLUMNS_WORLD_XY, 1):
        yield f"column_{index}", (cx, cy, 1.52), (0.14, 0.14, 1.43)
    yield "top_beam", (6.00, 4.03, 2.25), (1.28, 0.72, 0.18)
    yield "lower_platen", (GRIP_LINE_XY[0], GRIP_LINE_XY[1], 0.83), (0.48, 0.48, 0.08)
    yield "console", (
        (CONSOLE_X_RANGE[0] + CONSOLE_X_RANGE[1]) / 2,
        (CONSOLE_Y_RANGE[0] + CONSOLE_Y_RANGE[1]) / 2,
        CONSOLE_TOP_Z / 2,
    ), (
        CONSOLE_X_RANGE[1] - CONSOLE_X_RANGE[0],
        CONSOLE_Y_RANGE[1] - CONSOLE_Y_RANGE[0],
        CONSOLE_TOP_Z,
    )
