"""Smooth monotonic compact joint curve, shared by runtime and local checks."""
import math

# Preserve the branch that already completed the base approach and tester handoff.
TARGET = (2.597, -.357, -1.451, -1.094, -1.026, 0.)
SCHEDULE = (.4113185111453064, 1.3400070529043515, 1.0641470249901905,
            .24599207492435196, 4.440138191401998, 1.251056459727879)
VELOCITIES = (3.10668606855, 3.10668606855, 4.66002910282,
              4.13643032723, 4.13643032723, 4.13643032723)


def nearest_target(start, target=TARGET):
    selected = []
    for i, (source, value) in enumerate(zip(start, target)):
        bound = 2.8099800957108707 if i == 2 else 2 * math.pi - .0001
        variants = []
        for turn in range(-2, 3):
            angle = value + turn * 2 * math.pi
            if abs(angle) <= bound + .00011:
                variants.append(max(-bound, min(bound, angle)))
        selected.append(min(variants, key=lambda angle: abs(angle - source)))
    return tuple(selected)


def sample(start, target, schedule, progress):
    t = progress
    u = 10 * t**3 - 15 * t**4 + 6 * t**5
    du = 30 * t**2 * (1 - t)**2
    ddu = 60 * t * (1 - t) * (1 - 2 * t)
    positions, velocities, accelerations = [], [], []
    for source, finish, p in zip(start, target, schedule):
        denominator = p + (1 - p) * u
        g = u / denominator
        dg = p / denominator**2
        ddg = -2 * p * (1 - p) / denominator**3
        delta = finish - source
        positions.append(source + delta * g)
        velocities.append(delta * dg * du)
        accelerations.append(delta * (ddg * du**2 + dg * ddu))
    return positions, velocities, accelerations


def retime(start, target, schedule, count=1001, speed_scale=1.):
    if not math.isfinite(speed_scale) or not .25 <= speed_scale <= 2.:
        raise ValueError("speed scale must be finite and between 0.25 and 2.0")
    rows = [sample(start, target, schedule, i / (count - 1)) for i in range(count)]
    duration = 40.
    for _, speeds, accelerations in rows:
        for i in range(6):
            duration = max(duration, abs(speeds[i]) / (VELOCITIES[i] * .06),
                           math.sqrt(abs(accelerations[i]) / .12))
    # Slack for controller interpolation and feedback settling.
    # Time compression doubles velocity and quadruples acceleration at 2x.
    return rows, duration * 1.1 / speed_scale
