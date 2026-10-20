"""Declared simulation curricula. No parameter is inferred from steel truth."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Perturbation:
    source_xy_m: float
    source_yaw_rad: float
    arm_joints_rad: float
    base_xy_m: float = 0.
    base_yaw_rad: float = 0.
    image_gain: float = 0.
    image_offset: float = 0.
    image_noise_std: float = 0.


PROFILES = {
    'baseline': Perturbation(.03, .05, .02),
    'expanded': Perturbation(.06, .10, .035, .02, .02, .10, 4., 1.),
    'challenge': Perturbation(.09, .16, .05, .04, .035, .20, 8., 2.),
}


def profile(name):
    try:
        return PROFILES[name]
    except KeyError as error:
        raise ValueError(f'Unknown randomization profile: {name}') from error


def describe(name):
    return {'name': name, 'bounds': asdict(profile(name)),
            'source_geometry': 'station and supported steel move together at reset only',
            'image_noise': 'per-frame Gaussian; gain/offset fixed for each episode'}


def perturb_image(mono, gain, offset, noise_std, rng):
    import numpy as np
    if gain == 1. and offset == 0. and noise_std == 0.:
        return mono
    result = mono.astype(np.float32)*gain+offset
    if noise_std:
        result += rng.normal(0., noise_std, result.shape)
    return np.rint(result).clip(0, 255).astype(np.uint8)
