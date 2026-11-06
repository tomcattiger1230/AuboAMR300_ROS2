"""Synthetic projective-camera checks only; never acquires a live camera or robot."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .fiducial_localization import (CameraCalibration, Limits, MarkerAnchor, WristMarkerEstimator,
                                    rotation_distance, square_points)
from .workcells import FROZEN_POLICY_SHA256


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def transform(xyz, angles):
    t = np.eye(4); t[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix(); t[:3, 3] = xyz
    return t


def projected(t, k):
    return cv2.projectPoints(square_points(.16), cv2.Rodrigues(t[:3, :3])[0], t[:3, 3], k, None)[0].reshape(4, 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New artifact directory')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    checkpoint = root/'rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/frozen_policy.pt'
    before = digest(checkpoint)
    if before != FROZEN_POLICY_SHA256:
        raise ValueError('Frozen checkpoint identity mismatch')
    args.output.mkdir(parents=True, exist_ok=False)
    k = np.array([[1000., 0, 640], [0, 1000, 480], [0, 0, 1]])
    flange_camera = transform((.05, .02, .06), (.1, -.3, .2))
    c = CameraCalibration('synthetic_wrist', 'synthetic_intrinsics', 'synthetic_handeye',
                          (1280, 960), k, (0.,)*5, flange_camera,
                          'control_base', 'flange', 'wrist_optical', 'synthetic_common_clock')
    a = MarkerAnchor('machine_1', 'DICT_5X5_100', 11, .16, np.eye(4), 'synthetic_survey')
    estimator = WristMarkerEstimator(c, a)
    rng = np.random.default_rng(20261106); records = []
    for distance in (.5, .8, 1.2):
        for tilt_deg in (10, 25, 45):
            truth = transform((.03, .02, distance), (math.pi-math.radians(tilt_deg), .08, .06))
            points = projected(truth, k)
            for repeat in range(20):
                perturbed = points+rng.normal(0, .2, (4, 2))
                row = dict(distance_m=distance, tilt_deg=tilt_deg, repeat=repeat,
                           corners_px=perturbed.tolist(), origin='synthetic_projection')
                try:
                    recovered, choices = estimator.solve_corners(perturbed)
                    row.update(accepted=True, rms_px=choices[0]['rms_px'],
                               translation_error_m=float(np.linalg.norm(recovered[:3, 3]-truth[:3, 3])),
                               rotation_error_rad=rotation_distance(recovered, truth))
                except ValueError as exc:
                    row.update(accepted=False, reason=str(exc))
                records.append(row)
    raster = []
    for marker_id in (11, 12, 13):
        truth = transform((.03, .02, .8), (math.pi-.25, .15, .06))
        pixels = projected(truth, k)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 512)
        # These PNGs are synthetic detector fixtures, not dimensioned print masters.
        h = cv2.getPerspectiveTransform(np.float32([[0, 0], [511, 0], [511, 511], [0, 511]]), pixels.astype(np.float32))
        image = cv2.warpPerspective(marker, h, (1280, 960), flags=cv2.INTER_NEAREST, borderValue=255)
        image_path = args.output/f'synthetic_projected_marker_{marker_id}.png'
        cv2.imwrite(str(image_path), image)
        e = WristMarkerEstimator(c, MarkerAnchor(f'machine_{marker_id-10}', 'DICT_5X5_100', marker_id,
                                               .16, np.eye(4), 'synthetic_survey'))
        result = e.estimate(image, machine_id=f'machine_{marker_id-10}', frame_id=f'frame_{marker_id}',
                            image_stamp_s=1., fk_stamp_s=1., now_s=1.01, base_from_flange=np.eye(4),
                            camera_id=c.camera_id, calibration_id=c.calibration_id, hand_eye_id=c.hand_eye_id,
                            fk_base_frame_id=c.base_frame_id, fk_flange_frame_id=c.flange_frame_id,
                            optical_frame_id=c.optical_frame_id, clock_id=c.clock_id)
        expected = flange_camera @ truth
        actual = np.array(result['base_from_fixture'])
        raster.append(dict(machine_id=result['machine_id'], marker_id=marker_id,
                           image_file=image_path.name, file_sha256=digest(image_path),
                           translation_error_m=float(np.linalg.norm(actual[:3, 3]-expected[:3, 3])),
                           rotation_error_rad=rotation_distance(actual, expected), observation=result))
    after = digest(checkpoint)
    if before != after:
        raise ValueError('Checkpoint changed during offline check')
    accepted = [r for r in records if r['accepted']]
    report = dict(schema='fiducial_projective_check_v1', origin='synthetic_camera_geometry_only',
                  physical_acceptance=False, calibration_exact_in_synthetic_check=True,
                  caveat='Corner noise only; no real optics, survey, hand-eye, timing, flex or actuator error. Not engineering accuracy.',
                  corner_noise_std_px=.2, seed=20261106, limits=asdict(Limits()),
                  corner_trials=records, accepted_corner_trials=len(accepted),
                  rejected_corner_trials=len(records)-len(accepted), raster_detector_trials=raster,
                  policy_sha256_before=before, policy_sha256_after=after, policy_optimizer_loaded=False,
                  training=False, simulation=False, actuator_dispatch=False,
                  checkpoint_unchanged=before==after, opencv=cv2.__version__)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('accepted_corner_trials', 'rejected_corner_trials', 'checkpoint_unchanged')}))


if __name__ == '__main__':
    main()
