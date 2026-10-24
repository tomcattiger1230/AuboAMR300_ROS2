"""Candidate: yield recorded empty-wrist return before source leaves the image.

Uses only the already surveyed source, fresh camera extrinsics and robot
feedback. This is a motion handoff, never an estimator freshness exemption.
Must pass physical branch/collision and full-mission regression before use.
"""
import numpy as np


def source_return_handoff(*, valid, last_good_ns, last_error, camera, center,
                          tcp, fingers, finger_forces, visible_edges,
                          residual_px, return_index, history_length):
    if (not valid or camera is None or last_error or last_good_ns != camera.timestamp_ns
            or history_length < 4 or not 0 < return_index <= history_length // 4
            or visible_edges < 24 or not np.isfinite(residual_px) or residual_px > 4.):
        return False
    values = [center, tcp, fingers, finger_forces, camera.K, camera.T_scene_optical]
    if not all(np.isfinite(x).all() for x in values):
        return False
    if (np.max(np.abs(fingers)) >= .003 or np.max(finger_forces) >= .5
            or tcp[2] - center[2] < .10):
        return False
    T = camera.T_scene_optical
    point = T[:3, :3].T @ (center - T[:3, 3])
    if point[2] <= 0:
        return False
    pixel = camera.K @ point
    pixel = pixel[:2] / pixel[2]
    height, width = camera.mono8.shape
    # Leave a 96-pixel image-space margin while observations still pass the
    # existing estimator. No arm action is enabled after a rejected frame.
    return bool(not (96 <= pixel[0] <= width - 96 and 96 <= pixel[1] <= height - 96))
