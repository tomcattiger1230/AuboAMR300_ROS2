"""Calibrated mono geometry; no simulator imports, depth, or ground-truth reads.

Inputs must be undistorted pixel keypoints and an independently known support
plane through the bar centreline. This assumption is valid for table pickup,
not for a carried bar or arbitrary insertion pose.
"""
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class BarEstimate:
    frame_id: str
    timestamp_ns: int
    position_m: list[float]
    axis: list[float]
    length_m: float
    confidence: float
    source: str = "mono_keypoints_known_plane_v1"
    axis_sign_ambiguous: bool = True

    def to_dict(self):
        return asdict(self)


def calibration(K, T_frame_optical):
    K, T = np.asarray(K, float), np.asarray(T_frame_optical, float)
    if (K.shape != (3, 3) or T.shape != (4, 4) or not np.isfinite(K).all()
            or not np.isfinite(T).all() or K[0, 0] <= 0 or K[1, 1] <= 0
            or not np.allclose(K[2], [0, 0, 1]) or not np.allclose(T[3], [0, 0, 0, 1])
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-5)):
        raise ValueError("Invalid pinhole intrinsics or optical-to-frame rigid transform")
    return K, T


def project(points_frame, K, T_frame_optical):
    K, T = calibration(K, T_frame_optical)
    points = np.asarray(points_frame, float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Finite Nx3 points required")
    optical = (points - T[:3, 3]) @ T[:3, :3]
    pixels_h = optical @ K.T
    pixels = np.full((len(points), 2), np.nan)
    front = optical[:, 2] > 1e-6
    pixels[front] = pixels_h[front, :2] / pixels_h[front, 2, None]
    return pixels, optical[:, 2]


def estimate_on_plane(endpoints_px, K, T_frame_optical, *, plane_normal, plane_offset_m,
                      frame_id, timestamp_ns, now_ns, confidence, image_size,
                      expected_length_m=.6, length_tolerance_m=.06, max_age_ns=150_000_000,
                      minimum_confidence=.7):
    K, T = calibration(K, T_frame_optical)
    if (not frame_id or not isinstance(timestamp_ns, int) or timestamp_ns < 0
            or not 0 <= now_ns - timestamp_ns <= max_age_ns):
        raise ValueError("Missing frame or stale/future image timestamp")
    if not minimum_confidence <= confidence <= 1:
        raise ValueError("Insufficient keypoint confidence")
    pixels = np.asarray(endpoints_px, float)
    n = np.asarray(plane_normal, float)
    if (pixels.shape != (2, 2) or n.shape != (3,) or not np.isfinite(pixels).all()
            or not np.isfinite(n).all() or not np.isfinite(plane_offset_m)
            or not np.isclose(np.linalg.norm(n), 1)):
        raise ValueError("Two finite undistorted keypoints and a unit plane normal required")
    width, height = image_size
    if (width < 16 or height < 16 or np.any(pixels < 4)
            or np.any(pixels[:, 0] > width - 5) or np.any(pixels[:, 1] > height - 5)):
        raise ValueError("Truncated bar endpoints require another observation pose")
    rays = np.linalg.solve(K, np.column_stack((pixels, np.ones(2))).T).T @ T[:3, :3].T
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    denominator = rays @ n
    if np.any(np.abs(denominator) < .05):
        raise ValueError("Near-parallel camera rays make plane intersection unreliable")
    distances = -(T[:3, 3] @ n + plane_offset_m) / denominator
    if np.any(distances <= 0) or np.any(distances > 5):
        raise ValueError("Plane intersection behind camera or outside pickup range")
    endpoints = T[:3, 3] + distances[:, None] * rays
    delta = endpoints[1] - endpoints[0]
    length = float(np.linalg.norm(delta))
    if abs(length - expected_length_m) > length_tolerance_m or length < 1e-6:
        raise ValueError("Estimated length disagrees with the configured steel bar")
    return BarEstimate(frame_id, timestamp_ns, endpoints.mean(0).tolist(),
                       (delta / length).tolist(), length, float(confidence))
