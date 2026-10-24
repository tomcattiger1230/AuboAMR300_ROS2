import numpy as np
import pytest
from rebar_training.vision.geometry import estimate_on_plane, project


def inputs():
    K = np.array([[800, 0, 512], [0, 800, 307.5], [0, 0, 1.]])
    T = np.eye(4)
    T[:3, :3] = np.diag([1., -1., -1.])  # downward-facing ROS optical frame
    T[:3, 3] = [.1, -.2, 1.5]
    endpoints = np.array([[-.2, .1, .522], [.4, .1, .522]])
    pixels, depth = project(endpoints, K, T)
    assert (depth > 0).all()
    return pixels, K, T


def estimate(pixels, K, T, **changes):
    kwargs = dict(plane_normal=[0, 0, 1], plane_offset_m=-.522, frame_id="base_footprint",
                  timestamp_ns=100_000_000, now_ns=133_333_333, confidence=.95, image_size=(1024, 615))
    kwargs.update(changes)
    return estimate_on_plane(pixels, K, T, **kwargs)


def test_mono_plane_geometry_recovers_metric_bar_without_depth_or_simulator():
    pixels, K, T = inputs()
    result = estimate(pixels, K, T)
    assert result.position_m == pytest.approx([.1, .1, .522])
    assert result.axis == pytest.approx([1, 0, 0])
    assert result.length_m == pytest.approx(.6)
    assert result.axis_sign_ambiguous
    reverse = estimate(pixels[::-1], K, T)
    assert reverse.position_m == pytest.approx(result.position_m)
    assert reverse.axis == pytest.approx([-1, 0, 0])


@pytest.mark.parametrize("changes", [{"now_ns": 400_000_000}, {"now_ns": 0}, {"confidence": .3},
                                     {"plane_offset_m": -1.7}, {"expected_length_m": .3}])
def test_stale_low_confidence_or_inconsistent_geometry_is_rejected(changes):
    with pytest.raises(ValueError):
        estimate(*inputs(), **changes)


def test_truncated_endpoints_are_not_interpreted_as_the_bar_center():
    pixels, K, T = inputs()
    pixels[0, 1] = 0
    with pytest.raises(ValueError, match="Truncated"):
        estimate(pixels, K, T)


def test_invalid_rotation_and_near_parallel_rays_are_rejected():
    pixels, K, T = inputs()
    bad = T.copy(); bad[0, 0] = -1
    with pytest.raises(ValueError, match="rigid transform"):
        estimate(pixels, K, bad)
    with pytest.raises(ValueError, match="parallel"):
        estimate(np.array([[512., 307.5], [512., 308.]]), K, T,
                 plane_normal=[1, 0, 0], plane_offset_m=-.5)
