"""Independent projective-image/FK fixtures, not physical accuracy validation."""
from dataclasses import replace
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rebar_training.fiducial_localization import (
    CameraCalibration, Limits, MarkerAnchor, RelocalizationSession, WristMarkerEstimator,
    fixture_loading_geometry, inverse, rigid, rotation_distance, square_points)
from rebar_training.workcells import WorkcellLayout


def transform(xyz=(0, 0, 0), angles=(0, 0, 0)):
    t = np.eye(4); t[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix(); t[:3, 3] = xyz
    return t


def pixels(t, k, length=.16):
    return cv2.projectPoints(square_points(length), cv2.Rodrigues(t[:3, :3])[0],
                            t[:3, 3], k, None)[0].reshape(4, 2)


def marker_image(t, k, marker_id=11, length=.16):
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100), marker_id, 512)
    h = cv2.getPerspectiveTransform(np.float32([[0, 0], [511, 0], [511, 511], [0, 511]]),
                                   pixels(t, k, length).astype(np.float32))
    return cv2.warpPerspective(marker, h, (1280, 960), flags=cv2.INTER_NEAREST, borderValue=255)


@pytest.fixture
def setup():
    k = np.array([[1000., 0, 640], [0, 1000, 480], [0, 0, 1]])
    optical_from_marker = transform((.03, .02, .8), (math.pi-.25, .15, .06))
    base_from_flange = transform((.35, -.1, .9), (.2, -.4, .15))
    flange_from_optical = transform((.05, .02, .06), (.1, -.3, .2))
    # Survey fixture and marker independently from a known pose, not production inversion code.
    base_from_fixture = transform((-.3, -.77, 1.508))
    fixture_from_marker = np.linalg.inv(base_from_fixture) @ base_from_flange @ flange_from_optical @ optical_from_marker
    c = CameraCalibration('wrist_mono', 'intrinsics_v1', 'handeye_v1', (1280, 960),
                          k, (0.,)*5, flange_from_optical, 'control_base', 'flange', 'wrist_optical', 'exposure_common_clock')
    a = MarkerAnchor('machine_2', 'DICT_5X5_100', 11, .16, fixture_from_marker, 'survey_v1')
    e = WristMarkerEstimator(c, a)
    image = marker_image(optical_from_marker, k)
    kwargs = dict(machine_id='machine_2', frame_id='frame_1', image_stamp_s=1., fk_stamp_s=1.,
                  now_s=1.05, base_from_flange=base_from_flange, camera_id=c.camera_id,
                  calibration_id=c.calibration_id, hand_eye_id=c.hand_eye_id,
                  fk_base_frame_id=c.base_frame_id, fk_flange_frame_id=c.flange_frame_id,
                  optical_frame_id=c.optical_frame_id, clock_id=c.clock_id)
    return dict(e=e, c=c, a=a, image=image, kwargs=kwargs, optical=optical_from_marker,
                base_fixture=base_from_fixture, k=k)


def test_real_aruco_detector_and_pnp_on_projected_code(setup):
    s = setup; r = s['e'].estimate(s['image'], **s['kwargs'])
    actual = np.array(r['base_from_fixture'])
    # Rasterized synthetic target: broad bounds, not a hardware precision claim.
    assert np.linalg.norm(actual[:3, 3]-s['base_fixture'][:3, 3]) < .015
    assert rotation_distance(actual, s['base_fixture']) < math.radians(3)
    assert r['marker_id'] == 11 and not r['global_map_pose_corrected']
    assert not r['actuator_dispatch'] and not r['physical_acceptance']


def test_exact_corner_pose_does_not_reverse_marker_camera_transform(setup):
    s = setup; recovered, choices = s['e'].solve_corners(pixels(s['optical'], s['k']))
    np.testing.assert_allclose(recovered, s['optical'], atol=1e-8)
    assert len(choices) == 2
    wrong = np.linalg.inv(recovered)
    assert np.linalg.norm(wrong[:3, 3]-s['optical'][:3, 3]) > .1


def test_dynamic_wrist_fk_recovers_same_stationary_fixture(setup):
    s = setup
    for angles in ((.2, -.4, .15), (.22, -.38, .14), (.18, -.42, .16)):
        b_f = transform((.35, -.1, .9), angles)
        c_m = np.linalg.inv(b_f @ np.array(s['c'].flange_from_optical)) @ s['base_fixture'] @ np.array(s['a'].fixture_from_marker)
        recovered, _ = s['e'].solve_corners(pixels(c_m, s['k']))
        actual = b_f @ np.array(s['c'].flange_from_optical) @ recovered @ np.linalg.inv(s['a'].fixture_from_marker)
        np.testing.assert_allclose(actual, s['base_fixture'], atol=1e-8)


@pytest.mark.parametrize('change', [dict(machine_id='machine_3'), dict(camera_id='other_camera'),
    dict(calibration_id='old_intrinsics'), dict(hand_eye_id='old_mount'),
    dict(image_stamp_s=.5), dict(image_stamp_s=1.1), dict(fk_stamp_s=.95),
    dict(fk_stamp_s=1.1), dict(image_stamp_s=float('nan')),
    dict(fk_base_frame_id='wrong_base'), dict(fk_flange_frame_id='tcp'),
    dict(optical_frame_id='opengl_camera'), dict(clock_id='other_clock')])
def test_identity_and_exposure_fk_gates(setup, change):
    s = setup
    with pytest.raises(ValueError):
        s['e'].estimate(s['image'], **(s['kwargs'] | change))


def test_other_code_duplicate_code_blank_resized_and_wrong_encoding_rejected(setup):
    s = setup; image = s['image']; other = marker_image(s['optical'], s['k'], marker_id=12)
    # A second same-ID marker in a distinct image region is ambiguous ownership.
    second = marker_image(transform((-.3, -.2, .8), (math.pi-.25, .15, .06)), s['k'])
    for value in (other, np.minimum(image, second), np.full_like(image, 255), image[::2, ::2],
                  image.astype(np.uint16), np.repeat(image[:, :, None], 3, axis=2)):
        with pytest.raises(ValueError):
            s['e'].estimate(value, **s['kwargs'])


def test_material_planar_ambiguity_is_not_silently_selected(setup):
    s = setup
    # Both candidates are retained by IPPE; these broad gap settings deliberately
    # exercise refusal whenever distinct valid solutions cannot be distinguished.
    e = WristMarkerEstimator(s['c'], s['a'], replace(Limits(), ambiguity_error_gap_px=100.,
                                                ambiguity_rotation_rad=.001))
    with pytest.raises(ValueError, match='Ambiguous'):
        e.solve_corners(pixels(s['optical'], s['k']))


def test_near_frontal_noisy_view_is_rejected_with_default_ambiguity_limits(setup):
    s = setup
    truth = transform((.01, .02, .8), (math.pi-.01, .01, .02))
    noisy = pixels(truth, s['k'])+np.random.default_rng(0).normal(0, .2, (4, 2))
    with pytest.raises(ValueError, match='Ambiguous'):
        s['e'].solve_corners(noisy)


def test_wrong_scale_produces_no_candidate_in_declared_distance_range(setup):
    s = setup
    e = WristMarkerEstimator(s['c'], replace(s['a'], length_m=.8))
    with pytest.raises(ValueError):
        e.solve_corners(pixels(s['optical'], s['k']))


def test_three_fresh_exposures_required_and_invalid_sample_clears_readiness(setup):
    s = setup; observed = s['e'].estimate(s['image'], **s['kwargs'])
    goal = observed['fixture_from_base']
    session = RelocalizationSession(s['e'], session_id='dock_2_attempt_1', issued_s=.9, fixture_from_base_goal=goal)
    for n in range(3):
        kwargs = s['kwargs'] | dict(frame_id=f'frame_{n}', image_stamp_s=1.+n*.04,
                                    fk_stamp_s=1.+n*.04, now_s=1.02+n*.04)
        r = session.observe(s['image'], measured_base_twist=(0., 0.), base_stopped_confirmed=True, **kwargs)
        assert r['state'] == ('docked_geometry_candidate' if n == 2 else 'collecting')
        assert not r['loading_authorized']
    with pytest.raises(ValueError, match='Repeated'):
        session.observe(s['image'], measured_base_twist=(0., 0.), base_stopped_confirmed=True, **kwargs)
    assert session.frames == []
    assert session.invalidated


def test_session_timeout_cannot_be_satisfied_by_a_new_but_late_frame(setup):
    s = setup; session = RelocalizationSession(s['e'], session_id='dock_2', issued_s=.9,
                                             fixture_from_base_goal=inverse(s['base_fixture']))
    kwargs = s['kwargs'] | dict(now_s=12., image_stamp_s=11.99, fk_stamp_s=11.99)
    with pytest.raises(ValueError, match='deadline'):
        session.observe(s['image'], measured_base_twist=(0., 0.), base_stopped_confirmed=True, **kwargs)


@pytest.mark.parametrize('change', [dict(measured_base_twist=(.02, 0.)),
    dict(measured_base_twist=(0., .02)), dict(base_stopped_confirmed=False)])
def test_motion_invalidates_pose_session(setup, change):
    s = setup; session = RelocalizationSession(s['e'], session_id='dock_2', issued_s=.9,
                                             fixture_from_base_goal=inverse(s['base_fixture']))
    with pytest.raises(ValueError, match='stopped'):
        session.observe(s['image'], **(dict(measured_base_twist=(0., 0.), base_stopped_confirmed=True) | change), **s['kwargs'])


def test_large_local_correction_does_not_become_loading_candidate(setup):
    s = setup; goal = inverse(s['base_fixture']); goal[0, 3] += .5
    session = RelocalizationSession(s['e'], session_id='dock_2', issued_s=.9, fixture_from_base_goal=goal)
    with pytest.raises(ValueError, match='outside'):
        session.observe(s['image'], measured_base_twist=(0., 0.), base_stopped_confirmed=True, **s['kwargs'])


def test_station_local_targets_match_legacy_on_all_three_planar_stations():
    layout = WorkcellLayout.load(Path(__file__).parents[1]/'config/multi_workcell_offline.json')
    for machine in ('machine_1', 'machine_2', 'machine_3'):
        station = layout.station(machine); profile = layout.profiles[station.profile_id]
        base = layout.navigation_goal(machine)
        expected = layout.machine_geometry(machine, refined_pose=station.pose_map, base_pose=base)
        map_fixture = transform(station.pose_map.point(profile.fixture_goal), (0, 0, station.pose_map.yaw))
        map_base = transform(base.values()[:3], (0, 0, base.yaw))
        actual = fixture_loading_geometry(layout, machine, np.linalg.inv(map_base) @ map_fixture)
        for key in ('goal', 'standoff', 'retreat_vector', 'bar_axis'):
            assert actual[key] == pytest.approx(expected[key], abs=1e-10)
        r = Rotation.from_quat(actual['tcp_quaternion_xyzw']).as_matrix()
        np.testing.assert_allclose(r, Rotation.from_quat(expected['tcp_quaternion_xyzw']).as_matrix(), atol=1e-10)
        assert actual['policy_compatibility_accepted'] is False


@pytest.mark.parametrize('value', [np.diag([-1., 1, 1, 1]), np.ones((4, 4)), np.eye(3),
                                 np.eye(4)*float('nan')])
def test_invalid_rigid_transforms(value):
    with pytest.raises(ValueError):
        rigid(value)


def test_limits_and_calibration_are_immutable_and_finite(setup):
    with pytest.raises(ValueError):
        replace(Limits(), max_age_s=float('nan'))
    with pytest.raises(ValueError):
        replace(setup['c'], distortion=(0., 0., 0.))
    assert isinstance(setup['a'].fixture_from_marker, tuple)
