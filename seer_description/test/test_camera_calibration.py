"""Numerical calibration recovery and rendered-target integration; no ROS/camera needed."""
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from camera_calibration_core import (  # noqa: E402
    Board, BoardDetector, calibrate_views, gray8, is_duplicate, ros_camera_info,
    solve_dataset, validation_error, write_json,
)


def synthetic_views(count=32):
    rng = np.random.default_rng(93)
    detector = BoardDetector(Board())
    points = (detector.board.getChessboardCorners() if hasattr(detector.board, 'getChessboardCorners')
              else detector.board.chessboardCorners)
    k = np.array([[1200., 0., 640.], [0., 1220., 480.], [0., 0., 1.]])
    d = np.array([-.12, .03, .001, -.002, .005])
    result = []
    while len(result) < count:
        rotation = rng.uniform([-.8, -.8, -.3], [.8, .8, .3])
        translation = rng.uniform([-.4, -.3, .35], [.2, .15, .85])
        pixels, _ = cv2.projectPoints(points, rotation, translation, k, d)
        if (pixels[:, 0, 0].min() < 15 or pixels[:, 0, 0].max() > 1265 or
                pixels[:, 0, 1].min() < 15 or pixels[:, 0, 1].max() > 945):
            continue
        pixels += rng.normal(0., .01, pixels.shape)
        result.append({'objects': np.asarray(points, np.float32), 'pixels': pixels.reshape(-1, 2),
                       'ids': np.arange(len(points))})
    return result, k, d


def test_known_intrinsics_recovered_and_holdout_uses_fixed_model():
    views, expected_k, expected_d = synthetic_views()
    rms, k, d, errors = calibrate_views(views[:26], (1280, 960))
    assert rms < .06
    np.testing.assert_allclose(k, expected_k, atol=1.)
    np.testing.assert_allclose(d.reshape(-1)[:4], expected_d[:4], atol=.015)
    assert max(errors) < .07
    assert max(validation_error(v, k, d) for v in views[26:]) < .08
    wrong = k.copy()
    wrong[0, 0] *= 1.1
    assert validation_error(views[-1], wrong, d) > .08


def render_charuco(count=18):
    spec = Board()
    detector = BoardDetector(spec)
    pattern = detector.pattern(150, 0)
    height, width = pattern.shape
    source = np.float32([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]])
    board_corners = np.float32([[0, 0, 0], [.21, 0, 0], [.21, .15, 0], [0, .15, 0]])
    k = np.array([[1200., 0., 640.], [0., 1220., 480.], [0., 0., 1.]])
    rng = np.random.default_rng(9)
    for _ in range(count):
        rotation = rng.uniform([-.6, -.5, -.4], [.6, .5, .4])
        translation = rng.uniform([-.14, -.12, .5], [-.07, -.03, .7])
        target, _ = cv2.projectPoints(board_corners, rotation, translation, k, None)
        transform = cv2.getPerspectiveTransform(source, target.reshape(-1, 2))
        yield cv2.warpPerspective(pattern, transform, (1280, 960), borderValue=255), k


def make_dataset(tmp_path):
    dataset = tmp_path / 'dataset'
    dataset.mkdir()
    for split in ('train', 'validation'):
        (dataset / split).mkdir()
    write_json(dataset / 'session.json', {'board': asdict(Board()), 'image_size': [1280, 960]})
    for i, (image, _) in enumerate(render_charuco()):
        split = 'validation' if i >= 12 else 'train'
        assert cv2.imwrite(str(dataset / split / f'{i:03d}.png'), image)
    return dataset


def test_rendered_charuco_dataset_to_ros_yaml_and_previews(tmp_path):
    dataset = make_dataset(tmp_path)
    report = solve_dataset(dataset, tmp_path / 'result')
    assert len(report['training']) == 12
    assert len(report['validation']) == 6
    assert report['rms_px'] < 1.
    assert report['status'] == 'REVIEW_REQUIRED'  # insufficient count/coverage cannot silently pass
    assert abs(report['K'][0][0] - 1200) < 30
    info = yaml.safe_load((tmp_path / 'result' / 'camera_intrinsics.yaml').read_text())
    assert info['image_width'] == 1280 and info['image_height'] == 960
    assert info['distortion_coefficients']['cols'] == 5
    assert info['projection_matrix']['data'][0] == info['camera_matrix']['data'][0]
    assert list((tmp_path / 'result' / 'previews' / 'validation').glob('*.undistorted.png'))
    assert json.loads((tmp_path / 'result' / 'report.json').read_text())['status'] == 'REVIEW_REQUIRED'
    with pytest.raises(FileExistsError):
        solve_dataset(dataset, tmp_path / 'result')


def test_mixed_resolution_is_rejected(tmp_path):
    dataset = make_dataset(tmp_path)
    cv2.imwrite(str(dataset / 'validation' / 'wrong.png'), np.zeros((480, 640), np.uint8))
    with pytest.raises(ValueError, match='Mixed image sizes'):
        solve_dataset(dataset, tmp_path / 'result')


def test_holdout_leakage_is_rejected(tmp_path):
    dataset = make_dataset(tmp_path)
    (dataset / 'validation' / 'copy.png').write_bytes((dataset / 'train' / '000.png').read_bytes())
    with pytest.raises(ValueError, match='Repeated image'):
        solve_dataset(dataset, tmp_path / 'result')


def test_duplicate_pose_and_bit_depth_guards():
    views, _, _ = synthetic_views()
    assert is_duplicate(views[0], [views[0]], (1280, 960))
    assert not is_duplicate(views[-1], [views[0]], (1280, 960))
    with pytest.raises(ValueError, match='uint8'):
        gray8(np.zeros((8, 8), np.uint16))
    with pytest.raises(ValueError, match='At least 8'):
        calibrate_views(views[:7], (1280, 960))


def test_board_and_chessboard_corner_conventions():
    with pytest.raises(ValueError):
        Board(square_m=-1)
    with pytest.raises(ValueError):
        Board(marker_m=.04)
    detector = BoardDetector(Board(kind='chessboard'))
    image = detector.pattern(80, 100)
    detection = detector.detect(image)
    assert len(detection['pixels']) == 7*5
    assert image.shape == (6*80+200, 8*80+200)


def test_ros_schema_has_rectification_and_projection():
    k = np.array([[1200., 0, 640.], [0, 1220., 480.], [0, 0, 1.]])
    info = ros_camera_info((1280, 960), k, np.zeros(5), 'hik')
    assert info['distortion_model'] == 'plumb_bob'
    assert info['rectification_matrix']['data'] == np.eye(3).reshape(-1).tolist()
    assert info['projection_matrix']['cols'] == 4


def test_capture_headless_preserves_native_images_and_metadata(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('capture_calibration_test', SCRIPTS / 'capture_camera_calibration.py')
    capture = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(capture)
    images = list(render_charuco(5))

    class Source:
        metadata = {'source': 'synthetic_test'}
        closed = False

        def __init__(self, *_args):
            self.index = 0

        def read(self):
            image, _ = images[self.index % len(images)]
            self.index += 1
            return image, {'frame_number': self.index}

        def close(self):
            Source.closed = True

    monkeypatch.setattr(capture, 'MvsSource', Source)
    board = tmp_path / 'board.json'
    write_json(board, asdict(Board()))
    dataset = tmp_path / 'captured'
    assert capture.main(['--board-config', str(board), '--output', str(dataset), '--headless',
                         '--count', '5', '--every', '.001']) == 0
    session = json.loads((dataset / 'session.json').read_text())
    assert session['image_size'] == [1280, 960]
    assert session['saved'] == {'train': 4, 'validation': 1}
    assert Source.closed
    np.testing.assert_array_equal(cv2.imread(str(dataset / 'train' / '0001.png'), 0), images[0][0])
    assert len((dataset / 'frames.jsonl').read_text().splitlines()) == 5
