"""Shared, ROS-independent monocular calibration and dataset helpers."""
import hashlib
import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


@dataclass(frozen=True)
class Board:
    kind: str = 'charuco'
    columns: int = 7
    rows: int = 5
    square_m: float = 0.030
    marker_m: float = 0.022
    dictionary: str = 'DICT_5X5_100'

    def __post_init__(self):
        if self.kind not in ('charuco', 'chessboard'):
            raise ValueError('kind must be charuco or chessboard')
        if min(self.columns, self.rows) < 3 or not np.isfinite(self.square_m) or self.square_m <= 0:
            raise ValueError('Board needs >=3 rows/columns and positive square_m (metres)')
        if self.kind == 'charuco' and not (0 < self.marker_m < self.square_m):
            raise ValueError('marker_m must be smaller than square_m')

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text()))


class BoardDetector:
    def __init__(self, spec):
        self.spec = spec
        self.board = None
        self.detector = None
        if spec.kind == 'charuco':
            if not hasattr(cv2, 'aruco'):
                raise RuntimeError('ChArUco requires OpenCV aruco: install opencv-contrib-python, or use chessboard')
            aruco = cv2.aruco
            if not hasattr(aruco, spec.dictionary) or not spec.dictionary.startswith('DICT_'):
                raise ValueError('Unknown ArUco dictionary: ' + spec.dictionary)
            self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, spec.dictionary))
            if hasattr(aruco, 'CharucoBoard_create'):
                self.board = aruco.CharucoBoard_create(spec.columns, spec.rows, spec.square_m,
                                                       spec.marker_m, self.dictionary)
            else:
                self.board = aruco.CharucoBoard((spec.columns, spec.rows), spec.square_m,
                                                spec.marker_m, self.dictionary)
            if hasattr(aruco, 'CharucoDetector'):
                self.detector = aruco.CharucoDetector(self.board)

    def detect(self, gray):
        if gray.ndim != 2 or gray.dtype != np.uint8:
            raise ValueError('Detection requires an 8-bit grayscale image')
        s = self.spec
        if self.board is not None:
            if self.detector is not None:
                corners, ids, _, _ = self.detector.detectBoard(gray)
            else:
                markers, marker_ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
                if marker_ids is None:
                    return None
                _, corners, ids = cv2.aruco.interpolateCornersCharuco(markers, marker_ids, gray, self.board)
            if ids is None or len(ids) < 4:
                return None
            ids = ids.reshape(-1)
            all_points = (self.board.getChessboardCorners() if hasattr(self.board, 'getChessboardCorners')
                          else self.board.chessboardCorners)
            objects = np.asarray(all_points, np.float32)[ids]
        else:
            pattern = (s.columns, s.rows)  # inner corners, not squares
            if hasattr(cv2, 'findChessboardCornersSB'):
                found, corners = cv2.findChessboardCornersSB(gray, pattern)
            else:
                found, corners = cv2.findChessboardCorners(gray, pattern)
                if found:
                    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                              (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
            if not found:
                return None
            ids = np.arange(len(corners))
            objects = np.zeros((s.columns * s.rows, 3), np.float32)
            objects[:, :2] = np.mgrid[:s.columns, :s.rows].T.reshape(-1, 2) * s.square_m
        return {'objects': objects, 'pixels': np.asarray(corners, np.float32).reshape(-1, 2), 'ids': ids}

    def pattern(self, pixels_per_square=300, margin=100):
        s = self.spec
        columns = s.columns + (s.kind == 'chessboard')
        rows = s.rows + (s.kind == 'chessboard')
        size = (columns * pixels_per_square, rows * pixels_per_square)
        if self.board is not None:
            pattern = (self.board.generateImage(size) if hasattr(self.board, 'generateImage')
                       else self.board.draw(size))
        else:
            cells = (np.indices((rows, columns)).sum(axis=0) % 2 * 255).astype(np.uint8)
            pattern = cv2.resize(cells, size, interpolation=cv2.INTER_NEAREST)
        return cv2.copyMakeBorder(pattern, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)


def gray8(image):
    if image.dtype != np.uint8:
        raise ValueError('Only uint8 images supported; configure camera Mono8 (no silent 10/12/16-bit scaling)')
    if image.ndim == 2:
        return image
    if image.ndim == 3 and image.shape[2] in (3, 4):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY if image.shape[2] == 3 else cv2.COLOR_BGRA2GRAY)
    raise ValueError('Expected mono8, BGR8 or BGRA8 image')


def quality(gray, detection):
    pixels = detection['pixels']
    x, y, w, h = cv2.boundingRect(pixels)
    roi = gray[max(0, y):min(gray.shape[0], y+h), max(0, x):min(gray.shape[1], x+w)]
    return {'corners': len(pixels), 'sharpness': float(cv2.Laplacian(roi, cv2.CV_64F).var()),
            'coverage': float(cv2.contourArea(cv2.convexHull(pixels)) / gray.size)}


def is_duplicate(detection, previous, image_size, threshold=.015):
    diagonal = np.linalg.norm(image_size)
    for other in previous:
        _, a, b = np.intersect1d(detection['ids'], other['ids'], return_indices=True)
        if len(a) >= 8:
            displacement = np.sqrt(np.mean(np.sum((detection['pixels'][a] - other['pixels'][b])**2, axis=1)))
            if displacement / diagonal < threshold:
                return True
    return False


def calibrate_views(views, image_size):
    if len(views) < 8:
        raise ValueError('At least 8 usable training views required; collect 30-40 diverse views')
    rms, k, d, rotations, translations = cv2.calibrateCamera(
        [v['objects'] for v in views], [v['pixels'].reshape(-1, 1, 2) for v in views],
        tuple(image_size), None, None)
    if not np.isfinite(rms) or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)):
        raise ValueError('Calibration diverged; collect views with more tilt and position diversity')
    errors = []
    for view, rotation, translation in zip(views, rotations, translations):
        projected, _ = cv2.projectPoints(view['objects'], rotation, translation, k, d)
        errors.append(float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-view['pixels'])**2, axis=1)))))
    return float(rms), k, d, errors


def validation_error(view, k, d):
    # Only estimate the board pose. Intrinsics/distortion stay fixed on holdout images.
    ok, rotation, translation = cv2.solvePnP(view['objects'], view['pixels'], k, d)
    if not ok:
        raise ValueError('solvePnP failed for validation view')
    projected, _ = cv2.projectPoints(view['objects'], rotation, translation, k, d)
    residual = projected.reshape(-1, 2) - view['pixels']
    error = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    if not np.isfinite(error):
        raise ValueError('Non-finite validation error')
    return error


def ros_camera_info(image_size, k, d, camera_name):
    def matrix(value):
        return {'rows': value.shape[0], 'cols': value.shape[1], 'data': value.reshape(-1).tolist()}
    projection = np.column_stack((k, np.zeros(3)))
    return {'image_width': image_size[0], 'image_height': image_size[1], 'camera_name': camera_name,
            'camera_matrix': matrix(k), 'distortion_model': 'plumb_bob',
            'distortion_coefficients': matrix(d.reshape(1, -1)),
            'rectification_matrix': matrix(np.eye(3)), 'projection_matrix': matrix(projection)}


def solve_dataset(dataset, output, max_rms=.5, max_view_error=1., camera_name='mv_ch100_60um'):
    dataset, output = Path(dataset), Path(output)
    session = json.loads((dataset / 'session.json').read_text())
    spec = Board(**session['board'])
    detector = BoardDetector(spec)
    image_size = tuple(session['image_size']) if session.get('image_size') else None
    views = {'train': [], 'validation': []}
    rejected = []
    hashes = set()
    for split in views:
        for path in sorted((dataset / split).glob('*')):
            if path.suffix.lower() not in ('.png', '.bmp', '.tif', '.tiff', '.jpg', '.jpeg'):
                continue
            image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            if image is None:
                raise ValueError('Unreadable image: ' + str(path))
            gray = gray8(image)
            size = (gray.shape[1], gray.shape[0])
            if image_size is None:
                image_size = size
            if size != image_size:
                raise ValueError(f'Mixed image sizes: {path}: {size} != {image_size}; do not resize calibration images')
            digest = hashlib.sha256(gray.tobytes()).hexdigest()
            if digest in hashes:
                raise ValueError('Repeated image in dataset (including holdout leakage): ' + str(path))
            hashes.add(digest)
            detection = detector.detect(gray)
            if detection is None or len(detection['ids']) < 12:
                rejected.append({'file': str(path.relative_to(dataset)), 'reason': 'fewer than 12 corners'})
                continue
            detection.update(file=str(path.relative_to(dataset)), quality=quality(gray, detection))
            views[split].append(detection)
    if image_size is None:
        raise ValueError('Dataset has no images')
    rms, k, d, errors = calibrate_views(views['train'], image_size)
    validation = []
    for view in views['validation']:
        validation.append({'file': view['file'], 'rms_px': validation_error(view, k, d)})
    warnings = []
    if len(views['train']) < 25:
        warnings.append('Fewer than 25 training views; collect 30-40 diverse views')
    if len(validation) < 5:
        warnings.append('Fewer than 5 holdout views; independent validation incomplete')
    if not (0 < k[0, 2] < image_size[0] and 0 < k[1, 2] < image_size[1]):
        warnings.append('Principal point outside image; check board geometry and pose diversity')
    # Coverage is diagnostic, not a substitute for tilted poses or physical validation.
    all_pixels = np.concatenate([v['pixels'] for v in views['train']])
    occupied = sorted(set((min(2, int(y*3/image_size[1])), min(2, int(x*3/image_size[0])))
                          for x, y in all_pixels))
    if len(occupied) < 9:
        warnings.append('Training corners do not cover all 9 image regions; include image edges')
    tilts, distances = [], []
    for view in views['train']:
        ok, rotation, translation = cv2.solvePnP(view['objects'], view['pixels'], k, d)
        if not ok:
            raise ValueError('Failed to recover training board pose')
        normal = cv2.Rodrigues(rotation)[0][:, 2]
        tilts.append(np.degrees(np.arctan2(normal[:2], abs(normal[2]))))
        distances.append(float(translation[2, 0]))
    tilt_span = np.ptp(np.asarray(tilts), axis=0)
    if np.any(tilt_span < 15):
        warnings.append('Estimated board tilt span <15 degrees on an axis; add tilted views in both directions')
    distance_ratio = max(distances) / max(min(distances), 1e-9)
    if distance_ratio < 1.2:
        warnings.append('Estimated board distance range <1.2x; add near/far views with focus preserved')
    bad = [v['file'] for v, e in zip(views['train'], errors) if e > max_view_error]
    if bad:
        warnings.append('High-error training images; inspect overlays and recapture: ' + ', '.join(bad))
    passed = (rms <= max_rms and not bad and len(validation) >= 5 and
              all(v['rms_px'] <= max_view_error for v in validation) and not warnings)
    # Create a fresh result directory so a failed rerun cannot leave an old "good" YAML behind.
    output.mkdir(parents=True, exist_ok=False)
    yaml_path = output / 'camera_intrinsics.yaml'
    yaml_path.write_text(yaml.safe_dump(ros_camera_info(image_size, k, d, camera_name), sort_keys=False))
    report = {'created_utc': utc_now(), 'dataset': str(dataset.resolve()), 'opencv_version': cv2.__version__,
              'board': asdict(spec), 'source_metadata': session, 'image_size': list(image_size),
              'model': 'pinhole, k1 k2 p1 p2 k3', 'rms_px': rms, 'K': k.tolist(), 'D': d.reshape(-1).tolist(),
              'training': [{'file': v['file'], 'rms_px': e, 'quality': v['quality']}
                           for v, e in zip(views['train'], errors)],
              'validation': validation, 'rejected': rejected, 'occupied_image_regions_of_9': len(occupied),
              'estimated_tilt_span_degrees': tilt_span.tolist(), 'estimated_distance_ratio': distance_ratio,
              'thresholds': {'rms_px': max_rms, 'per_view_rms_px': max_view_error},
              'warnings': warnings, 'status': 'PASS' if passed else 'REVIEW_REQUIRED',
              'note': 'Image-space checks only; no claim of hand-eye or physical positioning accuracy. '
                      'No automatic outlier removal. Review YAML before use.'}
    write_json(output / 'report.json', report)
    # Same K is used for the undistorted image; P in the ROS YAML matches it.
    preview = output / 'previews'
    preview.mkdir()
    for split, selected in views.items():
        preview_split = preview / split
        preview_split.mkdir()
        for view in selected:
            image = cv2.imread(str(dataset / view['file']), cv2.IMREAD_GRAYSCALE)
            overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            for x, y in view['pixels']:
                cv2.circle(overlay, (int(x), int(y)), 5, (0, 255, 0), 2)
            label = Path(view['file']).name
            if not cv2.imwrite(str(preview_split / (label + '.corners.jpg')), overlay):
                raise OSError('Failed to write corner preview')
        for view in selected[:3]:
            image = cv2.imread(str(dataset / view['file']), cv2.IMREAD_GRAYSCALE)
            corrected = cv2.undistort(image, k, d, None, k)
            if not cv2.imwrite(str(preview_split / (Path(view['file']).name + '.undistorted.png')), corrected):
                raise OSError('Failed to write undistorted preview')
    return report
