#!/usr/bin/env python3
"""Generate a calibration target or solve a captured monocular dataset."""
import argparse
from dataclasses import asdict
import math
from pathlib import Path

import cv2

from camera_calibration_core import Board, BoardDetector, solve_dataset, write_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    board = commands.add_parser('board', help='Generate PNG target and its board.json specification')
    board.add_argument('--board-config', type=Path, required=True)
    board.add_argument('--output', type=Path, required=True, help='New directory')
    board.add_argument('--pixels-per-square', type=int, default=300)
    solve = commands.add_parser('solve', help='Fit training images and evaluate held-out images')
    solve.add_argument('--dataset', type=Path, required=True)
    solve.add_argument('--output', type=Path, required=True, help='New directory')
    solve.add_argument('--max-rms', type=float, default=.5)
    solve.add_argument('--max-view-error', type=float, default=1.)
    solve.add_argument('--camera-name', default='mv_ch100_60um')
    args = parser.parse_args(argv)
    try:
        if args.command == 'board':
            if args.pixels_per_square < 40:
                raise ValueError('pixels-per-square must be >=40')
            spec = Board.load(args.board_config)
            image = BoardDetector(spec).pattern(args.pixels_per_square)
            args.output.mkdir(parents=True, exist_ok=False)
            if not cv2.imwrite(str(args.output / 'board.png'), image):
                raise OSError('Failed to write target')
            write_json(args.output / 'board.json', asdict(spec))
            columns = spec.columns + (spec.kind == 'chessboard')
            rows = spec.rows + (spec.kind == 'chessboard')
            print(f'Pattern area excluding white margin: {columns*spec.square_m*1000:g} x '
                  f'{rows*spec.square_m*1000:g} mm. Print to this measured size; PNG has no print-scale guarantee.')
            print(args.output / 'board.png')
            return 0
        if not all(math.isfinite(x) and x > 0 for x in (args.max_rms, args.max_view_error)):
            raise ValueError('Error thresholds must be positive')
        report = solve_dataset(args.dataset, args.output, args.max_rms, args.max_view_error, args.camera_name)
        print(f"{report['status']}: RMS={report['rms_px']:.4f} px; "
              f"train={len(report['training'])}, validation={len(report['validation'])}")
        for warning in report['warnings']:
            print('REVIEW: ' + warning)
        print(args.output / 'camera_intrinsics.yaml')
        print(args.output / 'report.json')
        return 0 if report['status'] == 'PASS' else 2
    except (ValueError, RuntimeError, OSError, cv2.error) as exc:
        parser.exit(1, f'ERROR: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
