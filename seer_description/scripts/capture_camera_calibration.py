#!/usr/bin/env python3
"""Capture native-resolution PNG calibration images from Hikrobot MVS or ROS 2."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import cv2

from camera_calibration_core import Board, BoardDetector, gray8, is_duplicate, quality, utc_now, write_json
from camera_calibration_sources import MvsSource, PreviewSource, RosSource


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=('mvs', 'ros', 'preview'), default='mvs')
    parser.add_argument('--preview-url', default='http://127.0.0.1:8080')
    parser.add_argument('--sdk-path', help='Directory containing MvCameraControl_class.py (MvImport)')
    parser.add_argument('--serial')
    parser.add_argument('--list-devices', action='store_true', help='Enumerate USB cameras without opening them')
    parser.add_argument('--exposure-us', type=float)
    parser.add_argument('--gain-db', type=float)
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--board-config', type=Path)
    parser.add_argument('--output', type=Path, help='New dataset directory; existing paths are never overwritten')
    parser.add_argument('--notes', default='', help='Lens, focus, aperture, lighting, camera settings')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--count', type=int, default=40, help='Automatic mode: total accepted images')
    parser.add_argument('--every', type=float, default=2., help='Automatic capture interval (seconds)')
    parser.add_argument('--validation-every', type=int, default=5)
    parser.add_argument('--min-sharpness', type=float, default=20.)
    parser.add_argument('--min-coverage', type=float, default=.015)
    parser.add_argument('--duplicate-threshold', type=float, default=.015, help='Corner displacement/image diagonal')
    parser.add_argument('--idle-timeout', type=float, default=300., help='Abort if no image can be saved for this long')
    parser.add_argument('--frame-timeout', type=float, default=10., help='Abort on camera stream loss')
    args = parser.parse_args(argv)
    numeric_options = (args.every, args.min_sharpness, args.min_coverage, args.duplicate_threshold,
                       args.idle_timeout, args.frame_timeout, args.exposure_us, args.gain_db)
    if any(value is not None and not math.isfinite(value) for value in numeric_options):
        parser.error('Numeric options must be finite')
    if args.list_devices and args.source != 'mvs':
        parser.error('--list-devices requires --source mvs')
    if not args.list_devices and (args.board_config is None or args.output is None):
        parser.error('--board-config and --output are required for capture')
    if (args.count < 1 or args.every <= 0 or args.validation_every < 2 or args.idle_timeout <= 0 or
            args.frame_timeout <= 0 or args.min_sharpness < 0 or not 0 <= args.min_coverage <= 1 or
            not 0 < args.duplicate_threshold < 1 or
            (args.exposure_us is not None and args.exposure_us <= 0) or
            (args.gain_db is not None and args.gain_db < 0)):
        parser.error('Invalid count, interval, timeout or quality threshold')
    if args.source != 'mvs' and (args.exposure_us is not None or args.gain_db is not None or args.serial):
        parser.error('--exposure-us, --gain-db and --serial are MVS-only; configure the ROS driver directly')
    source = None
    window = False
    try:
        if not args.list_devices:
            detector = BoardDetector(Board.load(args.board_config))
            args.output.mkdir(parents=True, exist_ok=False)
            for split in ('train', 'validation'):
                (args.output / split).mkdir()
        if args.source == 'mvs':
            source = MvsSource(args.sdk_path, args.serial, args.exposure_us, args.gain_db, args.list_devices)
        elif args.source == 'ros':
            source = RosSource(args.topic)
        else:
            source = PreviewSource(args.preview_url, args.every)
        if args.list_devices:
            print(json.dumps(source.available_devices, indent=2))
            return 0
        session = {'created_utc': utc_now(), 'board': asdict(detector.spec), 'camera': source.metadata,
                   'notes': args.notes, 'opencv_version': cv2.__version__, 'image_size': None,
                   'capture_arguments': vars(args).copy(), 'saved': {'train': 0, 'validation': 0}}
        session['capture_arguments'] = {k: str(v) if isinstance(v, Path) else v
                                        for k, v in session['capture_arguments'].items()}
        write_json(args.output / 'session.json', session)
        if not args.headless:
            cv2.namedWindow('Camera calibration: s=train v=validation a=auto q=quit', cv2.WINDOW_NORMAL)
            window = True
            cv2.resizeWindow('Camera calibration: s=train v=validation a=auto q=quit', 1200, 800)
        automatic = args.headless
        previous = []
        last_save = last_frame = time.monotonic()
        last_attempt = 0.
        last_log = 0.
        saved = 0
        while True:
            result = source.read()
            now = time.monotonic()
            if now - last_save > args.idle_timeout:
                raise RuntimeError('No accepted captures within idle timeout; check target, lighting and pose diversity')
            if result is None:
                if now-last_frame > args.frame_timeout:
                    raise RuntimeError('Camera stream timeout; check USB cable, trigger mode and ROS topic')
                if window and cv2.waitKey(1) & 0xff in (ord('q'), 27):
                    break
                continue
            last_frame = now
            image, metadata = result
            gray = gray8(image)
            size = [gray.shape[1], gray.shape[0]]
            if session['image_size'] is None:
                session['image_size'] = size
                write_json(args.output / 'session.json', session)
                print(f'Native image: {size[0]}x{size[1]}; {source.metadata}', flush=True)
            elif size != session['image_size']:
                raise RuntimeError('Image dimensions changed during session; start a new dataset')
            detection = detector.detect(gray)
            stats = quality(gray, detection) if detection is not None else None
            reason = ''
            if stats is None or stats['corners'] < 12:
                reason = 'Need >=12 board corners'
            elif stats['sharpness'] < args.min_sharpness:
                reason = 'Blurred board: adjust focus/exposure'
            elif stats['coverage'] < args.min_coverage:
                reason = 'Board too small in image'
            elif is_duplicate(detection, previous, size, args.duplicate_threshold):
                reason = 'Similar view: move/tilt board'
            key = 255
            if window:
                scale = min(1., 1200 / gray.shape[1])
                display = cv2.cvtColor(cv2.resize(gray, None, fx=scale, fy=scale), cv2.COLOR_GRAY2BGR)
                if detection is not None:
                    for x, y in detection['pixels'] * scale:
                        cv2.circle(display, (int(x), int(y)), 3, (0, 255, 0), 1)
                text = f'{saved}/{args.count} train={session["saved"]["train"]} val={session["saved"]["validation"]} '
                text += ('AUTO ' if automatic else 'MANUAL ') + (reason or 'READY')
                cv2.putText(display, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 200, 255), 1)
                cv2.imshow('Camera calibration: s=train v=validation a=auto q=quit', display)
                key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            if key == ord('a'):
                automatic = not automatic
                print('Automatic capture: ' + str(automatic), flush=True)
            due = automatic and now-last_attempt >= args.every
            if key not in (ord('s'), ord('v')) and not due:
                continue
            last_attempt = now
            if reason:
                if now-last_log >= args.every:
                    print('SKIP: ' + reason, flush=True)
                    last_log = now
                continue
            if key in (ord('s'), ord('v')):
                split = 'validation' if key == ord('v') else 'train'
            else:
                split = 'validation' if (saved+1) % args.validation_every == 0 else 'train'
            relative = Path(split) / f'{saved+1:04d}.png'
            if not cv2.imwrite(str(args.output / relative), gray):
                raise OSError('Failed to write raw PNG')
            record = {'file': str(relative), 'split': split, 'saved_utc': utc_now(),
                      'image_size': size, 'frame': metadata, 'quality': stats}
            with (args.output / 'frames.jsonl').open('a') as stream:
                stream.write(json.dumps(record, allow_nan=False) + '\n')
            previous.append(detection)
            saved += 1
            session['saved'][split] += 1
            session['updated_utc'] = utc_now()
            write_json(args.output / 'session.json', session)
            last_save = time.monotonic()
            print(f'SAVED {relative}: {stats}', flush=True)
            if saved >= args.count:
                break
        print(f'Dataset: {args.output}; train={session["saved"]["train"]}, validation={session["saved"]["validation"]}')
        return 0 if saved else 2
    except KeyboardInterrupt:
        print('Capture stopped; already saved PNGs and session metadata are retained.')
        return 130
    except (ValueError, RuntimeError, OSError, cv2.error) as exc:
        parser.exit(1, f'ERROR: {exc}\n')
    finally:
        if source is not None:
            source.close()
        if window:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    raise SystemExit(main())
