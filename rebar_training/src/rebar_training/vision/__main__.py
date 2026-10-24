"""Offline camera/augmentation preparation. No simulator, remote host, or model is started."""
import argparse
import json
from pathlib import Path


def demo(output):
    import numpy as np
    from .dataset import DatasetWriter
    writer = DatasetWriter(output, {'fixture': True, 'runtime_validated': False, 'source': 'cpu_contract_fixture',
        'task': 'loading', 'nominal_fps': 10., 'observation_contract': 'loading_route_v1'})
    writer.start(0, 42)
    for index in range(6):
        image = np.zeros((48, 64), np.uint8)
        image[20:24, 10+index:42+index] = 180
        start, end = index * 100_000_000, (index+1) * 100_000_000
        action = [0.] * 7
        writer.append(0, {'t_start_ns': start, 't_end_ns': end, 'action_applied': action,
            'action_history': [{'t_start_ns': start, 't_end_ns': end, 'action_applied': action}],
            'state': {'tcp': [0., 0., 1.], 'bar': [0., 0., 1.], 'arm_joint_position_rad': [0.] * 6, 'finger_position_m': [0., 0.]},
            'terminal': index == 5, 'on_capture_grid': True},
            {'wrist': {'image': image, 'sensor_frame': index+1, 'depth_m': np.ones((48, 64)),
                'semantic_ids': (image > 0).astype(np.int32),
                'semantic_mapping': {'idToLabels': {'0': {'class': 'BACKGROUND'}, '1': {'class': 'rebar'}}},
                'calibration': {'K': [[50., 0., 32.], [0., 50., 24.], [0., 0., 1.]],
                                'T_env_optical': np.eye(4).tolist(), 'calibration_source': 'fixture'}}})
    # This flag only exercises the outcome contract; metadata.fixture prohibits
    # treating the test sequence as a physical demonstration.
    writer.finish(0, {'success': True, 'failure': False, 'timeout': False, 'picked': True, 'fixture': True})
    writer.close()
    return {'output': str(Path(output).resolve()), 'fixture': True, 'physics_validated': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('demo'); p.add_argument('--output', required=True, type=Path)
    p = commands.add_parser('validate'); p.add_argument('--dataset', required=True, type=Path)
    p = commands.add_parser('check-config'); p.add_argument('--config', required=True, type=Path)
    p = commands.add_parser('plan')
    p.add_argument('--dataset', required=True, type=Path); p.add_argument('--output', required=True, type=Path)
    p.add_argument('--prompts', required=True, type=Path); p.add_argument('--camera', default='wrist')
    p.add_argument('--control', choices=['edge', 'blur', 'depth', 'seg'], default='edge')
    p.add_argument('--model', choices=['Cosmos3-Nano', 'Cosmos3-Super'], default='Cosmos3-Nano')
    p.add_argument('--frames', type=int, default=121); p.add_argument('--allow-fixture', action='store_true')
    p = commands.add_parser('prepare-media')
    for name in ('dataset', 'plan', 'output'): p.add_argument('--'+name, required=True, type=Path)
    p.add_argument('--job-id', required=True)
    p = commands.add_parser('nim-request')
    p.add_argument('--prepared', required=True, type=Path); p.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.command == 'demo':
        result = demo(args.output)
    elif args.command == 'validate':
        from .dataset import validate
        result = validate(args.dataset)
    elif args.command == 'check-config':
        from .capture import load_config
        result = {'valid': True, 'config': load_config(args.config), 'gpu_validated': False}
    else:
        from .augment import plan, prepare_media, nim_request
        if args.command == 'plan':
            result = plan(args.dataset, args.output, args.camera, json.loads(args.prompts.read_text())['prompts'],
                          args.frames, args.control, args.model, args.allow_fixture)
            result = {'jobs': len(result['jobs']), 'path': str(args.output / 'jobs.json'), 'generated': False}
        elif args.command == 'prepare-media':
            result = prepare_media(args.dataset, args.plan, args.job_id, args.output)
            result.pop('calibration')
        else:
            result = nim_request(args.prepared, args.output)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
