#!/usr/bin/env python3
"""Measure settled rebar motion in chassis and world frames using live TF.

Observe after placement/retraction. Chassis-relative motion distinguishes
rolling in the saddles from the previously documented mobile-base drift.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


def pose(transform):
    p, q = transform.transform.translation, transform.transform.rotation
    return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]


def summarize(rows):
    result = {'samples': len(rows),
              'simulation_seconds': rows[-1]['stamp']-rows[0]['stamp'],
              'last_base_position': rows[-1]['base_position']}
    for frame in ('base', 'world'):
        spans = [max(r[frame+'_position'][i] for r in rows) -
                 min(r[frame+'_position'][i] for r in rows) for i in range(3)]
        q0 = rows[0][frame+'_quat']
        angles = []
        for row in rows:
            q = row[frame+'_quat']
            dot = abs(sum(a*b for a, b in zip(q, q0)))/math.sqrt(
                sum(a*a for a in q)*sum(a*a for a in q0))
            angles.append(math.degrees(2*math.acos(min(1., dot))))
        result[frame+'_span_mm'] = [v*1000 for v in spans]
        result[frame+'_max_rotation_deg'] = max(angles)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30,
                        help='Wall-clock observation window; report also records simulation time')
    parser.add_argument('--output', type=Path, default=Path('/tmp/rebar_settling.json'))
    parser.add_argument('--bars', nargs='+', default=[
        'rebar', 'loaded_rebar_2', 'loaded_rebar_3', 'loaded_rebar_4'])
    parser.add_argument('--max-span-mm', type=float, default=.1)
    parser.add_argument('--max-angle-deg', type=float, default=.25)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init()
    node = rclpy.create_node('measure_rebar_settling')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples, stamps = {}, {}
    end = time.monotonic()+args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            for name in args.bars:
                try:
                    base = buffer.lookup_transform('base_link', name, Time())
                    # Use the same timestamp to avoid comparing different frames.
                    world = buffer.lookup_transform('world', name, Time.from_msg(base.header.stamp))
                except TransformException:
                    continue
                stamp = base.header.stamp.sec+base.header.stamp.nanosec*1e-9
                if stamps.get(name) == stamp:
                    continue
                stamps[name] = stamp
                bp, bq = pose(base)
                wp, wq = pose(world)
                samples.setdefault(name, []).append({
                    'stamp': stamp, 'base_position': bp, 'base_quat': bq,
                    'world_position': wp, 'world_quat': wq})
        report = {'wall_seconds': args.seconds, 'bars': {}}
        for name, rows in samples.items():
            result = summarize(rows)
            result['pass'] = (len(rows) >= 10 and
                              max(result['base_span_mm']) <= args.max_span_mm and
                              result['base_max_rotation_deg'] <= args.max_angle_deg)
            report['bars'][name] = result
        report['missing_bars'] = sorted(set(args.bars)-set(samples))
        report['pass'] = (not report['missing_bars'] and
                          all(r['pass'] for r in report['bars'].values()))
        report['limits'] = {'relative_span_mm': args.max_span_mm,
                            'relative_rotation_deg': args.max_angle_deg}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
        return 0 if report['pass'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
