#!/usr/bin/env python3
"""Summarize eight mission reports and plot planned versus measured arm motion.

Run locally with uv after downloading the report directory from Ubuntu.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from offline_rebar_model import ARM, RobotModel


def path_metrics(model, joints):
    poses = np.array([model.tcp(q) for q in joints])
    rotations = Rotation.from_matrix(poses[:, :3, :3])
    angles = (rotations[:-1].inv() * rotations[1:]).magnitude()
    return poses[:, :3, 3], {
        'tcp_length_m': float(np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1).sum()),
        'orientation_arc_deg': float(np.degrees(angles.sum())),
        'joint_total_travel_rad': float(np.abs(np.diff(joints, axis=0)).sum()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result_dir', type=Path)
    args = parser.parse_args()
    directory = args.result_dir
    reports = [json.loads((directory / f'{stage:02}.json').read_text()) for stage in range(1, 9)]
    counts = [{'stage': f'{i:02}', 'checks': len(report),
               'passed': sum(row['pass'] is True for row in report)}
              for i, report in enumerate(reports, 1)]
    if any(row['checks'] != row['passed'] for row in counts):
        raise RuntimeError('workflow contains failed checks')
    trajectory = json.loads((directory / '04.trajectory.json').read_text())
    motion = json.loads((directory / '04.motion.json').read_text())
    if len(motion) < 2:
        raise RuntimeError('insufficient measured motion feedback')
    model = RobotModel()
    planned = np.array([row[0] for row in trajectory['rows']])
    actual = np.array([row['joints_rad'] for row in motion])
    tcp_plan, plan_metrics = path_metrics(model, planned)
    tcp_live, live_metrics = path_metrics(model, actual)
    observed_bar = np.array([row['rebar_base'] for row in motion])
    times = np.array([row['elapsed_s'] for row in motion])
    planned_times = .2 + np.linspace(0, trajectory['duration_s'], len(planned))
    reference = np.stack([np.interp(times, planned_times, planned[:, i]) for i in range(6)], axis=1)
    summary = {
        'status': 'eight_stages_passed_in_isaac',
        'stage_checks': counts, 'total_checks_passed': sum(row['passed'] for row in counts),
        'planned_duration_s': trajectory['duration_s'], 'feedback_samples': len(motion),
        'planned': plan_metrics, 'feedback_fk': live_metrics,
        'max_feedback_joint_error_rad': float(np.abs(actual-reference).max()),
        'max_bar_to_nominal_tcp_offset_m': float(np.linalg.norm(observed_bar-tcp_live, axis=1).max()),
        'baseline_commanded_orientation_lower_bound_deg': 270,
        'planned_orientation_reduction_from_baseline_lower_bound_percent':
            100*(1-plan_metrics['orientation_arc_deg']/270),
        'handoff_checks': reports[-1],
        'note': 'Feedback FK uses nominal URDF TCP; steel center is measured separately. '
                'Feedback path length includes tracking noise. Baseline is a commanded waypoint '
                'lower bound, not a recorded old trajectory or a timing comparison. '
                'One completed trial does not establish a success rate.',
    }
    (directory / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    for ax, coords, label in [(axes[0], (0, 2), 'X / Z'), (axes[1], (0, 1), 'X / Y')]:
        a, b = coords
        ax.plot(tcp_plan[:, a], tcp_plan[:, b], label='Planned', linewidth=2)
        ax.plot(tcp_live[:, a], tcp_live[:, b], '--', label='Feedback FK', linewidth=1.2)
        ax.scatter([tcp_plan[0, a], tcp_plan[-1, a]], [tcp_plan[0, b], tcp_plan[-1, b]], s=25)
        ax.set(xlabel=f'{label[0]} (m)', ylabel=f'{label[-1]} (m)', title=f'TCP path: {label}')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.25)
        ax.legend()
    for i, name in enumerate(ARM):
        line, = axes[2].plot(planned_times, np.degrees(planned[:, i]), label=name)
        axes[2].plot(times, np.degrees(actual[:, i]), '--', color=line.get_color(), linewidth=.9)
    axes[2].set(xlabel='Elapsed wall time (s)', ylabel='Joint angle (degrees)',
                title='Solid: planned; dashed: feedback')
    axes[2].grid(alpha=.25)
    axes[2].legend(fontsize=7)
    fig.savefig(directory / 'execution_comparison.png', dpi=160)
    plt.close(fig)
    print(json.dumps({key: summary[key] for key in
                     ('status', 'total_checks_passed', 'planned', 'feedback_fk')}, indent=2))


if __name__ == '__main__':
    main()
